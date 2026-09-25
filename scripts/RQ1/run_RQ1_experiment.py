#!/usr/bin/env python3
"""
run_RQ1_experiment.py — RQ1 voting-ensemble stability experiment orchestrator.

Pipeline for each (dataset, level v):
  1. Assign 3v unique voters to 3 trials (no reuse within a level).
  2. SWEEP: for each trial, run synthesis-only at all thresholds t=1..v.
            (clustering is the expensive part; it is cached after the first call.
             Synthesis at each t is cheap — 2 LLM calls per threshold per trial.)
  3. PICK:  compute Option-A F1 = harmonic(stability, completeness) per threshold,
            using embedding-based pairwise ASG coverage across the 3 trial models.
            Choose optimal t* = argmax F1(t).
  4. FINALIZE: run full Stage 4 post-processing for each trial at t*.
               Saves to trial{n}/final/V0-ensemble.json.
  5. Record voter assignments, threshold sweep scores, and cumulative costs in
     experiments/{dataset}/v{v}/threshold_sweep.json.

Usage (from RQ1/):
    python3 run_RQ1_experiment.py --dataset dronology
    python3 run_RQ1_experiment.py --dataset dronology --level 2
    python3 run_RQ1_experiment.py --dataset dronology --dry-run
    python3 run_RQ1_experiment.py --dataset dronology --smoke-test
"""

import argparse
import hashlib
import json
import math
import random
import sys
from itertools import combinations
from pathlib import Path

import numpy as np

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent / "core"))

from merge_voters import CostTracker, finalize_trial, sweep_trial

# ── Configuration ─────────────────────────────────────────────────────────────

DATASETS: dict[str, dict] = json.loads(
    (HERE / "rq1_config.json").read_text())["datasets"]

LEVELS  = [1, 2, 3, 4, 5]
TRIALS  = 3
SEED    = 42
KB      = HERE.parent / "KB-iso.json"
PREFIX  = "V0-ensemble"

PAIRS   = list(combinations(range(1, TRIALS + 1), 2))  # (1,2),(1,3),(2,3)
SIM_THRESHOLD = 0.6
EMB_MODEL     = "all-mpnet-base-v2"


# ── Voter assignment ───────────────────────────────────────────────────────────

def _stable_hash(s: str) -> int:
    return int(hashlib.md5(s.encode()).hexdigest(), 16) % (2 ** 16)


def build_voter_pool(pool_dir: str, n_voters: int) -> list[Path]:
    base = HERE / pool_dir
    return [base / f"run{i}" / "V0.json" for i in range(1, n_voters + 1)]


def assign_voters(pool: list[Path], v: int, dataset: str) -> dict[int, list[Path]]:
    needed = v * TRIALS
    if needed > len(pool):
        raise ValueError(
            f"Need {needed} unique voters for v={v}×{TRIALS} trials "
            f"but pool has {len(pool)}."
        )
    seed = SEED + _stable_hash(dataset) + v
    rng  = random.Random(seed)
    sel  = rng.sample(pool, needed)
    return {t + 1: sel[t * v: (t + 1) * v] for t in range(TRIALS)}


def verify_assignments(assignments: dict[int, list[Path]], v: int) -> list[str]:
    errors = []
    all_p: list[Path] = []
    for trial, voters in assignments.items():
        if len(voters) != v:
            errors.append(f"Trial {trial}: expected {v} voters, got {len(voters)}")
        all_p.extend(voters)
    if len(all_p) != len({str(p) for p in all_p}):
        errors.append("Duplicate voters across trials")
    return errors


# ── Option-A threshold selection ──────────────────────────────────────────────

def _encode_ags(encoder, trial_model_paths: dict[int, Path]) -> dict[int, tuple]:
    """Return {trial: (ags_list, embeddings)} for each trial model."""
    from compute_v3_metrics import encode

    result = {}
    for trial, path in trial_model_paths.items():
        data = json.loads(path.read_text())
        ags  = [n for n in data.get("nodes", []) if n.get("kind") == "ag"]
        if ags:
            texts = [n["label"] + ". " + n.get("statement", "") for n in ags]
            embs  = encode(encoder, texts)
        else:
            embs = np.zeros((0, encoder.get_sentence_embedding_dimension()))
        result[trial] = (ags, embs)
    return result


def _pairwise_asg_coverage(
    encoded: dict[int, tuple], threshold: float = SIM_THRESHOLD
) -> float:
    """Mean bidirectional ASG coverage across all trial pairs."""
    from compute_v3_metrics import cosine_sim_matrix, match_nodes

    coverages: list[float] = []
    for (ti, tj) in PAIRS:
        ags_a, emb_a = encoded[ti]
        ags_b, emb_b = encoded[tj]
        if not ags_a or not ags_b:
            continue
        sim   = cosine_sim_matrix(emb_a, emb_b)
        match = match_nodes(ags_a, ags_b, sim, [threshold], threshold)
        p     = match["at_primary"]
        n     = p["n_matched"]
        coverages.append(n / len(ags_a))
        coverages.append(n / len(ags_b))
    return float(np.mean(coverages)) if coverages else 0.0


def pick_optimal_threshold(
    level_dir: Path, v: int, encoder
) -> tuple[int, dict]:
    """
    Compute Option-A F1 = harmonic(stability, completeness) for t=1..v.
    stability   = mean pairwise ASG coverage across the 3 trials
    completeness = mean(n_ags at t) / mean(n_ags at t=1)

    Returns (optimal_t, {t: {stability, completeness, f1, mean_ags}}).
    """
    sweep: dict[int, dict] = {}

    # Collect synthesis-only models
    for t in range(1, v + 1):
        trial_paths: dict[int, Path] = {}
        for trial in range(1, TRIALS + 1):
            p = level_dir / f"trial{trial}" / f"t{t}" / f"{PREFIX}.json"
            # v=1 special case: the path IS the voter file (stored elsewhere);
            # we check for the copied/linked version in the expected location.
            if not p.exists() and v == 1:
                # For v=1, sweep_trial returns the voter path directly;
                # the voter_assignments.json records the actual path.
                assign_file = level_dir / "voter_assignments.json"
                if assign_file.exists():
                    rec = json.loads(assign_file.read_text())
                    voter = HERE / rec["trials"][str(trial)][0]
                    if voter.exists():
                        trial_paths[trial] = voter
            elif p.exists():
                trial_paths[trial] = p
        if len(trial_paths) < TRIALS:
            continue

        encoded   = _encode_ags(encoder, trial_paths)
        stability = _pairwise_asg_coverage(encoded)
        ags_list  = [len(encoded[tr][0]) for tr in range(1, TRIALS + 1)]
        mean_ags  = float(np.mean(ags_list))
        sweep[t]  = {"mean_ags": round(mean_ags, 2), "stability": round(stability, 4)}

    # Completeness is relative to t=1
    baseline_ags = sweep.get(1, {}).get("mean_ags", 1.0) or 1.0
    for t, d in sweep.items():
        completeness = d["mean_ags"] / baseline_ags
        s, c         = d["stability"], completeness
        f1           = 2 * s * c / (s + c) if (s + c) > 0 else 0.0
        d["completeness"] = round(completeness, 4)
        d["f1"]           = round(f1, 4)

    if not sweep:
        return math.ceil(v / 2), {}

    optimal_t = max(sweep, key=lambda t: sweep[t]["f1"])
    return optimal_t, sweep


# ── Experiment runner ──────────────────────────────────────────────────────────

def run_level(
    dataset: str,
    cfg: dict,
    v: int,
    encoder,
    dry_run: bool,
    tracker: CostTracker,
) -> None:
    level_dir = HERE / "experiments" / dataset / f"v{v}"
    level_dir.mkdir(parents=True, exist_ok=True)
    assign_file = level_dir / "voter_assignments.json"

    if assign_file.exists():
        # Reload from disk so the voter set always matches the cluster cache
        # that was built during the original run.
        record      = json.loads(assign_file.read_text())
        assignments = {
            int(t): [HERE / p for p in paths]
            for t, paths in record["trials"].items()
        }
        print(f"  Loaded existing voter assignments from {assign_file.relative_to(HERE)}")
    else:
        pool        = build_voter_pool(cfg["pool_dir"], cfg["n_voters"])
        assignments = assign_voters(pool, v, dataset)
        errs        = verify_assignments(assignments, v)
        if errs:
            print(f"\n[ERROR] v={v}: {errs}")
            sys.exit(1)
        record = {
            "dataset": dataset, "level": v,
            "seed": SEED + _stable_hash(dataset) + v,
            "trials": {
                str(t): [str(p.relative_to(HERE)) for p in voters]
                for t, voters in assignments.items()
            },
        }
        assign_file.write_text(json.dumps(record, indent=2))

    sweep_file = level_dir / "threshold_sweep.json"

    print(f"\n  v={v}  threshold=⌈v/2⌉={math.ceil(v/2)}  "
          f"voters/trial={v}  sweep t=1..{v}")
    for trial, voters in assignments.items():
        print(f"    Trial {trial}: {[p.parent.name for p in voters]}")

    if dry_run:
        print(f"  [dry-run] skipping API calls.")
        return

    # ── Phase 1: sweep synthesis-only models for each trial ───────────────────
    print(f"\n  ── Phase 1: threshold sweep (synthesis-only) ────────────────")
    snap_sweep_start = tracker.snapshot()
    for trial, voter_paths in assignments.items():
        trial_dir = level_dir / f"trial{trial}"
        trial_dir.mkdir(parents=True, exist_ok=True)
        print(f"    Trial {trial}:")
        sweep_trial(voter_paths, trial_dir, PREFIX, tracker)
    sweep_cost = tracker.delta(snap_sweep_start)
    print(f"  Sweep phase cost: ${sweep_cost['cost_usd']:.4f}  "
          f"({sweep_cost['calls']} calls)")

    # ── Phase 2: pick optimal threshold ───────────────────────────────────────
    print(f"\n  ── Phase 2: threshold selection (Option A F1) ───────────────")
    optimal_t, f1_data = pick_optimal_threshold(level_dir, v, encoder)
    print(f"  Threshold scores:")
    for t in sorted(f1_data):
        d = f1_data[t]
        marker = "  ← WINNER" if t == optimal_t else ""
        print(f"    t={t}:  stability={d['stability']:.3f}  "
              f"completeness={d['completeness']:.3f}  "
              f"F1={d['f1']:.3f}  "
              f"mean_ASGs={d['mean_ags']}{marker}")

    # ── Phase 3: finalize at optimal threshold ─────────────────────────────────
    print(f"\n  ── Phase 3: finalize at t={optimal_t} (full post-processing) ─")
    snap_final_start = tracker.snapshot()
    for trial, voter_paths in assignments.items():
        trial_dir = level_dir / f"trial{trial}"
        print(f"    Trial {trial}:")
        finalize_trial(voter_paths, trial_dir, optimal_t, PREFIX, KB, tracker)
    final_cost = tracker.delta(snap_final_start)
    print(f"  Finalize phase cost: ${final_cost['cost_usd']:.4f}  "
          f"({final_cost['calls']} calls)")

    # ── Save sweep summary ─────────────────────────────────────────────────────
    total_cost = tracker.delta(snap_sweep_start)
    sweep_record = {
        "dataset":     dataset,
        "level_v":     v,
        "optimal_t":   optimal_t,
        "by_threshold": f1_data,
        "costs": {
            "sweep":    sweep_cost,
            "finalize": final_cost,
            "total":    total_cost,
        },
    }
    sweep_file.write_text(json.dumps(sweep_record, indent=2))
    print(f"\n  Saved: {sweep_file.relative_to(HERE)}")
    print(f"  Level total cost: ${total_cost['cost_usd']:.4f}")


def run_dataset(
    dataset: str, cfg: dict, levels: list[int],
    encoder, dry_run: bool, tracker: CostTracker
) -> None:
    pool    = build_voter_pool(cfg["pool_dir"], cfg["n_voters"])
    missing = [p for p in pool if not p.exists()]
    if missing and not dry_run:
        print(f"\nERROR: {len(missing)} voter file(s) missing in "
              f"{cfg['pool_dir']}. Generate them first.")
        for m in missing[:5]:
            print(f"  {m}")
        sys.exit(1)
    elif missing:
        print(f"  WARNING: {len(missing)} voter file(s) missing (dry-run — continuing).")

    for v in levels:
        if v > cfg["max_v"]:
            continue
        print(f"\n{'─'*60}")
        run_level(dataset, cfg, v, encoder, dry_run, tracker)


# ── Smoke test ─────────────────────────────────────────────────────────────────

def smoke_test(datasets: dict) -> bool:
    all_ok = True
    for ds, cfg in datasets.items():
        pool = build_voter_pool(cfg["pool_dir"], cfg["n_voters"])
        print(f"\n=== Smoke test: voter assignments — {ds} ===")
        for v in LEVELS:
            if v > cfg["max_v"]:
                continue
            try:
                asgn = assign_voters(pool, v, ds)
                errs = verify_assignments(asgn, v)
                if errs:
                    print(f"  v={v}  FAIL: {errs}")
                    all_ok = False
                else:
                    print(f"  v={v}  OK  "
                          + str({t: [p.parent.name for p in vv]
                                 for t, vv in asgn.items()}))
            except ValueError as e:
                print(f"  v={v}  ERROR: {e}")
                all_ok = False
    return all_ok


# ── Main ───────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dataset", choices=list(DATASETS.keys()))
    parser.add_argument("--level", type=int, choices=LEVELS)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--smoke-test", action="store_true")
    parser.add_argument("--force", action="store_true",
                        help="Delete existing experiment outputs for the dataset and re-run from scratch.")
    args = parser.parse_args()

    datasets = {args.dataset: DATASETS[args.dataset]} if args.dataset else DATASETS
    levels   = [args.level] if args.level else LEVELS

    if args.force:
        import shutil
        if not args.dataset:
            print("ERROR: --force requires --dataset (refusing to wipe all datasets at once).")
            sys.exit(1)
        target = HERE / "experiments" / args.dataset
        if target.exists():
            shutil.rmtree(target)
            print(f"[--force] Deleted {target.relative_to(HERE)}")
        else:
            print(f"[--force] Nothing to delete at {target.relative_to(HERE)}")

    if args.smoke_test:
        ok = smoke_test(datasets)
        sys.exit(0 if ok else 1)

    # Load encoder once (used for threshold selection across all levels)
    encoder = None
    if not args.dry_run:
        print(f"Loading sentence encoder ({EMB_MODEL})...")
        from sentence_transformers import SentenceTransformer
        encoder = SentenceTransformer(EMB_MODEL)

    tracker = CostTracker()

    for ds, cfg in datasets.items():
        print(f"\n{'═'*60}")
        print(f"  RQ1 experiment — {ds}")
        print(f"{'═'*60}")
        run_dataset(ds, cfg, levels, encoder, args.dry_run, tracker)

    if not args.dry_run:
        snap = tracker.snapshot()
        print(f"\n{'═'*60}")
        print(f"  Run complete.  Total cost: ${snap['cost_usd']:.4f}"
              f"  ({snap['calls']} API calls,"
              f"  {snap['input_tokens']:,} in / {snap['output_tokens']:,} out tokens)")

    if args.dry_run:
        print("\n[dry-run complete — no API calls made]")


if __name__ == "__main__":
    main()
