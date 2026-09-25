#!/usr/bin/env python3
"""
run_RQ2_full.py — RQ2 delta-chain stability experiment.

For each dataset:
  1. Run 3 delta trials, each starting from the baseline model M_0 given by
     v0_json (in the paper: the RQ1 k=5 trial-2 ensemble for all trials).
     Each trial evolves V0→V1→...→Vn via the delta pipeline (discover → align).
  2. For each trial, build an independent from-scratch snapshot model S_n of the
     final version Vn (--runs 1), stored in that trial's scratch/ directory.
  3. Compare each trial's evolved model M_n against its snapshot S_n.

Config: rq2_config.json
  discovery_runs  — extraction runs per delta step (e.g. 4)
  voting_t        — vote threshold for discovery (null = ceil(runs/2))
  v0_json         — path to the RQ1 V0 ensemble used as M_0; "{trial}" in the
                    path is replaced by the trial number, giving each trial its
                    own M_0

Usage (from RQ2/):
    python run_RQ2_full.py --dataset dronology
    python run_RQ2_full.py --dataset dronology --trial 1
    python run_RQ2_full.py --dataset dronology --dry-run

Output:
    experiments/<dataset>/trial<n>/v0/V0.json
    experiments/<dataset>/trial<n>/vN/VN-discovery.json
    experiments/<dataset>/trial<n>/vN/VN-aligned.json
    experiments/<dataset>/trial<n>/vN/VN-timing.json       (discover_s, align_s)
    experiments/<dataset>/trial<n>/scratch/VN-scratch.json
    experiments/<dataset>/trial<n>/scratch/VN-scratch-timing.json
    experiments/<dataset>/trial<n>/comparison.json
    experiments/<dataset>/rq2_summary.json
"""

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

HERE   = Path(__file__).parent
CORE   = HERE.parent / "core"
KB     = HERE.parent / "KB-iso.json"
CONFIG = HERE / "rq2_config.json"

TRIALS = 3


# ── Config helpers ─────────────────────────────────────────────────────────────

def load_config(dataset: str) -> dict:
    cfg = json.loads(CONFIG.read_text())
    datasets = cfg["datasets"]
    if dataset not in datasets:
        print(f"ERROR: '{dataset}' not found in rq2_config.json. "
              f"Available: {list(datasets.keys())}")
        sys.exit(1)
    return datasets[dataset]


def load_versions(versions_json_rel: str) -> list[dict]:
    return json.loads((HERE / versions_json_rel).read_text())


def source_path(s: str) -> Path:
    """Expand $TP_DATASETS (and other env vars / ~) in a versions.json source path.
    TP_DATASETS defaults to the datasets/ folder next to this package."""
    os.environ.setdefault("TP_DATASETS", str(HERE.parent.parent / "datasets"))
    expanded = os.path.expandvars(s)
    if "$" in expanded:
        print(f"ERROR: unresolved variable in source path {s!r} — set TP_DATASETS.")
        sys.exit(1)
    return Path(expanded).expanduser()


def resolve_voting_t(cfg: dict, runs: int, key: str = "voting_t") -> int:
    t = cfg.get(key)
    return int(t) if t is not None else math.ceil(runs / 2)


# ── Subprocess helper ──────────────────────────────────────────────────────────

def _run(cmd: list[str], dry_run: bool, label: str) -> float | None:
    """Run one pipeline command; return its wall-clock time in seconds."""
    print(f"    [{label}] {' '.join(str(c) for c in cmd)}")
    if dry_run:
        return None
    t0 = time.perf_counter()
    subprocess.run([str(c) for c in cmd], check=True)
    elapsed = time.perf_counter() - t0
    print(f"    [{label}] {elapsed / 60:.1f} min")
    return elapsed


def write_timing(path: Path, **seconds: float | None) -> None:
    """Merge measured step times (seconds) into a timing JSON file."""
    measured = {k: round(v, 1) for k, v in seconds.items() if v is not None}
    if not measured:
        return
    data = json.loads(path.read_text()) if path.exists() else {}
    data.update(measured)
    path.write_text(json.dumps(data, indent=2))


# ── Delta chain ────────────────────────────────────────────────────────────────

def run_delta_step(
    version: dict,
    prior_aligned: Path,
    trial_dir: Path,
    discovery_runs: int,
    voting_t: int,
    dry_run: bool,
) -> Path:
    vname      = version["name"]
    new_source = source_path(version["source"])
    step_dir   = trial_dir / vname.lower()
    step_dir.mkdir(parents=True, exist_ok=True)

    discovery_out = step_dir / f"{vname}-discovery.json"
    aligned_out   = step_dir / f"{vname}-aligned.json"
    timing_out    = step_dir / f"{vname}-timing.json"
    discover_s = align_s = None

    if discovery_out.exists():
        print(f"    [skip] {discovery_out.relative_to(HERE)} already exists")
    else:
        prior_source_str = version.get("prior_source", "")
        prior_source = source_path(prior_source_str) if prior_source_str else None
        cmd = [
            sys.executable, CORE / "analyze_delta_discovery.py",
            str(new_source),
            "--prior-source",   str(prior_source) if prior_source else str(new_source),
            "--output",         str(discovery_out),
            "--runs",           str(discovery_runs),
            "--vote-threshold", str(voting_t),
            "--kb",             str(KB),
        ]
        discover_s = _run(cmd, dry_run, f"{vname} discover")

    if aligned_out.exists():
        print(f"    [skip] {aligned_out.relative_to(HERE)} already exists")
    else:
        cmd = [
            sys.executable, CORE / "analyze_delta_alignment.py",
            "--prior",     str(prior_aligned),
            "--discovery", str(discovery_out),
            "--output",    str(aligned_out),
        ]
        align_s = _run(cmd, dry_run, f"{vname} align")

    write_timing(timing_out, discover_s=discover_s, align_s=align_s)
    return aligned_out


# ── Per-trial scratch + comparison ────────────────────────────────────────────

def run_scratch_and_compare(
    trial_n: int,
    trial_dir: Path,
    delta_final: Path,
    final_version: dict,
    dry_run: bool,
) -> tuple[Path, Path]:
    """Generate one from-scratch model of the final version for this trial,
    then compare it against the delta endpoint. Both stored in the trial dir.
    """
    vname  = final_version["name"]
    source = str(source_path(final_version["source"]))

    scratch_out    = trial_dir / "scratch" / f"{vname}-scratch.json"
    scratch_s      = None
    comparison_out = trial_dir / "comparison.json"
    scratch_out.parent.mkdir(parents=True, exist_ok=True)

    if scratch_out.exists():
        print(f"    [skip] {scratch_out.relative_to(HERE)} already exists")
    else:
        cmd = [
            sys.executable, CORE / "analyze_architecture.py",
            source,
            "--runs",   "1",
            "--output", str(scratch_out),
            "--kb",     str(KB),
        ]
        scratch_s = _run(cmd, dry_run, f"scratch {vname}")
        write_timing(scratch_out.with_name(f"{vname}-scratch-timing.json"),
                     snapshot_s=scratch_s)

    if comparison_out.exists():
        print(f"    [skip] {comparison_out.relative_to(HERE)} already exists")
    else:
        cmd = [
            sys.executable, CORE / "compute_v3_metrics.py",
            str(delta_final),
            str(scratch_out),
            "--output", str(comparison_out),
        ]
        _run(cmd, dry_run, f"trial{trial_n} delta vs scratch")

    return scratch_out, comparison_out


# ── Trial runner ───────────────────────────────────────────────────────────────

def run_trial(
    dataset: str,
    trial_n: int,
    cfg: dict,
    versions: list[dict],
    dry_run: bool,
) -> dict:
    v0_seed_path = (HERE / cfg["v0_json"].replace("{trial}", str(trial_n))).resolve()
    if not v0_seed_path.exists() and not dry_run:
        print(f"  ERROR: V0 seed not found: {v0_seed_path}")
        sys.exit(1)

    trial_dir      = HERE / "experiments" / dataset / f"trial{trial_n}"
    trial_dir.mkdir(parents=True, exist_ok=True)
    discovery_runs = cfg["discovery_runs"]
    voting_t       = resolve_voting_t(cfg, discovery_runs, "voting_t")

    print(f"\n  Trial {trial_n}  v0={v0_seed_path.relative_to(HERE.parent)}"
          f"  discovery_runs={discovery_runs}  voting_t={voting_t}")

    # V0 — copy voted ensemble into trial dir
    v0_dir = trial_dir / "v0"
    v0_dir.mkdir(parents=True, exist_ok=True)
    v0_out = v0_dir / "V0.json"
    if not v0_out.exists():
        if not dry_run:
            shutil.copy2(v0_seed_path, v0_out)
            print(f"    [V0] copied voted ensemble → {v0_out.relative_to(HERE)}")
        else:
            print(f"    [dry-run] would copy {v0_seed_path} → {v0_out}")
    else:
        print(f"    [skip] {v0_out.relative_to(HERE)} already exists")

    # Delta chain V1..Vn
    delta_versions = [v for v in versions if v["name"] != "V0"]
    all_sources    = {v["name"]: v["source"] for v in versions}
    prior_aligned  = v0_out

    for i, v in enumerate(delta_versions):
        prior_name   = versions[i]["name"]
        prior_source = all_sources.get(prior_name, "")
        v_enriched   = dict(v, prior_source=prior_source)
        print(f"\n  ── {v['name']} ──")
        prior_aligned = run_delta_step(
            v_enriched, prior_aligned, trial_dir, discovery_runs, voting_t, dry_run
        )

    # From-scratch model of final version + comparison
    final_version = delta_versions[-1]
    print(f"\n  ── Scratch + compare ({final_version['name']}) ──")
    scratch_out, comparison_out = run_scratch_and_compare(
        trial_n, trial_dir, prior_aligned, final_version, dry_run
    )

    # Collect token counts written by discovery/alignment/scratch scripts
    def _sum_token_files(paths):
        total = {"calls": 0, "input_tokens": 0, "output_tokens": 0, "total_tokens": 0}
        for p in paths:
            if p.exists():
                d = json.loads(p.read_text())
                for k in total:
                    total[k] += d.get(k, 0)
        return total

    incremental_token_files = [
        trial_dir / v["name"].lower() / f"{v['name']}-discovery-tokens.json"
        for v in delta_versions
    ] + [
        trial_dir / v["name"].lower() / f"{v['name']}-aligned-tokens.json"
        for v in delta_versions
    ]
    scratch_token_file = scratch_out.with_name(scratch_out.stem + "-tokens.json")

    inc_tokens = _sum_token_files(incremental_token_files)
    scr_tokens = _sum_token_files([scratch_token_file])
    token_ratio = round(inc_tokens["total_tokens"] / scr_tokens["total_tokens"], 4) \
        if scr_tokens["total_tokens"] else None

    tokens_summary = {
        "incremental": inc_tokens,
        "scratch":     scr_tokens,
        "token_ratio": token_ratio,
    }
    (trial_dir / "trial-tokens.json").write_text(json.dumps(tokens_summary, indent=2))

    return {
        "trial":            trial_n,
        "v0_seed":          str(v0_seed_path),
        "delta_final_path": str(prior_aligned),
        "delta_final":      str(prior_aligned.relative_to(HERE)) if prior_aligned.exists() else None,
        "scratch":          str(scratch_out.relative_to(HERE)) if scratch_out.exists() else None,
        "comparison":       str(comparison_out.relative_to(HERE)) if comparison_out.exists() else None,
        "tokens":           tokens_summary,
    }


# ── Dataset runner ─────────────────────────────────────────────────────────────

def run_dataset(dataset: str, trials: list[int], dry_run: bool) -> None:
    print(f"\n{'═'*60}")
    print(f"  RQ2 — {dataset}")
    print(f"{'═'*60}")

    cfg            = load_config(dataset)
    versions       = load_versions(cfg["versions_json"])
    discovery_runs = cfg["discovery_runs"]
    voting_t       = resolve_voting_t(cfg, discovery_runs, "voting_t")

    trial_results = []
    for trial_n in trials:
        result = run_trial(dataset, trial_n, cfg, versions, dry_run)
        trial_results.append(result)

    summary = {
        "dataset": dataset,
        "config": {
            "voting_t":       voting_t,
            "discovery_runs": discovery_runs,
        },
        "trials": trial_results,
    }
    summary_path = HERE / "experiments" / dataset / "rq2_summary.json"
    if not dry_run:
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        summary_path.write_text(json.dumps(summary, indent=2))
        print(f"\n  Saved: {summary_path.relative_to(HERE)}")


# ── Main ───────────────────────────────────────────────────────────────────────

def main() -> None:
    all_datasets = list(json.loads(CONFIG.read_text())["datasets"].keys())

    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--dataset", choices=all_datasets, required=True)
    parser.add_argument("--trial", type=int, choices=[1, 2, 3],
                        help="Run one trial only (default: all 3)")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    trials = [args.trial] if args.trial else [1, 2, 3]
    run_dataset(args.dataset, trials, args.dry_run)

    if args.dry_run:
        print("\n[dry-run complete — no API calls made]")


if __name__ == "__main__":
    main()
