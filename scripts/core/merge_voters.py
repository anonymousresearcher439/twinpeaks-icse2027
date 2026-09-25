#!/usr/bin/env python3
"""
merge_voters.py — Merge pre-existing voter V0.json files into a consensus TPM.

Two-phase design for the threshold sweep:
  Phase 1  sweep_trial()    — for each threshold t in 1..v: run AG+AD synthesis
                              (clustering is reused from cache — no re-extraction).
                              Saves synthesis-only models to trial_dir/t{t}/V0-ensemble.json.
  Phase 2  finalize_trial() — run full Stage 4 post-processing on the winning threshold.
                              Saves to trial_dir/final/V0-ensemble.json.

For v=1: no merging or API calls needed; the voter IS the final model.

All API calls are tracked by CostTracker for cost estimation.

Usage (direct, for smoke-testing):
    python3 merge_voters.py \\
        --voters dronboard/v0/run1/V0.json dronboard/v0/run2/V0.json \\
        --trial-dir experiments/dronboard/v2/trial1 \\
        --kb ../KB-iso.json \\
        [--finalize-threshold 1]
"""

import argparse
import json
import math
import os
import shutil
import sys
from datetime import date
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import anthropic
from dotenv import load_dotenv
from twin_peaks_postprocess import (
    assess_ad_connections,
    classify_nfrs,
    cluster_and_synthesize_ads,
    consolidate_goals,
    prune_orphan_ags,
    refine_goals,
    vote_and_synthesize_ags,
)

load_dotenv()

_INTERNAL_KEYS = {"_verification", "_abstraction_check", "_is_new"}

# claude-sonnet-4-6 pricing ($/million tokens)
_INPUT_PRICE_PER_M  = 3.00
_OUTPUT_PRICE_PER_M = 15.00


# ── Cost tracking ─────────────────────────────────────────────────────────────

class _TrackedStreamCtx:
    """Context manager that captures usage after streaming completes."""

    def __init__(self, ctx, tracker):
        self._ctx     = ctx
        self._tracker = tracker
        self._stream  = None

    def __enter__(self):
        self._stream = self._ctx.__enter__()
        return self

    def __exit__(self, *args):
        return self._ctx.__exit__(*args)

    def get_final_text(self) -> str:
        text = self._stream.get_final_text()
        try:
            msg = self._stream.get_final_message()
            self._tracker.record(msg.usage.input_tokens, msg.usage.output_tokens)
        except Exception:
            pass
        return text

    def get_final_message(self):
        return self._stream.get_final_message()


class CostTracker:
    """Wraps an Anthropic client to accumulate token usage across all stream calls."""

    def __init__(self):
        self.input_tokens  = 0
        self.output_tokens = 0
        self.calls         = 0

    def instrument(self, client: anthropic.Anthropic) -> anthropic.Anthropic:
        """Monkey-patch client.messages.stream and client.beta.messages.stream."""
        self._patch(client.messages)
        self._patch(client.beta.messages)
        return client

    def _patch(self, messages_obj) -> None:
        tracker = self
        orig    = messages_obj.stream

        def _tracked(**kwargs):
            return _TrackedStreamCtx(orig(**kwargs), tracker)

        messages_obj.stream = _tracked

    def record(self, inp: int, out: int) -> None:
        self.input_tokens  += inp
        self.output_tokens += out
        self.calls         += 1

    @property
    def cost_usd(self) -> float:
        return (self.input_tokens  * _INPUT_PRICE_PER_M
              + self.output_tokens * _OUTPUT_PRICE_PER_M) / 1_000_000

    def snapshot(self) -> dict:
        return {
            "calls":         self.calls,
            "input_tokens":  self.input_tokens,
            "output_tokens": self.output_tokens,
            "cost_usd":      round(self.cost_usd, 6),
            "model":         "claude-sonnet-4-6",
        }

    def delta(self, prior: dict) -> dict:
        """Cost of work done since a prior snapshot()."""
        di   = self.input_tokens  - prior["input_tokens"]
        do   = self.output_tokens - prior["output_tokens"]
        cost = (di * _INPUT_PRICE_PER_M + do * _OUTPUT_PRICE_PER_M) / 1_000_000
        return {
            "calls":         self.calls         - prior["calls"],
            "input_tokens":  di,
            "output_tokens": do,
            "cost_usd":      round(cost, 6),
        }


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_client(tracker: CostTracker) -> anthropic.Anthropic:
    api_key = os.environ.get("ANTHROPIC_API_KEY")
    if not api_key:
        print("Error: ANTHROPIC_API_KEY not set.", file=sys.stderr)
        sys.exit(1)
    return tracker.instrument(anthropic.Anthropic(api_key=api_key))


def _copy_cluster_cache(src_dir: Path, dst_dir: Path, prefix: str) -> None:
    """Copy threshold-agnostic cluster files so synthesis can reuse them."""
    dst_dir.mkdir(parents=True, exist_ok=True)
    for suffix in ["-vote-clusters.json", "-vote-ad-clusters.json",
                   "-cluster-audit.json"]:
        src = src_dir / f"{prefix}{suffix}"
        dst = dst_dir / f"{prefix}{suffix}"
        if src.exists() and not dst.exists():
            shutil.copy(src, dst)


def _assemble_meta(run_results: list[dict], threshold: int, n: int) -> dict:
    first = run_results[0].get("meta", {})
    repos = list({
        name
        for r in run_results
        for name in r.get("meta", {}).get("repositories", [])
    })
    return {
        "schema_version": "1.0",
        "created":        date.today().isoformat(),
        "last_modified":  date.today().isoformat(),
        "description":    (
            f"Twin Peaks ensemble from {n} voters "
            f"(vote threshold ≥{threshold}/{n})."
        ),
        "repositories":      repos,
        "link_types":        first.get("link_types", {}),
        "confidence_levels": first.get("confidence_levels", {}),
    }


def _strip_internal(result: dict) -> dict:
    for node in result.get("nodes", []):
        for k in _INTERNAL_KEYS:
            node.pop(k, None)
    return result


# ── Phase 1: synthesis-only sweep ─────────────────────────────────────────────

def _synthesize_at_threshold(
    run_results: list[dict],
    threshold: int,
    trial_dir: Path,
    prefix: str,
    client: anthropic.Anthropic,
) -> Path:
    """
    Produce a synthesis-only (no full post-processing) model at the given threshold.
    Cluster cache is copied from trial_dir; synthesis files are saved to
    trial_dir/t{threshold}/.

    Returns the output path.
    """
    t_dir  = trial_dir / f"t{threshold}"
    output = t_dir / f"{prefix}.json"
    if output.exists():
        return output

    _copy_cluster_cache(trial_dir, t_dir, prefix)

    n         = len(run_results)
    canonical_ags, ag_id_map = vote_and_synthesize_ags(
        client, run_results, threshold, t_dir, prefix
    )
    deduped_ads, new_edges   = cluster_and_synthesize_ads(
        client, run_results, ag_id_map, t_dir, prefix
    )

    result = {
        "meta":  _assemble_meta(run_results, threshold, n),
        "nodes": canonical_ags + deduped_ads,
        "edges": new_edges,
    }
    result = prune_orphan_ags(result)
    _strip_internal(result)
    output.write_text(json.dumps(result, indent=2))
    return output


def sweep_trial(
    voter_paths: list[Path],
    trial_dir: Path,
    prefix: str,
    tracker: CostTracker,
) -> dict[int, Path]:
    """
    Run synthesis-only sweep for all thresholds t=1..v for one trial.
    Returns {threshold: synthesis_model_path}.
    For v=1 returns {1: voter_path} with no API calls.
    """
    if len(voter_paths) == 1:
        # No merging needed; voter IS the model at the only valid threshold.
        return {1: voter_paths[0]}

    run_results = [json.loads(p.read_text()) for p in voter_paths]
    client      = _make_client(tracker)
    v           = len(run_results)

    # Ensure cluster cache exists in trial_dir (created on first threshold call)
    paths: dict[int, Path] = {}
    for t in range(1, v + 1):
        snap = tracker.snapshot()
        path = _synthesize_at_threshold(run_results, t, trial_dir, prefix, client)
        cost = tracker.delta(snap)
        paths[t] = path
        ags = sum(1 for n in json.loads(path.read_text()).get("nodes", [])
                  if n.get("kind") == "ag")
        print(f"  t={t}: {ags} ASGs  (synthesis cost: ${cost['cost_usd']:.4f})")

    return paths


# ── Phase 2: full post-processing for winning threshold ───────────────────────

def finalize_trial(
    voter_paths: list[Path],
    trial_dir: Path,
    threshold: int,
    prefix: str,
    kb_path: Path,
    tracker: CostTracker,
) -> Path:
    """
    Load the synthesis-only model at `threshold` and apply full Stage 4
    post-processing. Saves to trial_dir/final/{prefix}.json.

    For v=1: copies the voter directly (no API calls).
    """
    final_dir = trial_dir / "final"
    output    = final_dir / f"{prefix}.json"
    if output.exists():
        print(f"  [skip] {output.relative_to(trial_dir.parent.parent)} already exists.")
        return output

    final_dir.mkdir(parents=True, exist_ok=True)

    if len(voter_paths) == 1:
        shutil.copy(voter_paths[0], output)
        print(f"  [v=1] Copied {voter_paths[0].name} → {output}")
        return output

    # Load the synthesis-only model for the winning threshold
    synth_path = trial_dir / f"t{threshold}" / f"{prefix}.json"
    if not synth_path.exists():
        raise FileNotFoundError(
            f"Synthesis model not found: {synth_path}\n"
            f"Run sweep_trial() first."
        )

    result = json.loads(synth_path.read_text())
    client = _make_client(tracker)

    print(f"  Stage 4 post-processing (t={threshold})...")
    snap   = tracker.snapshot()
    result = consolidate_goals(result, client, final_dir)
    result, _ = refine_goals(result, client, final_dir)
    result = prune_orphan_ags(result)
    result = assess_ad_connections(result, client, final_dir)
    result = classify_nfrs(result, client, kb_path, final_dir, prefix)
    cost   = tracker.delta(snap)

    _strip_internal(result)
    output.write_text(json.dumps(result, indent=2))

    ags = sum(1 for n in result["nodes"] if n.get("kind") == "ag")
    ads = sum(1 for n in result["nodes"] if n.get("kind") == "ad")
    print(f"  Final: {ags} ASGs  {ads} ADs  {len(result['edges'])} edges"
          f"  (post-proc cost: ${cost['cost_usd']:.4f})")
    return output


# ── CLI (smoke-testing) ───────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--voters", nargs="+", required=True)
    parser.add_argument("--trial-dir", required=True,
                        help="Directory for this trial's outputs")
    parser.add_argument("--kb", default=str(Path(__file__).resolve().parent.parent / "KB-iso.json"))
    parser.add_argument("--prefix", default="V0-ensemble")
    parser.add_argument("--finalize-threshold", type=int, default=None,
                        help="If set, also run full post-processing at this threshold")
    args = parser.parse_args()

    voter_paths = [Path(v).expanduser() for v in args.voters]
    for p in voter_paths:
        if not p.exists():
            print(f"Error: {p} not found", file=sys.stderr)
            sys.exit(1)

    trial_dir = Path(args.trial_dir).expanduser()
    trial_dir.mkdir(parents=True, exist_ok=True)
    kb_path   = Path(args.kb).expanduser()
    tracker   = CostTracker()

    print(f"\nSweeping {len(voter_paths)} voter(s) across all thresholds...")
    paths = sweep_trial(voter_paths, trial_dir, args.prefix, tracker)
    print(f"\nSweep complete. Threshold models: {list(paths.keys())}")

    if args.finalize_threshold is not None:
        t = args.finalize_threshold
        print(f"\nFinalizing at threshold t={t}...")
        finalize_trial(voter_paths, trial_dir, t, args.prefix, kb_path, tracker)

    print(f"\nTotal cost: {tracker.snapshot()}")


if __name__ == "__main__":
    main()
