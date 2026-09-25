#!/usr/bin/env python3
"""
report_timing.py — Print wall-clock timing for building snapshots and for
evolving the model from one version to the next.

Reads the timing files written by run_snapshots.py and run_RQ2_full.py:
    experiments/<dataset>/snapshots/VN-snapshot-timing.json
    experiments/<dataset>/trial<n>/vN/VN-timing.json
    experiments/<dataset>/trial<n>/scratch/VN-scratch-timing.json

Snapshot times are taken from snapshots/ and from every trial's final
snapshot S_n; evolution times are averaged over all trials that have them.
No API calls.

Usage (from RQ2/):
    python3 report_timing.py --dataset dronology
"""

import argparse
import json
from collections import defaultdict
from pathlib import Path

HERE   = Path(__file__).parent
CONFIG = HERE / "rq2_config.json"


def load(path: Path) -> dict:
    return json.loads(path.read_text()) if path.exists() else {}


def fmt(values: list[float]) -> str:
    if not values:
        return f"{'—':>6}"
    mean = sum(values) / len(values) / 60
    if len(values) == 1:
        return f"{mean:6.1f}"
    lo, hi = min(values) / 60, max(values) / 60
    return f"{mean:6.1f}  ({lo:.1f}–{hi:.1f}, n={len(values)})"


def main() -> None:
    cfg_all = json.loads(CONFIG.read_text())["datasets"]
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--dataset", choices=list(cfg_all), required=True)
    args = parser.parse_args()

    versions = json.loads((HERE / cfg_all[args.dataset]["versions_json"]).read_text())
    names    = [v["name"] for v in versions]
    exp      = HERE / "experiments" / args.dataset

    snapshot = defaultdict(list)
    discover = defaultdict(list)
    align    = defaultdict(list)

    for vn in names:
        s = load(exp / "snapshots" / f"{vn}-snapshot-timing.json").get("snapshot_s")
        if s is not None:
            snapshot[vn].append(s)

    for trial_dir in sorted(exp.glob("trial*")):
        s = load(trial_dir / "scratch" / f"{names[-1]}-scratch-timing.json").get("snapshot_s")
        if s is not None:
            snapshot[names[-1]].append(s)
        for vn in names[1:]:
            t = load(trial_dir / vn.lower() / f"{vn}-timing.json")
            if "discover_s" in t and "align_s" in t:
                discover[vn].append(t["discover_s"])
                align[vn].append(t["align_s"])

    print(f"\nTiming — {args.dataset}  (wall-clock minutes)\n")
    print("Build snapshot from scratch")
    print(f"  {'Version':<10}{'Minutes'}")
    for vn in names:
        print(f"  {vn:<10}{fmt(snapshot[vn])}")

    print("\nEvolve model to the next version (discover + align)")
    print(f"  {'Step':<10}{'Discover':<10}{'Align':<10}{'Total'}")
    for prev, vn in zip(names, names[1:]):
        total = [d + a for d, a in zip(discover[vn], align[vn])]
        d = f"{sum(discover[vn]) / len(discover[vn]) / 60:.1f}" if discover[vn] else "—"
        a = f"{sum(align[vn]) / len(align[vn]) / 60:.1f}" if align[vn] else "—"
        print(f"  {prev + '→' + vn:<10}{d:<10}{a:<10}{fmt(total).strip()}")
    print()


if __name__ == "__main__":
    main()
