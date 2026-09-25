#!/usr/bin/env python3
"""
run_snapshots.py — Build a from-scratch snapshot model of every version and
time it.

For each version V0..Vn in the dataset's versions.json, runs the full
analysis (analyze_architecture.py) on that version's source, exactly as
run_RQ2_full.py does for the final snapshot S_n, and records the wall-clock
time. Together with the per-step times written by run_RQ2_full.py this gives
the cost of re-analysing a version from scratch vs. evolving the model to it.

Usage (from RQ2/):
    python3 run_snapshots.py --dataset dronology
    python3 run_snapshots.py --dataset dronology --versions V0 V4
    python3 run_snapshots.py --dataset dronology --dry-run

Output:
    experiments/<dataset>/snapshots/VN-snapshot.json
    experiments/<dataset>/snapshots/VN-snapshot-timing.json   (snapshot_s)
"""

import argparse
import json
import sys

from run_RQ2_full import (
    CONFIG, CORE, HERE, KB, _run, load_config, load_versions, source_path,
    write_timing,
)


def main() -> None:
    all_datasets = list(json.loads(CONFIG.read_text())["datasets"].keys())

    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--dataset", choices=all_datasets, required=True)
    parser.add_argument("--versions", nargs="+", metavar="VN",
                        help="Versions to build (default: all in versions.json)")
    parser.add_argument("--runs", type=int, default=1,
                        help="Extraction runs per snapshot (default: 1, as for S_n)")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    cfg      = load_config(args.dataset)
    versions = load_versions(cfg["versions_json"])
    if args.versions:
        unknown = set(args.versions) - {v["name"] for v in versions}
        if unknown:
            sys.exit(f"ERROR: unknown version(s) {sorted(unknown)}")
        versions = [v for v in versions if v["name"] in args.versions]

    out_dir = HERE / "experiments" / args.dataset / "snapshots"
    out_dir.mkdir(parents=True, exist_ok=True)

    for v in versions:
        vname = v["name"]
        out   = out_dir / f"{vname}-snapshot.json"
        print(f"\n  ── Snapshot {vname} ──")
        if out.exists():
            print(f"    [skip] {out.relative_to(HERE)} already exists")
            continue
        cmd = [
            sys.executable, CORE / "analyze_architecture.py",
            str(source_path(v["source"])),
            "--runs",   str(args.runs),
            "--output", str(out),
            "--kb",     str(KB),
        ]
        elapsed = _run(cmd, args.dry_run, f"snapshot {vname}")
        write_timing(out_dir / f"{vname}-snapshot-timing.json", snapshot_s=elapsed)

    if args.dry_run:
        print("\n[dry-run complete — no API calls made]")


if __name__ == "__main__":
    main()
