#!/usr/bin/env python3
"""
compute_RQ2_metrics_no_helps.py — RQ2 table recomputed on makes+harms edges only.

For each trial model pair (V4-aligned.json, V4-scratch.json):
  1. Strip all "helps" edges.
  2. Drop any DD whose only edges were "helps" (now has none).
  3. Drop any ASC whose only edges were "helps" (now has none).
  4. Write the filtered models as V4-aligned-no-helps.json and
     V4-scratch-no-helps.json in the same directories.
  5. Run the semantic comparison on the filtered pair and save
     comparison-no-helps.json alongside comparison.json.
  6. Aggregate mean ± std across 3 trials and print the table.

Usage (from RQ2/):
    python3 compute_RQ2_metrics_no_helps.py
    python3 compute_RQ2_metrics_no_helps.py --latex
    python3 compute_RQ2_metrics_no_helps.py --datasets ardupilot px4
    python3 compute_RQ2_metrics_no_helps.py --skip-filter   # reuse existing no-helps files
"""

import argparse
import json
from datetime import datetime
from pathlib import Path

import numpy as np
from sentence_transformers import SentenceTransformer
from scipy.optimize import linear_sum_assignment

# ── Reuse comparison machinery from compute_v3_metrics ───────────────────────
import sys
sys.path.insert(0, str(Path(__file__).parent.parent / "core"))
from compute_v3_metrics import (
    ag_text, ad_text, encode, cosine_sim_matrix,
    match_nodes, compare_edges, compare_nfr_tags, compute_graph_metrics,
)

HERE = Path(__file__).parent

RQ2_CONFIG = json.loads((HERE / "rq2_config.json").read_text())["datasets"]
DATASETS   = list(RQ2_CONFIG)
TRIALS   = [1, 2, 3]
THRESHOLDS = [0.5, 0.6, 0.7, 0.8]
PRIMARY_T  = 0.6
MODEL_NAME = "all-mpnet-base-v2"

DATASET_LABELS = {
    "dronboard":  "DROnboard",
    "dronology":  "Dronology",
    "ardupilot":  "Ardupilot",
    "px4":        "PX4",
    "aerostack2": "AeroStack2",
}


# ── Filtering ─────────────────────────────────────────────────────────────────

def filter_no_helps(model: dict) -> dict:
    """Return a copy of model with 'helps' edges removed.

    DDs whose only edges were 'helps' (now edge-less) are also removed,
    as are ASCs whose only edges were 'helps'.
    """
    kept_edges = [e for e in model["edges"] if e["link_type"] != "helps"]
    ad_ids_with_edge = {e["source"] for e in kept_edges}
    ag_ids_with_edge = {e["target"] for e in kept_edges}
    kept_nodes = [
        n for n in model["nodes"]
        if (n["kind"] == "ad" and n["id"] in ad_ids_with_edge)
        or (n["kind"] == "ag" and n["id"] in ag_ids_with_edge)
    ]
    return {**model, "nodes": kept_nodes, "edges": kept_edges}


def write_no_helps(src_path: Path, dst_path: Path) -> dict:
    """Filter src_path and write result to dst_path. Returns the filtered model."""
    model = json.loads(src_path.read_text())
    filtered = filter_no_helps(model)
    dst_path.write_text(json.dumps(filtered, indent=2))
    ags = sum(1 for n in filtered["nodes"] if n["kind"] == "ag")
    ads = sum(1 for n in filtered["nodes"] if n["kind"] == "ad")
    print(f"    filtered {src_path.name} → {dst_path.name}  "
          f"(ag {ags}, ad {ads}, edges {len(filtered['edges'])})")
    return filtered


# ── Comparison (mirrors compute_v3_metrics.main, but in-memory) ───────────────

def compare_pair(encoder, path_a: Path, path_b: Path, out_path: Path) -> dict:
    """Run semantic comparison on two model files; cache result to out_path."""
    if out_path.exists():
        print(f"    [skip] {out_path.name} already exists")
        return json.loads(out_path.read_text())

    def load(p):
        d = json.loads(p.read_text())
        nodes, edges = d["nodes"], d["edges"]
        ags = [n for n in nodes if n["kind"] == "ag"]
        ads = [n for n in nodes if n["kind"] == "ad"]
        return ags, ads, edges

    ags_a, ads_a, edges_a = load(path_a)
    ags_b, ads_b, edges_b = load(path_b)

    ag_sim = cosine_sim_matrix(encode(encoder, [ag_text(n) for n in ags_a]),
                                encode(encoder, [ag_text(n) for n in ags_b]))
    ad_sim = cosine_sim_matrix(encode(encoder, [ad_text(n) for n in ads_a]),
                                encode(encoder, [ad_text(n) for n in ads_b]))

    ag_match = match_nodes(ags_a, ags_b, ag_sim, THRESHOLDS, PRIMARY_T)
    ad_match = match_nodes(ads_a, ads_b, ad_sim, THRESHOLDS, PRIMARY_T)

    def strip(d):
        return {k: v for k, v in d.items() if not k.startswith("_")}

    report = {
        "meta": {
            "path_a":            str(path_a),
            "path_b":            str(path_b),
            "model":             MODEL_NAME,
            "primary_threshold": PRIMARY_T,
            "filter":            "no-helps (makes+harms edges only)",
            "timestamp":         datetime.utcnow().isoformat() + "Z",
        },
        "counts": {
            "a": {"ags": len(ags_a), "ads": len(ads_a), "edges": len(edges_a)},
            "b": {"ags": len(ags_b), "ads": len(ads_b), "edges": len(edges_b)},
        },
        "asg_matching": strip(ag_match),
        "ad_matching":  strip(ad_match),
        "edge_comparison": compare_edges(edges_a, edges_b, ag_match, ad_match),
        "nfr_tags":        compare_nfr_tags(ags_a, ags_b, ag_match),
        "graph_metrics":   compute_graph_metrics(
                               ags_a, ads_a, edges_a, ags_b, ads_b, edges_b, ad_match),
    }

    out_path.write_text(json.dumps(report, indent=2))
    print(f"    wrote {out_path.name}")
    return report


# ── Per-trial pipeline ────────────────────────────────────────────────────────

def process_trial(dataset: str, trial: int, encoder,
                  skip_filter: bool) -> dict:
    # Final version name (e.g. "V4") comes from the dataset's versions.json
    versions    = json.loads((HERE / RQ2_CONFIG[dataset]["versions_json"]).read_text())
    vn          = versions[-1]["name"]
    trial_dir   = HERE / "experiments" / dataset / f"trial{trial}"
    vn_dir      = trial_dir / vn.lower()
    scratch_dir = trial_dir / "scratch"

    aligned_src = vn_dir      / f"{vn}-aligned.json"
    scratch_src = scratch_dir / f"{vn}-scratch.json"
    aligned_dst = vn_dir      / f"{vn}-aligned-no-helps.json"
    scratch_dst = scratch_dir / f"{vn}-scratch-no-helps.json"
    cmp_out     = trial_dir   / "comparison-no-helps.json"

    if skip_filter:
        print(f"  [{dataset}/trial{trial}] using existing no-helps files")
    else:
        write_no_helps(aligned_src, aligned_dst)
        write_no_helps(scratch_src, scratch_dst)

    return compare_pair(encoder, aligned_dst, scratch_dst, cmp_out)


# ── Metric extraction & aggregation ──────────────────────────────────────────

def extract_metrics(d: dict) -> dict:
    ag  = d["asg_matching"]
    ad  = d["ad_matching"]
    t06 = "0.6"
    return {
        "ags_a":     d["counts"]["a"]["ags"],
        "ags_b":     d["counts"]["b"]["ags"],
        "ads_a":     d["counts"]["a"]["ads"],
        "ads_b":     d["counts"]["b"]["ads"],
        "c_asc_s_m": ag["by_threshold"][t06]["match_rate_b"],
        "c_asc_m_s": ag["by_threshold"][t06]["match_rate_a"],
        "asc_sim":   ag["at_primary"]["mean_sim"],
        "c_dd_s_m":  ad["by_threshold"][t06]["match_rate_b"],
        "c_dd_m_s":  ad["by_threshold"][t06]["match_rate_a"],
        "dd_sim":    ad["at_primary"]["mean_sim"],
    }


def aggregate(reports: list[dict]) -> dict:
    rows = [extract_metrics(r) for r in reports]
    keys = list(rows[0].keys())
    result = {}
    for k in keys:
        vals = np.array([r[k] for r in rows])
        result[k] = {"mean": float(vals.mean()), "std": float(vals.std()),
                     "raw": vals.tolist()}
    return result


# ── Average std footnote ─────────────────────────────────────────────────────

def footnote_stds(stats: dict) -> str:
    """Compute mean std per group across all datasets for the table footnote."""
    ds_list = list(stats.keys())
    groups = {
        "node counts":        ["ags_a", "ags_b", "ads_a", "ads_b"],
        "coverage metrics":   ["c_asc_s_m", "c_asc_m_s", "c_dd_s_m", "c_dd_m_s"],
        "cosine similarity":  ["asc_sim", "dd_sim"],
    }
    parts = []
    for label, keys in groups.items():
        all_stds = [stats[ds][k]["std"] for ds in ds_list for k in keys]
        mean_std = np.mean(all_stds)
        fmt = f"{mean_std:.0f}" if label == "node counts" else f"{mean_std:.2f}"
        parts.append(f"{label} $\\pm${fmt}")
    return (
        "Values are means over 3 independent trials per dataset (different V0 seeds); "
        "\\textit{helps} edges excluded. "
        "Mean $\\sigma$ computed across all 15 trials (3 per dataset $\\times$ 5 datasets): "
        + ", ".join(parts) + "."
    )


# ── Table printing ────────────────────────────────────────────────────────────

def fmt_mean(mean, d=3):
    return f"{mean:.{d}f}"

def fmt_count(mean):
    return f"{mean:.0f}"


def print_table(stats: dict) -> None:
    col_w  = 12
    ds_list = list(stats.keys())
    header = f"{'Metric':<32}" + "".join(
        f"{DATASET_LABELS.get(ds, ds):>{col_w}}" for ds in ds_list)
    print()
    print("(makes + harms edges only — 'helps' excluded)")
    print(header)
    print("─" * len(header))

    def row(label, key_a, key_b=None, decimals=3):
        if key_b:
            vals = [
                f"{fmt_count(stats[ds][key_a]['mean'])} / "
                f"{fmt_count(stats[ds][key_b]['mean'])}"
                for ds in ds_list
            ]
            col = 14
        else:
            vals = [fmt_mean(stats[ds][key_a]["mean"], decimals) for ds in ds_list]
            col = col_w
        print(f"{label:<32}" + "".join(f"{v:>{col}}" for v in vals))

    print()
    print("── Scope ──────────────────────────────────────────────────────────")
    row("|ASC(Mn)| / |ASC(Sn)|", "ags_a", "ags_b")
    row("|DD(Mn)|  / |DD(Sn)|",  "ads_a", "ads_b")
    print()
    print("── ASG coverage & similarity ───────────────────────────────────────")
    row("C_ASC(Sn, Mn)", "c_asc_s_m")
    row("C_ASC(Mn, Sn)", "c_asc_m_s")
    row("ASC mean cosine sim",  "asc_sim")
    print()
    print("── DD coverage & similarity ────────────────────────────────────────")
    row("C_DD(Sn, Mn)",  "c_dd_s_m")
    row("C_DD(Mn, Sn)",  "c_dd_m_s")
    row("DD mean cosine sim",   "dd_sim")
    print()


def print_latex(stats: dict) -> None:
    ds_list = list(stats.keys())
    labels  = [DATASET_LABELS.get(ds, ds) for ds in ds_list]
    cols    = " & ".join(f"\\textbf{{{l}}}" for l in labels)

    def cell_count(ds, ka, kb):
        return f"${stats[ds][ka]['mean']:.0f}$ / ${stats[ds][kb]['mean']:.0f}$"

    def cell(ds, k, d=3):
        return f"${stats[ds][k]['mean']:.{d}f}$"

    def latex_row(label, fn):
        vals = " & ".join(fn(ds) for ds in ds_list)
        return f"    {label}\n      & {vals} \\\\"

    note = footnote_stds(stats)

    print()
    print(r"\begin{tabular}{lrrrrr}")
    print(r"    \toprule")
    print(f"    \\textbf{{Metric}}")
    print(f"      & {cols} \\\\")
    print(r"    \midrule")
    print(latex_row(r"$|ASC(M_n)|$ / $|ASC(S_n)|$",
                    lambda ds: cell_count(ds, "ags_a", "ags_b")))
    print(latex_row(r"$|DD(M_n)|$ / $|DD(S_n)|$",
                    lambda ds: cell_count(ds, "ads_a", "ads_b")))
    print(r"    \midrule")
    print(latex_row(r"$C_{\text{ASC}}(S_n,M_n)$", lambda ds: cell(ds, "c_asc_s_m")))
    print(latex_row(r"$C_{\text{ASC}}(M_n,S_n)$", lambda ds: cell(ds, "c_asc_m_s")))
    print(latex_row(r"ASC mean cosine similarity", lambda ds: cell(ds, "asc_sim")))
    print(r"    \midrule")
    print(latex_row(r"$C_{\text{DD}}(S_n,M_n)$",  lambda ds: cell(ds, "c_dd_s_m")))
    print(latex_row(r"$C_{\text{DD}}(M_n,S_n)$",  lambda ds: cell(ds, "c_dd_m_s")))
    print(latex_row(r"DD mean cosine similarity",  lambda ds: cell(ds, "dd_sim")))
    print(r"    \bottomrule")
    print(f"    \\multicolumn{{6}}{{l}}{{\\footnotesize {note}}}")
    print(r"\end{tabular}")
    print()


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    all_ds = DATASETS
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--datasets", nargs="+", choices=all_ds, default=all_ds,
                        metavar="DS", help="Datasets to include (default: all in rq2_config.json)")
    parser.add_argument("--latex", action="store_true",
                        help="Also print a LaTeX tabular snippet")
    parser.add_argument("--skip-filter", action="store_true",
                        help="Skip regenerating no-helps files (use existing ones)")
    args = parser.parse_args()

    print(f"Loading encoder ({MODEL_NAME})...")
    encoder = SentenceTransformer(MODEL_NAME)

    stats: dict[str, dict] = {}
    for ds in args.datasets:
        print(f"\n{ds}")
        reports = []
        for t in TRIALS:
            try:
                r = process_trial(ds, t, encoder, args.skip_filter)
                reports.append(r)
            except FileNotFoundError as e:
                print(f"  WARNING: skipping trial {t} — {e}")
        if reports:
            stats[ds] = aggregate(reports)

    if not stats:
        print("No data found.")
        return

    print_table(stats)
    if args.latex:
        print_latex(stats)


if __name__ == "__main__":
    main()
