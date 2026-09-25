#!/usr/bin/env python3
"""
compute_stability.py — Pairwise stability metrics for RQ1 ensemble experiments.

For each (dataset, level v), loads the 3 FINALIZED trial models from
trial{n}/final/V0-ensemble.json, computes pairwise metrics across all 3 pairs
using sentence-transformer embeddings + Hungarian matching (no LLM calls),
and aggregates to mean ± std.

Also reads threshold_sweep.json (written by run_RQ1_experiment.py) to report
the Option-A F1 sweep scores alongside the definitive stability metrics.

Usage (from RQ1/):
    python3 compute_stability.py --dataset dronology
    python3 compute_stability.py --dataset dronology --level 2

Output:
    experiments/{dataset}/v{v}/stability/trial{i}_vs_trial{j}.json
    experiments/{dataset}/v{v}/stability/summary.json
    experiments/stability_summary.csv
"""

import argparse
import csv
import json
import sys
from itertools import combinations
from pathlib import Path

import numpy as np

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent / "core"))

from compute_v3_metrics import (
    compare_edges,
    compare_nfr_tags,
    cosine_sim_matrix,
    encode,
    load_model,
    match_nodes,
)
from sentence_transformers import SentenceTransformer

LEVELS     = [1, 2, 3, 4, 5]
TRIALS     = [1, 2, 3]
PAIRS      = list(combinations(TRIALS, 2))
THRESHOLDS = [0.5, 0.6, 0.7, 0.8]
PRIMARY    = 0.6

DATASETS: dict[str, dict] = json.loads(
    (HERE / "rq1_config.json").read_text())["datasets"]

STABILITY_METRICS = [
    "asg_coverage_a", "asg_coverage_b", "asg_mean_sim",
    "ad_coverage_a",  "ad_coverage_b",  "ad_mean_sim",
    "nfr_jaccard", "link_type_agreement",
    "edge_jaccard", "edge_jaccard_topology",
]
SIZE_METRICS = ["n_ags", "n_ads", "n_edges"]


# ── Model path helper ──────────────────────────────────────────────────────────

def _final_path(dataset: str, v: int, trial: int) -> Path:
    """Path to the fully post-processed model for a given (dataset, v, trial)."""
    return (HERE / "experiments" / dataset / f"v{v}"
            / f"trial{trial}" / "final" / "V0-ensemble.json")


# ── Single-pair comparison ─────────────────────────────────────────────────────

def compare_pair(path_a: Path, path_b: Path, encoder: SentenceTransformer) -> dict:
    ags_a, ads_a, edges_a = load_model(path_a)
    ags_b, ads_b, edges_b = load_model(path_b)

    def _ag_text(n): return n["label"] + ". " + n.get("statement", "")
    def _ad_text(n): return n["label"] + ". " + n.get("decision", "")

    _empty = np.zeros((0, encoder.get_sentence_embedding_dimension()))

    ag_emb_a = encode(encoder, [_ag_text(n) for n in ags_a]) if ags_a else _empty
    ag_emb_b = encode(encoder, [_ag_text(n) for n in ags_b]) if ags_b else _empty
    ad_emb_a = encode(encoder, [_ad_text(n) for n in ads_a]) if ads_a else _empty
    ad_emb_b = encode(encoder, [_ad_text(n) for n in ads_b]) if ads_b else _empty

    def _match(nodes_a, nodes_b, emb_a, emb_b):
        if nodes_a and nodes_b:
            return match_nodes(nodes_a, nodes_b,
                               cosine_sim_matrix(emb_a, emb_b), THRESHOLDS, PRIMARY)
        return {
            "by_threshold": {}, "similarity_histogram": {},
            "at_primary": {"n_matched": 0, "mean_sim": 0.0, "median_sim": 0.0,
                           "matched_pairs": [], "unmatched_a": [], "unmatched_b": []},
            "_map_a_to_b": {}, "_map_b_to_a": {},
            "_matched_a_ids": set(), "_matched_b_ids": set(),
        }

    ag_match   = _match(ags_a, ags_b, ag_emb_a, ag_emb_b)
    ad_match   = _match(ads_a, ads_b, ad_emb_a, ad_emb_b)
    edge_cmp   = compare_edges(edges_a, edges_b, ag_match, ad_match)
    nfr        = compare_nfr_tags(ags_a, ags_b, ag_match)

    ag_p = ag_match["at_primary"]
    ad_p = ad_match["at_primary"]

    def _cov(n, total): return n / total if total else 0.0

    summary = {
        "n_ags_a":               len(ags_a),
        "n_ags_b":               len(ags_b),
        "n_ads_a":               len(ads_a),
        "n_ads_b":               len(ads_b),
        "n_edges_a":             len(edges_a),
        "n_edges_b":             len(edges_b),
        "asg_coverage_a":        _cov(ag_p["n_matched"], len(ags_a)),
        "asg_coverage_b":        _cov(ag_p["n_matched"], len(ags_b)),
        "asg_mean_sim":          ag_p["mean_sim"],
        "ad_coverage_a":         _cov(ad_p["n_matched"], len(ads_a)),
        "ad_coverage_b":         _cov(ad_p["n_matched"], len(ads_b)),
        "ad_mean_sim":           ad_p["mean_sim"],
        "nfr_jaccard":           nfr["mean_nfr_jaccard"],
        "link_type_agreement":   edge_cmp.get("link_type_agreement") or 0.0,
        "edge_jaccard":          edge_cmp.get("edge_jaccard", 0.0),
        "edge_jaccard_topology": edge_cmp.get("edge_jaccard_topology", 0.0),
    }

    def _strip(d):
        return {k: v for k, v in d.items() if not k.startswith("_")}

    return {
        "path_a": str(path_a),
        "path_b": str(path_b),
        "counts": {
            "a": {"ags": len(ags_a), "ads": len(ads_a), "edges": len(edges_a)},
            "b": {"ags": len(ags_b), "ads": len(ads_b), "edges": len(edges_b)},
        },
        "asg_matching":    _strip(ag_match),
        "ad_matching":     _strip(ad_match),
        "edge_comparison": edge_cmp,
        "nfr_tags":        nfr,
        "_summary":        summary,
    }


# ── Level aggregation ─────────────────────────────────────────────────────────

def _aggregate(pair_summaries: list[dict]) -> dict:
    if not pair_summaries:
        return {}
    return {
        k: {
            "mean":   round(float(np.mean([p[k] for p in pair_summaries])), 4),
            "std":    round(float(np.std( [p[k] for p in pair_summaries], ddof=0)), 4),
            "values": [p[k] for p in pair_summaries],
        }
        for k in pair_summaries[0]
    }


def compute_level(
    dataset: str, v: int, encoder: SentenceTransformer, force: bool = False
) -> dict | None:
    level_dir   = HERE / "experiments" / dataset / f"v{v}"
    stab_dir    = level_dir / "stability"
    summary_out = stab_dir / "summary.json"

    if summary_out.exists() and not force:
        print(f"  [skip] {summary_out.relative_to(HERE)}")
        return json.loads(summary_out.read_text())

    # Locate finalized trial models
    trial_paths: dict[int, Path] = {}
    for t in TRIALS:
        p = _final_path(dataset, v, t)
        if not p.exists():
            print(f"  [skip] v={v}: trial {t} not finalized yet ({p.relative_to(HERE)})")
            return None
        trial_paths[t] = p

    stab_dir.mkdir(parents=True, exist_ok=True)
    pair_summaries: list[dict] = []

    for (i, j) in PAIRS:
        pair_out = stab_dir / f"trial{i}_vs_trial{j}.json"
        if pair_out.exists() and not force:
            report = json.loads(pair_out.read_text())
        else:
            print(f"  Computing trial{i} vs trial{j}...")
            report = compare_pair(trial_paths[i], trial_paths[j], encoder)
            clean  = {k: v for k, v in report.items() if k != "_summary"}
            pair_out.write_text(json.dumps(clean, indent=2))

        pair_summaries.append(report["_summary"])

    aggregated = _aggregate(pair_summaries)

    # Per-trial model sizes
    trial_sizes = {}
    for t, p in trial_paths.items():
        data  = json.loads(p.read_text())
        nodes = data.get("nodes", [])
        trial_sizes[f"trial{t}"] = {
            "n_ags":   sum(1 for n in nodes if n.get("kind") == "ag"),
            "n_ads":   sum(1 for n in nodes if n.get("kind") == "ad"),
            "n_edges": len(data.get("edges", [])),
        }
    sl = list(trial_sizes.values())
    size_agg = {
        m: {
            "mean": round(float(np.mean([s[m] for s in sl])), 2),
            "std":  round(float(np.std( [s[m] for s in sl], ddof=0)), 2),
        }
        for m in ["n_ags", "n_ads", "n_edges"]
    }

    # Pull in threshold sweep info if available
    sweep_info = {}
    sweep_file = level_dir / "threshold_sweep.json"
    if sweep_file.exists():
        sweep_data = json.loads(sweep_file.read_text())
        sweep_info = {
            "optimal_t":    sweep_data.get("optimal_t"),
            "by_threshold": sweep_data.get("by_threshold", {}),
            "costs":        sweep_data.get("costs", {}),
        }

    summary = {
        "dataset":            dataset,
        "level_v":            v,
        "n_trials":           len(TRIALS),
        "pairs":              [f"trial{i}_vs_trial{j}" for i, j in PAIRS],
        "model_sizes":        {"per_trial": trial_sizes, "aggregated": size_agg},
        "pairwise_stability": aggregated,
        "threshold_sweep":    sweep_info,
    }
    summary_out.write_text(json.dumps(summary, indent=2))
    print(f"  Saved: {summary_out.relative_to(HERE)}")
    return summary


# ── CSV roll-up ───────────────────────────────────────────────────────────────

def write_csv(all_summaries: list[dict]) -> None:
    out  = HERE / "experiments" / "stability_summary.csv"
    rows = []
    for s in all_summaries:
        if not s:
            continue
        ds = s["dataset"]
        v  = s["level_v"]
        ot = s.get("threshold_sweep", {}).get("optimal_t", "?")

        pw = s.get("pairwise_stability", {})
        for metric in STABILITY_METRICS:
            if metric in pw:
                rows.append({
                    "dataset": ds, "v": v, "optimal_t": ot,
                    "scope": "pairwise", "metric": metric,
                    "mean": pw[metric]["mean"], "std": pw[metric]["std"],
                })

        sz = s.get("model_sizes", {}).get("aggregated", {})
        for metric in SIZE_METRICS:
            if metric in sz:
                rows.append({
                    "dataset": ds, "v": v, "optimal_t": ot,
                    "scope": "model_size", "metric": metric,
                    "mean": sz[metric]["mean"], "std": sz[metric]["std"],
                })

    if not rows:
        print("No data to write.")
        return
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(
            f, fieldnames=["dataset", "v", "optimal_t", "scope", "metric", "mean", "std"]
        )
        w.writeheader()
        w.writerows(rows)
    print(f"\nCSV → {out.relative_to(HERE)}")


# ── Console summary ───────────────────────────────────────────────────────────

def _print_summary(s: dict) -> None:
    pw = s.get("pairwise_stability", {})
    sz = s.get("model_sizes", {}).get("aggregated", {})
    sw = s.get("threshold_sweep", {})

    def fmt(k):
        if k not in pw:
            return "n/a"
        return f"{pw[k]['mean']:.3f} ± {pw[k]['std']:.3f}"

    def szf(k):
        d = sz.get(k, {})
        return f"{d.get('mean','?'):.1f} ± {d.get('std','?'):.1f}"

    if sw:
        ot    = sw.get("optimal_t", "?")
        costs = sw.get("costs", {})
        print(f"    Optimal threshold: t={ot}  "
              f"(sweep ${costs.get('sweep',{}).get('cost_usd', 0):.4f}"
              f"  finalize ${costs.get('finalize',{}).get('cost_usd', 0):.4f}"
              f"  total ${costs.get('total',{}).get('cost_usd', 0):.4f})")
        if sw.get("by_threshold"):
            for t, d in sorted(sw["by_threshold"].items(), key=lambda x: int(x[0])):
                marker = " ←" if str(t) == str(ot) else "  "
                print(f"      t={t}: F1={d['f1']:.3f}  "
                      f"stability={d['stability']:.3f}  "
                      f"completeness={d['completeness']:.3f}  "
                      f"ASGs={d['mean_ags']}{marker}")

    print(f"    Model size  ASGs={szf('n_ags')}  ADs={szf('n_ads')}")
    print(f"    ASG coverage A/B:   {fmt('asg_coverage_a')} / {fmt('asg_coverage_b')}")
    print(f"    AD  coverage A/B:   {fmt('ad_coverage_a')} / {fmt('ad_coverage_b')}")
    print(f"    ASG mean cosine:    {fmt('asg_mean_sim')}")
    print(f"    AD  mean cosine:    {fmt('ad_mean_sim')}")
    print(f"    NFR Jaccard:        {fmt('nfr_jaccard')}")
    print(f"    Link-type agr:      {fmt('link_type_agreement')}")
    print(f"    Edge Jaccard:       {fmt('edge_jaccard')}")


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dataset", choices=list(DATASETS.keys()))
    parser.add_argument("--level", type=int, choices=LEVELS)
    parser.add_argument("--model", default="all-mpnet-base-v2")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    datasets = {args.dataset: DATASETS[args.dataset]} if args.dataset else DATASETS
    levels   = [args.level] if args.level else LEVELS

    print(f"Loading encoder ({args.model})...")
    encoder = SentenceTransformer(args.model)

    all_summaries = []
    for ds, cfg in datasets.items():
        print(f"\n{'═'*60}")
        print(f"  Stability analysis — {ds}")
        print(f"{'═'*60}")
        for v in levels:
            if v > cfg["max_v"]:
                continue
            print(f"\n  v={v}:")
            summary = compute_level(ds, v, encoder, force=args.force)
            if summary:
                _print_summary(summary)
                all_summaries.append(summary)

    write_csv(all_summaries)


if __name__ == "__main__":
    main()
