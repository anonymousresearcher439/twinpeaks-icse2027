#!/usr/bin/env python3
"""
compare_vfinal.py — Deterministic semantic comparison of two Twin Peaks models.

Uses GPU-accelerated sentence embeddings + optimal bipartite matching (Hungarian).
No LLM API calls.

Usage (from delta-runs/):
    python3 ../compare_vfinal.py v5/V5-aligned.json v-final/VFinal.json
    python3 ../compare_vfinal.py v5/V5-aligned.json v-final/VFinal.json \\
        --model all-mpnet-base-v2 --threshold 0.6 --output v-final/comparison.json
"""

import argparse
import json
import os
import sys
from datetime import datetime

import networkx as nx
import numpy as np
from scipy.optimize import linear_sum_assignment
from sentence_transformers import SentenceTransformer

THRESHOLDS = [0.5, 0.6, 0.7, 0.8]


# ── Data loading ────────────────────────────────────────────────────────────────

def load_model(path):
    with open(path) as f:
        data = json.load(f)
    nodes = data["nodes"]
    edges = data["edges"]
    ags = [n for n in nodes if n["kind"] == "ag"]
    ads = [n for n in nodes if n["kind"] == "ad"]
    return ags, ads, edges


def ag_text(node):
    return node["label"] + ". " + node.get("statement", "")


def ad_text(node):
    return node["label"] + ". " + node.get("decision", "")


# ── Embedding + similarity ───────────────────────────────────────────────────────

def encode(encoder, texts):
    return encoder.encode(texts, normalize_embeddings=True, show_progress_bar=False)


def cosine_sim_matrix(emb_a, emb_b):
    # Embeddings are unit-length; cosine similarity = dot product
    return (emb_a @ emb_b.T).astype(float)


# ── Node matching ────────────────────────────────────────────────────────────────

def match_nodes(nodes_a, nodes_b, sim_matrix, thresholds, primary_threshold):
    """
    Globally optimal 1-to-1 matching via Hungarian algorithm.
    Returns a dict with stats at multiple thresholds plus the ID maps needed
    for edge comparison (stored under '_' keys, stripped before JSON output).
    """
    row_ind, col_ind = linear_sum_assignment(-sim_matrix)
    all_matches = [(int(r), int(c), float(sim_matrix[r, c]))
                   for r, c in zip(row_ind, col_ind)]

    # Per-threshold stats
    by_threshold = {}
    for t in thresholds:
        n = sum(1 for _, _, s in all_matches if s >= t)
        by_threshold[str(t)] = {
            "match_rate_a": round(n / len(nodes_a), 4) if nodes_a else 0.0,
            "match_rate_b": round(n / len(nodes_b), 4) if nodes_b else 0.0,
            "n_matched": n,
        }

    # Primary threshold
    primary = [(i, j, s) for i, j, s in all_matches if s >= primary_threshold]
    matched_a = {i for i, _, _ in primary}
    matched_b = {j for _, j, _ in primary}
    sims = [s for _, _, s in primary]

    at_primary = {
        "threshold": primary_threshold,
        "n_matched": len(primary),
        "mean_sim": round(float(np.mean(sims)), 4) if sims else 0.0,
        "median_sim": round(float(np.median(sims)), 4) if sims else 0.0,
        "matched_pairs": [
            {
                "id_a": nodes_a[i]["id"],
                "id_b": nodes_b[j]["id"],
                "similarity": round(s, 4),
                "label_a": nodes_a[i]["label"],
                "label_b": nodes_b[j]["label"],
            }
            for i, j, s in primary
        ],
        # Unmatched: include best available match even below threshold
        "unmatched_a": [
            {
                "id": nodes_a[i]["id"],
                "label": nodes_a[i]["label"],
                "best_match_id": nodes_b[j]["id"],
                "best_match_label": nodes_b[j]["label"],
                "best_match_sim": round(s, 4),
            }
            for i, j, s in all_matches if i not in matched_a
        ],
        "unmatched_b": [
            {
                "id": nodes_b[j]["id"],
                "label": nodes_b[j]["label"],
                "best_match_id": nodes_a[i]["id"],
                "best_match_label": nodes_a[i]["label"],
                "best_match_sim": round(s, 4),
            }
            for i, j, s in all_matches if j not in matched_b
        ],
    }

    # Per-node best-match score histogram (all nodes, not just matched pairs)
    best_scores_a = sim_matrix.max(axis=1).tolist()
    hist, bin_edges = np.histogram(best_scores_a, bins=10, range=(0.0, 1.0))
    similarity_histogram = {
        "bins": [round(float(b), 2) for b in bin_edges],
        "counts": hist.tolist(),
        "note": "best-match cosine score for each A node against all B nodes",
    }

    return {
        "by_threshold": by_threshold,
        "at_primary": at_primary,
        "similarity_histogram": similarity_histogram,
        # Internal keys for edge comparison (stripped before JSON output)
        "_map_a_to_b": {nodes_a[i]["id"]: nodes_b[j]["id"] for i, j, s in primary},
        "_map_b_to_a": {nodes_b[j]["id"]: nodes_a[i]["id"] for i, j, s in primary},
        "_matched_a_ids": {nodes_a[i]["id"] for i, _, _ in primary},
        "_matched_b_ids": {nodes_b[j]["id"] for _, j, _ in primary},
    }


# ── Edge comparison ──────────────────────────────────────────────────────────────

def compare_edges(edges_a, edges_b, ag_match, ad_match):
    """
    Map both models' edges into VFinal (B) canonical ID space, then compare sets.

    Two comparison scopes:
      matched_ags_and_ads — both endpoints matched (strictest, current baseline)
      matched_ags_only    — AG matched; unmatched ADs get unique tokens so they
                            count as non-overlapping rather than being dropped
    """
    v5_ag_to_vf = ag_match["_map_a_to_b"]   # V5 AG id  → VFinal AG id
    v5_ad_to_vf = ad_match["_map_a_to_b"]   # V5 AD id  → VFinal AD id
    matched_vf_ag = ag_match["_matched_b_ids"]
    matched_vf_ad = ad_match["_matched_b_ids"]

    # ── Scope 1: both endpoints matched ────────────────────────────────────────
    set_a_both = set()
    set_a_both_topo = set()
    comparable_count_a = 0
    for e in edges_a:
        src_vf = v5_ad_to_vf.get(e["source"])
        tgt_vf = v5_ag_to_vf.get(e["target"])
        if src_vf and tgt_vf:
            comparable_count_a += 1
            set_a_both.add((src_vf, tgt_vf, e["link_type"]))
            set_a_both_topo.add((src_vf, tgt_vf))

    set_b_both = set()
    set_b_both_topo = set()
    comparable_count_b = 0
    for e in edges_b:
        if e["source"] in matched_vf_ad and e["target"] in matched_vf_ag:
            comparable_count_b += 1
            set_b_both.add((e["source"], e["target"], e["link_type"]))
            set_b_both_topo.add((e["source"], e["target"]))

    # ── Scope 2: AG matched only (unmatched ADs get unique tokens) ─────────────
    # Answers: "for the ASGs both models agree on, do the same ADs wire into them?"
    # Unmatched ADs in A/B get prefixed tokens so they never spuriously intersect.
    set_a_ag = set()
    set_a_ag_topo = set()
    for e in edges_a:
        tgt_vf = v5_ag_to_vf.get(e["target"])
        if tgt_vf is None:
            continue
        src_vf = v5_ad_to_vf.get(e["source"])
        src_token = src_vf if src_vf else f"_a_{e['source']}"
        set_a_ag.add((src_token, tgt_vf, e["link_type"]))
        set_a_ag_topo.add((src_token, tgt_vf))

    set_b_ag = set()
    set_b_ag_topo = set()
    for e in edges_b:
        if e["target"] not in matched_vf_ag:
            continue
        src = e["source"]
        src_token = src if src in matched_vf_ad else f"_b_{src}"
        set_b_ag.add((src_token, e["target"], e["link_type"]))
        set_b_ag_topo.add((src_token, e["target"]))

    def jaccard(s1, s2):
        u = len(s1 | s2)
        return round(len(s1 & s2) / u, 4) if u else 1.0

    # Link-type agreement (both-endpoints scope)
    topo_both = set_a_both_topo & set_b_both_topo
    if topo_both:
        a_lt = {(s, t): lt for s, t, lt in set_a_both if (s, t) in topo_both}
        b_lt = {(s, t): lt for s, t, lt in set_b_both if (s, t) in topo_both}
        agree = sum(1 for k in topo_both if a_lt.get(k) == b_lt.get(k))
        lt_agreement = round(agree / len(topo_both), 4)
    else:
        lt_agreement = None

    def lt_dist(edges):
        d = {"makes": 0, "helps": 0, "harms": 0}
        for e in edges:
            lt = e.get("link_type", "")
            if lt in d:
                d[lt] += 1
        return d

    return {
        # Scope: both endpoints matched
        "comparable_edges_a": comparable_count_a,
        "comparable_edges_b": comparable_count_b,
        "comparable_ratio_a": round(comparable_count_a / len(edges_a), 4) if edges_a else 0.0,
        "comparable_ratio_b": round(comparable_count_b / len(edges_b), 4) if edges_b else 0.0,
        "edge_jaccard": jaccard(set_a_both, set_b_both),
        "edge_jaccard_topology": jaccard(set_a_both_topo, set_b_both_topo),
        "link_type_agreement": lt_agreement,
        # Scope: matched ASGs only (new — broader, includes unmatched ADs)
        "edge_jaccard_matched_ags": jaccard(set_a_ag, set_b_ag),
        "edge_jaccard_matched_ags_topology": jaccard(set_a_ag_topo, set_b_ag_topo),
        "link_type_distribution": {
            "a": lt_dist(edges_a),
            "b": lt_dist(edges_b),
        },
    }


# ── NFR tag agreement ────────────────────────────────────────────────────────────

def compare_nfr_tags(ags_a, ags_b, ag_match):
    map_a_to_b = ag_match["_map_a_to_b"]
    a_by_id = {n["id"]: n for n in ags_a}
    b_by_id = {n["id"]: n for n in ags_b}

    per_pair = []
    for a_id, b_id in map_a_to_b.items():
        a_node = a_by_id.get(a_id)
        b_node = b_by_id.get(b_id)
        if not a_node or not b_node:
            continue
        tags_a = set(a_node.get("nfr_tags") or [])
        tags_b = set(b_node.get("nfr_tags") or [])
        u = len(tags_a | tags_b)
        j = round(len(tags_a & tags_b) / u, 4) if u else 1.0
        per_pair.append({
            "id_a": a_id, "id_b": b_id,
            "tags_a": sorted(tags_a), "tags_b": sorted(tags_b),
            "jaccard": j,
        })

    mean_j = round(float(np.mean([p["jaccard"] for p in per_pair])), 4) if per_pair else 0.0

    all_tags_a = {t for n in ags_a for t in (n.get("nfr_tags") or [])}
    all_tags_b = {t for n in ags_b for t in (n.get("nfr_tags") or [])}

    return {
        "mean_nfr_jaccard": mean_j,
        "tag_coverage": {
            "only_in_a": sorted(all_tags_a - all_tags_b),
            "only_in_b": sorted(all_tags_b - all_tags_a),
            "in_both": sorted(all_tags_a & all_tags_b),
        },
        "per_pair": per_pair,
    }


# ── Graph-level metrics ──────────────────────────────────────────────────────────

def compute_graph_metrics(ags_a, ads_a, edges_a, ags_b, ads_b, edges_b, ad_match):
    def build_graph(ags, ads, edges):
        G = nx.DiGraph()
        for n in ags:
            G.add_node(n["id"], kind="ag")
        for n in ads:
            G.add_node(n["id"], kind="ad")
        for e in edges:
            G.add_edge(e["source"], e["target"], link_type=e["link_type"])
        return G

    G_a = build_graph(ags_a, ads_a, edges_a)
    G_b = build_graph(ags_b, ads_b, edges_b)

    density_a = len(edges_a) / (len(ags_a) * len(ads_a)) if ags_a and ads_a else 0.0
    density_b = len(edges_b) / (len(ags_b) * len(ads_b)) if ags_b and ads_b else 0.0

    # AD out-degree (number of ASGs each AD links to)
    deg_a = {n["id"]: G_a.out_degree(n["id"]) for n in ads_a}
    deg_b = {n["id"]: G_b.out_degree(n["id"]) for n in ads_b}

    top5_a = sorted(deg_a.items(), key=lambda x: -x[1])[:5]
    top5_b = sorted(deg_b.items(), key=lambda x: -x[1])[:5]

    # Hub overlap: are the top-5 ADs in A (translated to B IDs) in B's top-5?
    map_a_to_b = ad_match["_map_a_to_b"]
    top5_a_b_ids = {map_a_to_b[ad_id] for ad_id, _ in top5_a if ad_id in map_a_to_b}
    top5_b_ids = {ad_id for ad_id, _ in top5_b}
    hub_overlap = round(len(top5_a_b_ids & top5_b_ids) / 5, 4)

    # Degree distribution cosine similarity
    max_deg = max(max((d for d in deg_a.values()), default=0),
                  max((d for d in deg_b.values()), default=0))
    if max_deg > 0:
        hist_a = np.zeros(max_deg + 1)
        hist_b = np.zeros(max_deg + 1)
        for d in deg_a.values():
            hist_a[d] += 1
        for d in deg_b.values():
            hist_b[d] += 1
        na, nb = np.linalg.norm(hist_a), np.linalg.norm(hist_b)
        deg_dist_sim = round(float(np.dot(hist_a / na, hist_b / nb)), 4) if na and nb else 0.0
    else:
        deg_dist_sim = 0.0

    return {
        "density_a": round(density_a, 4),
        "density_b": round(density_b, 4),
        "degree_distribution_similarity": deg_dist_sim,
        "hub_overlap_top5": hub_overlap,
        "top5_hubs_a": [{"id": k, "degree": v} for k, v in top5_a],
        "top5_hubs_b": [{"id": k, "degree": v} for k, v in top5_b],
    }


# ── Printed summary ──────────────────────────────────────────────────────────────

def print_summary(report, path_a, path_b):
    pt = report["meta"]["primary_threshold"]
    c_a = report["counts"]["a"]
    c_b = report["counts"]["b"]
    ag = report["asg_matching"]
    ad = report["ad_matching"]
    ec = report["edge_comparison"]
    nfr = report["nfr_tags"]
    gm = report["graph_metrics"]
    ag_p = ag["at_primary"]
    ad_p = ad["at_primary"]

    def pct(n, d):
        return f"{n}/{d} ({n/d*100:.1f}%)" if d else "0/0"

    print()
    print("═" * 57)
    print("  VFINAL vs V5 COMPARISON")
    print(f"  A = {os.path.basename(path_a)}")
    print(f"  B = {os.path.basename(path_b)}")
    print(f"  Primary threshold = {pt}")
    print("═" * 57)
    print()
    print(f"{'COUNTS':<22} {'A (V5)':<14} B (VFinal)")
    print(f"  {'ASGs':<20} {c_a['ags']:<14} {c_b['ags']}")
    print(f"  {'ADs':<20} {c_a['ads']:<14} {c_b['ads']}")
    print(f"  {'Edges':<20} {c_a['edges']:<14} {c_b['edges']}")
    print()

    print("ASG MATCHING")
    print(f"  A coverage    {pct(ag_p['n_matched'], c_a['ags'])}")
    print(f"  B coverage    {pct(ag_p['n_matched'], c_b['ags'])}")
    print(f"  Mean sim / median    {ag_p['mean_sim']:.3f} / {ag_p['median_sim']:.3f}")
    print("  By threshold:")
    for t, v in ag["by_threshold"].items():
        print(f"    {t}  →  A:{v['match_rate_a']*100:.1f}%  B:{v['match_rate_b']*100:.1f}%  (n={v['n_matched']})")
    print()

    print("AD MATCHING")
    print(f"  A coverage    {pct(ad_p['n_matched'], c_a['ads'])}")
    print(f"  B coverage    {pct(ad_p['n_matched'], c_b['ads'])}")
    print(f"  Mean sim / median    {ad_p['mean_sim']:.3f} / {ad_p['median_sim']:.3f}")
    print("  By threshold:")
    for t, v in ad["by_threshold"].items():
        print(f"    {t}  →  A:{v['match_rate_a']*100:.1f}%  B:{v['match_rate_b']*100:.1f}%  (n={v['n_matched']})")
    print()

    print("EDGE STRUCTURE")
    print(f"  Scope: both endpoints matched")
    print(f"    Edge Jaccard (with link type)  {ec['edge_jaccard']:.3f}")
    print(f"    Edge Jaccard (topology only)   {ec['edge_jaccard_topology']:.3f}")
    lta = ec["link_type_agreement"]
    print(f"    Link-type agreement            {f'{lta:.3f}' if lta is not None else 'n/a'}")
    print(f"    Comparable edges A             {pct(ec['comparable_edges_a'], c_a['edges'])}")
    print(f"    Comparable edges B             {pct(ec['comparable_edges_b'], c_b['edges'])}")
    print(f"  Scope: matched ASGs only (unmatched ADs treated as distinct)")
    print(f"    Edge Jaccard (with link type)  {ec['edge_jaccard_matched_ags']:.3f}")
    print(f"    Edge Jaccard (topology only)   {ec['edge_jaccard_matched_ags_topology']:.3f}")
    lt_a = ec["link_type_distribution"]["a"]
    lt_b = ec["link_type_distribution"]["b"]
    print(f"  Link types A   makes={lt_a['makes']}  helps={lt_a['helps']}  harms={lt_a['harms']}")
    print(f"  Link types B   makes={lt_b['makes']}  helps={lt_b['helps']}  harms={lt_b['harms']}")
    print()

    print(f"NFR TAGS   mean Jaccard {nfr['mean_nfr_jaccard']:.3f}")
    cov = nfr["tag_coverage"]
    if cov["only_in_a"]:
        print(f"  Only in A: {', '.join(cov['only_in_a'])}")
    if cov["only_in_b"]:
        print(f"  Only in B: {', '.join(cov['only_in_b'])}")
    print()

    print("GRAPH METRICS")
    print(f"  Density A / B                {gm['density_a']:.4f} / {gm['density_b']:.4f}")
    print(f"  Degree distribution sim      {gm['degree_distribution_similarity']:.3f}")
    print(f"  Hub overlap (top 5 ADs)      {gm['hub_overlap_top5']*100:.0f}%")
    print()

    unmatched_ag_a = ag_p["unmatched_a"]
    unmatched_ag_b = ag_p["unmatched_b"]

    if unmatched_ag_a:
        print(f"UNMATCHED A ASGs — delta-only or below threshold ({len(unmatched_ag_a)}):")
        for u in unmatched_ag_a:
            print(f"  {u['id']}  \"{u['label']}\"")
            print(f"    best: {u['best_match_id']} \"{u['best_match_label']}\" (sim={u['best_match_sim']:.3f})")
    else:
        print("UNMATCHED A ASGs: none")
    print()

    if unmatched_ag_b:
        print(f"UNMATCHED B ASGs — scratch-only or below threshold ({len(unmatched_ag_b)}):")
        for u in unmatched_ag_b:
            print(f"  {u['id']}  \"{u['label']}\"")
            print(f"    best: {u['best_match_id']} \"{u['best_match_label']}\" (sim={u['best_match_sim']:.3f})")
    else:
        print("UNMATCHED B ASGs: none")
    print()


# ── Main ─────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Deterministic semantic comparison of two Twin Peaks models."
    )
    parser.add_argument("path_a", help="Model A (e.g. v5/V5-aligned.json)")
    parser.add_argument("path_b", help="Model B (e.g. v-final/VFinal.json)")
    parser.add_argument("--model", default="all-mpnet-base-v2",
                        help="Sentence-transformer model (default: all-mpnet-base-v2)")
    parser.add_argument("--threshold", type=float, default=0.6,
                        help="Primary match threshold (default: 0.6)")
    parser.add_argument("--output", default=None,
                        help="JSON output path (default: alongside path_b)")
    args = parser.parse_args()

    print(f"Loading models...")
    ags_a, ads_a, edges_a = load_model(args.path_a)
    ags_b, ads_b, edges_b = load_model(args.path_b)
    print(f"  A: {len(ags_a)} ASGs, {len(ads_a)} ADs, {len(edges_a)} edges")
    print(f"  B: {len(ags_b)} ASGs, {len(ads_b)} ADs, {len(edges_b)} edges")

    print(f"Loading encoder ({args.model})...")
    encoder = SentenceTransformer(args.model)

    print("Encoding ASGs...")
    ag_emb_a = encode(encoder, [ag_text(n) for n in ags_a])
    ag_emb_b = encode(encoder, [ag_text(n) for n in ags_b])
    ag_sim = cosine_sim_matrix(ag_emb_a, ag_emb_b)

    print("Encoding ADs...")
    ad_emb_a = encode(encoder, [ad_text(n) for n in ads_a])
    ad_emb_b = encode(encoder, [ad_text(n) for n in ads_b])
    ad_sim = cosine_sim_matrix(ad_emb_a, ad_emb_b)

    print("Matching (Hungarian)...")
    ag_match = match_nodes(ags_a, ags_b, ag_sim, THRESHOLDS, args.threshold)
    ad_match = match_nodes(ads_a, ads_b, ad_sim, THRESHOLDS, args.threshold)

    print("Comparing edges...")
    edge_cmp = compare_edges(edges_a, edges_b, ag_match, ad_match)

    print("NFR tag agreement...")
    nfr = compare_nfr_tags(ags_a, ags_b, ag_match)

    print("Graph metrics...")
    gm = compute_graph_metrics(ags_a, ads_a, edges_a, ags_b, ads_b, edges_b, ad_match)

    def strip_internal(d):
        return {k: v for k, v in d.items() if not k.startswith("_")}

    report = {
        "meta": {
            "path_a": args.path_a,
            "path_b": args.path_b,
            "model": args.model,
            "primary_threshold": args.threshold,
            "timestamp": datetime.utcnow().isoformat() + "Z",
        },
        "counts": {
            "a": {"ags": len(ags_a), "ads": len(ads_a), "edges": len(edges_a)},
            "b": {"ags": len(ags_b), "ads": len(ads_b), "edges": len(edges_b)},
        },
        "asg_matching": strip_internal(ag_match),
        "ad_matching": strip_internal(ad_match),
        "edge_comparison": edge_cmp,
        "nfr_tags": nfr,
        "graph_metrics": gm,
    }

    print_summary(report, args.path_a, args.path_b)

    out_path = args.output or os.path.join(
        os.path.dirname(os.path.abspath(args.path_b)),
        "VFinal-vs-V5-comparison.json"
    )
    with open(out_path, "w") as f:
        json.dump(report, f, indent=2)
    print(f"JSON report → {out_path}")


if __name__ == "__main__":
    main()
