#!/usr/bin/env python3
"""
analyze_delta_alignment.py — Step 2 of delta analysis: align discoveries with prior model.

Takes a prior Twin Peaks model and a delta-discovery JSON (output of
analyze_delta_discovery.py), then asks Claude to classify each discovered
ASG and AD against the prior:

  exists  — same concept already in prior; absorb new evidence
  belongs — close enough to merge into an existing prior node (subsumes/refines)
  new     — genuinely not in prior; add as a new node

For exists/belongs ASGs: update evidence/statement in the prior node; attach
any of the discovery's ADs that have no equivalent in prior.

For new ASGs: add the node and all its associated ADs (after AD alignment).

AD alignment follows the same exists/belongs/new logic, applied after AG
alignment so AD→AG links can reference resolved prior IDs.

Outputs:
  <output>.json            — updated Twin Peaks model
  <output-dir>/alignment-report.json  — human-readable alignment decisions

Usage:
    python3 -u analyze_delta_alignment.py \\
        --prior     v0/V0.json \\
        --discovery v1/V1-delta-discovery.json \\
        --output    v1/V1-aligned.json

Pipeline position:
  Step 1: analyze_delta_discovery.py   → delta-discovery.json
  Step 2: analyze_delta_alignment.py   → V1-aligned.json        ← THIS SCRIPT
  TODO Step 3: removal check (deleted files → candidate deletions)
  TODO Step 4: cohesion check (non-cohesive nodes after merge → split candidates)
"""

import argparse
import json
import os
import sys
import time
from datetime import date
from pathlib import Path

import anthropic
from dotenv import load_dotenv

load_dotenv()

from twin_peaks_postprocess import call_claude, parse_json_response, _try_parse_json

# ── Prompts ────────────────────────────────────────────────────────────────────

SYSTEM_PROMPT = """\
You are an expert software architect maintaining a living Twin Peaks traceability
model. You are given a prior model and a set of newly discovered nodes from
changed code. Your job is to classify each discovery and produce a precise
alignment plan.

Return ONLY valid JSON. No markdown fences, no explanation, no preamble.
"""

ALIGNMENT_TOKENS = 32_000


def _node_summary(nodes: list[dict], kind: str) -> str:
    body_field = "statement" if kind == "ag" else "decision"
    lines = []
    for n in nodes:
        if n.get("kind") != kind:
            continue
        lines.append(
            f'  {n["id"]}: {n["label"]}\n'
            f'    {body_field}: {n.get(body_field, "")[:200]}'
        )
    return "\n".join(lines)


def _build_alignment_prompt(prior: dict, discovery: dict) -> str:
    prior_nodes     = prior.get("nodes", [])
    disc_nodes      = discovery.get("nodes", [])
    prior_ags       = [n for n in prior_nodes  if n.get("kind") == "ag"]
    prior_ads       = [n for n in prior_nodes  if n.get("kind") == "ad"]
    disc_ags        = [n for n in disc_nodes   if n.get("kind") == "ag"]
    disc_ads        = [n for n in disc_nodes   if n.get("kind") == "ad"]
    disc_edges      = discovery.get("edges", [])

    today = date.today().isoformat()

    return f"""\
Today: {today}

You are aligning newly discovered architectural nodes from changed code against
an existing Twin Peaks model.

══ CLASSIFICATION RULES ══════════════════════════════════════════════════════

For each discovered AG and AD, assign one of:

  exists  — the prior already has a node for this exact concept.
             Map to its prior ID. Optionally supply additional_evidence to
             enrich the prior node's evidence field.

  belongs — the concept is a refinement, sub-concern, or close variant of a
             prior node — close enough that they should remain as one.
             Map to the most appropriate prior ID. Supply updated_label and/or
             updated_statement only if the prior wording genuinely needs
             broadening to accommodate the new evidence; otherwise leave empty.
             Supply additional_evidence.

  new     — genuinely absent from the prior. Provide the full node body.

For AGs that are "exists" or "belongs": also list which of the discovery's
directly-linked ADs should be attached to the mapped prior AG (those not
already covered by an existing prior AD).

For ADs, "exists" or "belongs" means the prior already has an equivalent
decision. "new" means add it. For new ADs, specify which AG (prior ID or new
AG delta_id) they link to, the link type (makes / helps / harms), and a one-
sentence connection_rationale explaining the link.
For "exists"/"belongs" ADs, use the links array ONLY to declare connections to
AGs classified as "new" in this run — supply connection_rationale for each.
Leave links empty if all connected AGs already exist in the prior model.

══ PRIOR MODEL ═══════════════════════════════════════════════════════════════

PRIOR AGs ({len(prior_ags)}):
{_node_summary(prior_ags, "ag")}

PRIOR ADs ({len(prior_ads)}):
{_node_summary(prior_ads, "ad")}

══ DELTA DISCOVERIES ═════════════════════════════════════════════════════════

DISCOVERED AGs ({len(disc_ags)}):
{_node_summary(disc_ags, "ag")}

DISCOVERED ADs ({len(disc_ads)}):
{_node_summary(disc_ads, "ad")}

DISCOVERY EDGES (AD → AG, using discovery IDs):
{json.dumps(disc_edges, indent=2)}

══ OUTPUT SCHEMA ═════════════════════════════════════════════════════════════

Return exactly this JSON structure:

{{
  "ag_alignments": [
    {{
      "delta_id":           "AG-01",
      "decision":           "exists|belongs|new",
      "prior_id":           "AG-XX or null if new",
      "rationale":          "one sentence",
      "additional_evidence":"<extra evidence to append to prior node, or empty>",
      "updated_label":      "<revised label if belongs and label needs broadening, else empty>",
      "updated_statement":  "<revised statement if belongs and statement needs broadening, else empty>",
      "new_node":           null
    }},
    {{
      "delta_id":  "AG-NN",
      "decision":  "new",
      "prior_id":  null,
      "rationale": "one sentence",
      "additional_evidence": "",
      "updated_label": "",
      "updated_statement": "",
      "new_node": {{
        "kind": "ag",
        "label": "...",
        "statement": "...",
        "evidence": "...",
        "rationale": "...",
        "confidence": "explicit|strongly_inferred|weakly_inferred",
        "safety_critical": false,
        "affected_components": []
      }}
    }}
  ],
  "ad_alignments": [
    {{
      "delta_id":  "AD-01",
      "decision":  "exists|belongs|new",
      "prior_id":  "AD-XX or null if new",
      "rationale": "one sentence",
      "additional_evidence": "",
      "updated_label": "",
      "updated_decision": "",
      "new_node":  null,
      "links": [
        {{
          "target": "<new AG delta_id — only list links to AGs classified as new>",
          "link_type": "makes|helps|harms",
          "connection_rationale": "<one sentence: how/why this AD satisfies or contributes to that AG>"
        }}
      ]
    }},
    {{
      "delta_id":  "AD-NN",
      "decision":  "new",
      "prior_id":  null,
      "rationale": "one sentence",
      "additional_evidence": "",
      "updated_label": "",
      "updated_decision": "",
      "new_node": {{
        "kind": "ad",
        "label": "...",
        "decision": "...",
        "evidence": "...",
        "rationale": "...",
        "confidence": "explicit|strongly_inferred|weakly_inferred",
        "safety_critical": false,
        "affected_components": []
      }},
      "links": [
        {{
          "target": "<prior AG ID or new AG delta_id>",
          "link_type": "makes|helps|harms",
          "connection_rationale": "<one sentence: how/why this AD satisfies or contributes to this AG>"
        }}
      ]
    }}
  ]
}}

Every discovered AG and AD must appear exactly once in the appropriate list.
Use prior IDs (e.g. AG-05) for targets when the AG is "exists" or "belongs".
Use the delta discovery ID (e.g. AG-09) for targets when the AG is "new".
"""


# ── Model assembly ─────────────────────────────────────────────────────────────

def _append_evidence(existing: str, additional: str) -> str:
    if not additional or not additional.strip():
        return existing
    if existing and additional.strip() not in existing:
        return existing.rstrip(". ") + "; " + additional.strip()
    return existing or additional


def _next_id(nodes: list[dict], kind: str) -> str:
    prefix = "AG" if kind == "ag" else "AD"
    existing = [
        int(n["id"].split("-")[1])
        for n in nodes
        if n.get("kind") == kind and "-" in n.get("id", "")
    ]
    nxt = max(existing, default=0) + 1
    return f"{prefix}-{nxt:02d}"


def apply_alignment(prior: dict, discovery: dict,
                    alignment: dict) -> tuple[dict, dict]:
    """
    Apply alignment decisions to produce an updated model.
    Returns (updated_model, alignment_report).
    """
    import copy
    model = copy.deepcopy(prior)
    nodes_by_id = {n["id"]: n for n in model["nodes"]}
    today = date.today().isoformat()

    report = {
        "meta": {
            "report_type": "delta_alignment",
            "date": today,
            "prior":     str(prior.get("meta", {}).get("repositories", "")),
            "discovery": str(discovery.get("meta", {}).get("delta_source", "")),
        },
        "ag_decisions": [],
        "ad_decisions": [],
        "summary": {},
    }

    # Map delta AG IDs → resolved model IDs (needed when wiring AD links)
    delta_ag_to_model_id: dict[str, str] = {}
    # Track model IDs newly created in this run (for fallback edge filtering)
    new_model_ag_ids: set[str] = set()

    # ── AG alignment ──────────────────────────────────────────────────────────
    ag_counts = {"exists": 0, "belongs": 0, "new": 0}
    for align in alignment.get("ag_alignments", []):
        delta_id = align["delta_id"]
        decision = align["decision"]
        prior_id = align.get("prior_id")
        ag_counts[decision] = ag_counts.get(decision, 0) + 1

        if decision in ("exists", "belongs"):
            delta_ag_to_model_id[delta_id] = prior_id
            node = nodes_by_id.get(prior_id)
            if node:
                # Absorb additional evidence
                node["evidence"] = _append_evidence(
                    node.get("evidence", ""), align.get("additional_evidence", "")
                )
                # Update label/statement only if provided (belongs case)
                if align.get("updated_label", "").strip():
                    node["label"] = align["updated_label"].strip()
                if align.get("updated_statement", "").strip():
                    node["statement"] = align["updated_statement"].strip()
                node["last_modified"] = today
            report["ag_decisions"].append({
                "delta_id": delta_id,
                "decision": decision,
                "mapped_to": prior_id,
                "rationale": align.get("rationale", ""),
            })

        else:  # new
            new_data = align.get("new_node") or {}
            new_id = _next_id(model["nodes"], "ag")
            new_node = {
                "id":                  new_id,
                "kind":                "ag",
                "label":               new_data.get("label", delta_id),
                "statement":           new_data.get("statement", ""),
                "evidence":            new_data.get("evidence", ""),
                "rationale":           new_data.get("rationale", ""),
                "confidence":          new_data.get("confidence", "strongly_inferred"),
                "safety_critical":     new_data.get("safety_critical", False),
                "affected_components": new_data.get("affected_components", []),
                "added_in_version":    today,
            }
            model["nodes"].append(new_node)
            nodes_by_id[new_id] = new_node
            delta_ag_to_model_id[delta_id] = new_id
            new_model_ag_ids.add(new_id)
            report["ag_decisions"].append({
                "delta_id": delta_id,
                "decision": "new",
                "new_id":   new_id,
                "label":    new_node["label"],
                "rationale": align.get("rationale", ""),
            })

    # ── AD alignment ──────────────────────────────────────────────────────────
    ad_counts = {"exists": 0, "belongs": 0, "new": 0}
    new_edges: list[dict] = []

    # Index discovery edges by source AD delta_id for edge propagation below
    disc_edges_by_source: dict[str, list[dict]] = {}
    for e in discovery.get("edges", []):
        disc_edges_by_source.setdefault(e["source"], []).append(e)

    # Deduplication set so we never double-add an edge already in the model
    existing_edge_pairs = {(e["source"], e["target"]) for e in model.get("edges", [])}

    for align in alignment.get("ad_alignments", []):
        delta_id = align["delta_id"]
        decision = align["decision"]
        prior_id = align.get("prior_id")
        ad_counts[decision] = ad_counts.get(decision, 0) + 1

        if decision in ("exists", "belongs"):
            node = nodes_by_id.get(prior_id)
            if node:
                node["evidence"] = _append_evidence(
                    node.get("evidence", ""), align.get("additional_evidence", "")
                )
                if align.get("updated_label", "").strip():
                    node["label"] = align["updated_label"].strip()
                if align.get("updated_decision", "").strip():
                    node["decision"] = align["updated_decision"].strip()
                node["last_modified"] = today

            # Wire connections to new AGs. Prefer LLM-supplied links (which
            # carry connection_rationale); fall back to discovery edges for any
            # the LLM missed (rationale will be absent in that case).
            llm_links = align.get("links", [])
            llm_covered: set[str] = set()
            for link in llm_links:
                raw_target = link.get("target", "")
                resolved_ag = delta_ag_to_model_id.get(raw_target, raw_target)
                if (resolved_ag in nodes_by_id and
                        (prior_id, resolved_ag) not in existing_edge_pairs):
                    new_edges.append({
                        "source":             prior_id,
                        "target":             resolved_ag,
                        "link_type":          link.get("link_type", "helps"),
                        "connection_rationale": link.get("connection_rationale", ""),
                    })
                    existing_edge_pairs.add((prior_id, resolved_ag))
                llm_covered.add(raw_target)

            for disc_edge in disc_edges_by_source.get(delta_id, []):
                raw_target = disc_edge.get("target", "")
                if raw_target in llm_covered:
                    continue
                resolved_ag = delta_ag_to_model_id.get(raw_target, raw_target)
                # Only propagate to new AGs — links between existing nodes
                # are noise from the N=5 extraction and must come from the LLM.
                if (resolved_ag in new_model_ag_ids and
                        (prior_id, resolved_ag) not in existing_edge_pairs):
                    new_edges.append({
                        "source":    prior_id,
                        "target":    resolved_ag,
                        "link_type": disc_edge.get("link_type", "helps"),
                        "connection_rationale": disc_edge.get("connection_rationale", ""),
                    })
                    existing_edge_pairs.add((prior_id, resolved_ag))

            report["ad_decisions"].append({
                "delta_id": delta_id,
                "decision": decision,
                "mapped_to": prior_id,
                "rationale": align.get("rationale", ""),
            })

        else:  # new
            new_data = align.get("new_node") or {}
            new_id = _next_id(model["nodes"], "ad")
            new_node = {
                "id":                  new_id,
                "kind":                "ad",
                "label":               new_data.get("label", delta_id),
                "decision":            new_data.get("decision", ""),
                "evidence":            new_data.get("evidence", ""),
                "rationale":           new_data.get("rationale", ""),
                "confidence":          new_data.get("confidence", "strongly_inferred"),
                "safety_critical":     new_data.get("safety_critical", False),
                "affected_components": new_data.get("affected_components", []),
                "added_in_version":    today,
            }
            model["nodes"].append(new_node)
            nodes_by_id[new_id] = new_node

            # Resolve links: delta AG IDs → model IDs
            for link in align.get("links", []):
                raw_target = link.get("target", "")
                resolved = delta_ag_to_model_id.get(raw_target, raw_target)
                if (resolved in nodes_by_id and
                        (new_id, resolved) not in existing_edge_pairs):
                    new_edges.append({
                        "source":               new_id,
                        "target":               resolved,
                        "link_type":            link.get("link_type", "makes"),
                        "connection_rationale": link.get("connection_rationale", ""),
                    })
                    existing_edge_pairs.add((new_id, resolved))

            report["ad_decisions"].append({
                "delta_id": delta_id,
                "decision": "new",
                "new_id":   new_id,
                "label":    new_node["label"],
                "rationale": align.get("rationale", ""),
                "links":    [{"target": delta_ag_to_model_id.get(l["target"], l["target"]),
                               "link_type": l["link_type"]}
                              for l in align.get("links", [])],
            })

    model["edges"] = model.get("edges", []) + new_edges

    # Update model metadata
    model.setdefault("meta", {})["last_modified"] = today

    # Summary
    prior_ags = sum(1 for n in prior.get("nodes", []) if n.get("kind") == "ag")
    prior_ads = sum(1 for n in prior.get("nodes", []) if n.get("kind") == "ad")
    new_ags   = sum(1 for n in model["nodes"] if n.get("kind") == "ag")
    new_ads   = sum(1 for n in model["nodes"] if n.get("kind") == "ad")
    report["summary"] = {
        "prior_ags":     prior_ags,
        "prior_ads":     prior_ads,
        "ag_exists":     ag_counts.get("exists", 0),
        "ag_belongs":    ag_counts.get("belongs", 0),
        "ag_new":        ag_counts.get("new", 0),
        "ad_exists":     ad_counts.get("exists", 0),
        "ad_belongs":    ad_counts.get("belongs", 0),
        "ad_new":        ad_counts.get("new", 0),
        "new_edges":     len(new_edges),
        "updated_ags":   new_ags,
        "updated_ads":   new_ads,
    }

    return model, report


# ── Main ───────────────────────────────────────────────────────────────────────

def run(prior_path: Path, discovery_path: Path,
        output_path: Path, client: anthropic.Anthropic) -> None:

    output_dir = output_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)
    prefix = output_path.stem

    prior     = json.loads(prior_path.read_text())
    discovery = json.loads(discovery_path.read_text())

    prior_nodes = prior.get("nodes", [])
    disc_nodes  = discovery.get("nodes", [])
    print(f"Prior     : {sum(1 for n in prior_nodes if n.get('kind')=='ag')} AGs + "
          f"{sum(1 for n in prior_nodes if n.get('kind')=='ad')} ADs")
    print(f"Discovery : {sum(1 for n in disc_nodes if n.get('kind')=='ag')} AGs + "
          f"{sum(1 for n in disc_nodes if n.get('kind')=='ad')} ADs")

    # ── Check for cached alignment ─────────────────────────────────────────
    raw_alignment_path = output_dir / f"{prefix}-raw-alignment.json"
    if raw_alignment_path.exists():
        print(f"\nLoading cached alignment: {raw_alignment_path.name}")
        alignment = json.loads(raw_alignment_path.read_text())
    else:
        for attempt in range(1, 3):
            print("\nRunning alignment (Claude, T=0)...")
            t0  = time.time()
            raw = call_claude(
                client,
                _build_alignment_prompt(prior, discovery),
                system=SYSTEM_PROMPT,
                max_tokens=ALIGNMENT_TOKENS,
                betas=["output-128k-2025-02-19"],
                temperature=0.0,
            )
            print(f"  done in {time.time()-t0:.1f}s")
            alignment = _try_parse_json(raw, f"{prefix}-raw-alignment", output_dir)
            if alignment is not None:
                break
            if attempt == 1:
                print("  WARNING: JSON parse failed — retrying once...", file=sys.stderr)
        else:
            print(f"\nFailed to parse alignment response after 2 attempts", file=sys.stderr)
            sys.exit(1)
        raw_alignment_path.write_text(json.dumps(alignment, indent=2))
        print(f"  Saved raw alignment → {raw_alignment_path.name}")

    # ── Print summary before applying ─────────────────────────────────────
    ag_aligns = [a for a in alignment.get("ag_alignments", []) if "decision" in a]
    ad_aligns = [a for a in alignment.get("ad_alignments", []) if "decision" in a]
    alignment["ag_alignments"] = ag_aligns
    alignment["ad_alignments"] = ad_aligns
    print(f"\nAG alignment ({len(ag_aligns)} decisions):")
    for a in ag_aligns:
        mapped = f"→ {a.get('prior_id')}" if a.get("prior_id") else "(new)"
        print(f"  [{a['decision']:7s}] {a['delta_id']} {mapped}  — {a.get('rationale','')[:80]}")
    print(f"\nAD alignment ({len(ad_aligns)} decisions):")
    for a in ad_aligns:
        mapped = f"→ {a.get('prior_id')}" if a.get("prior_id") else "(new)"
        print(f"  [{a['decision']:7s}] {a['delta_id']} {mapped}  — {a.get('rationale','')[:80]}")

    # ── Apply alignment to build updated model ─────────────────────────────
    print("\nApplying alignment to prior model...")
    updated_model, report = apply_alignment(prior, discovery, alignment)

    s = report["summary"]
    print(f"\n  AGs: {s['prior_ags']} → {s['updated_ags']}  "
          f"(exists={s['ag_exists']} belongs={s['ag_belongs']} new={s['ag_new']})")
    print(f"  ADs: {s['prior_ads']} → {s['updated_ads']}  "
          f"(exists={s['ad_exists']} belongs={s['ad_belongs']} new={s['ad_new']})")
    print(f"  New edges added: {s['new_edges']}")

    # ── Write outputs ──────────────────────────────────────────────────────
    output_path.write_text(json.dumps(updated_model, indent=2))
    report_path = output_dir / f"{prefix}-alignment-report.json"
    report_path.write_text(json.dumps(report, indent=2))

    print(f"\n  Updated model  → {output_path}")
    print(f"  Alignment report → {report_path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Align delta discoveries with prior Twin Peaks model."
    )
    parser.add_argument("--prior",     required=True, help="Prior model JSON (e.g. v0/V0.json)")
    parser.add_argument("--discovery", required=True, help="Delta discovery JSON")
    parser.add_argument("--output",    required=True, help="Output aligned model JSON")
    args = parser.parse_args()

    api_key = os.environ.get("ANTHROPIC_API_KEY")
    if not api_key:
        print("ERROR: ANTHROPIC_API_KEY not set.", file=sys.stderr)
        sys.exit(1)
    _raw_client = anthropic.Anthropic(api_key=api_key)
    _calls = [0]; _in_tok = [0]; _out_tok = [0]
    _orig_stream      = _raw_client.messages.stream
    _orig_beta_stream = _raw_client.beta.messages.stream
    class _TrackingCM:
        def __init__(self, ctx):
            self._ctx = ctx; self._s = None
        def __enter__(self):
            self._s = self._ctx.__enter__(); return self._s
        def __exit__(self, *a):
            r = self._ctx.__exit__(*a)
            try:
                u = self._s.get_final_message().usage
                _calls[0] += 1; _in_tok[0] += u.input_tokens; _out_tok[0] += u.output_tokens
            except Exception:
                pass
            return r
    _raw_client.messages.stream      = lambda **kw: _TrackingCM(_orig_stream(**kw))
    _raw_client.beta.messages.stream = lambda **kw: _TrackingCM(_orig_beta_stream(**kw))
    client = _raw_client

    output_path = Path(args.output).expanduser()
    run(
        prior_path     = Path(args.prior).expanduser(),
        discovery_path = Path(args.discovery).expanduser(),
        output_path    = output_path,
        client         = client,
    )

    tokens_path = output_path.parent / f"{output_path.stem}-tokens.json"
    tokens_path.write_text(json.dumps({
        "calls": _calls[0], "input_tokens": _in_tok[0],
        "output_tokens": _out_tok[0], "total_tokens": _in_tok[0] + _out_tok[0],
    }, indent=2))


if __name__ == "__main__":
    main()
