#!/usr/bin/env python3
"""
analyze_architecture.py — Program 1: Initial Twin Peaks architectural analysis

Reads source code from one or more folders, sends it to Claude, and recovers
Non-Functional Requirements (NFRs) and Architectural Decisions (ADs) using
Bashar Nuseibeh's Twin Peaks model.

For large codebases the content is split into chunks, each analyzed separately,
then merged into a single unified result.

With --runs N (N > 1) a voting workflow is used for stability:
  1. N independent chunk→merge passes produce N raw twin-peaks models.
  2. AGs are clustered semantically across all N runs (one Claude call, T=0).
  3. Clusters present in fewer than --vote-threshold runs are pruned.
  4. One batch synthesis call produces a canonical AG for each surviving cluster,
     combining evidence and wording from all N variants.
  5. ADs are collected from all runs, deduplicated by label, and edges remapped
     to canonical AG IDs via the cluster map.
  6. Standard postprocess passes (consolidate → refine → prune) run once.

Output: twin-peaks.json

Usage:
    python analyze_architecture.py <folder1> [folder2 ...] [--output DIR]
    python analyze_architecture.py --runs 3 <folder1> [folder2 ...]
    python analyze_architecture.py          # interactive prompts

Requirements:
    pip install anthropic python-dotenv
    Create a .env file with: ANTHROPIC_API_KEY=sk-ant-...
"""

import argparse
import json
import math
import os
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import date
from pathlib import Path

import anthropic
from dotenv import load_dotenv

load_dotenv()

from twin_peaks_postprocess import (
    consolidate_goals, refine_goals, prune_orphan_ags, assess_ad_connections,
    collect_files, split_chunks,
    call_claude, parse_json_response, _try_parse_json,
    MODEL, MERGE_TOKENS, MERGE_BETAS,
    CLUSTER_TOKENS, SYNTHESIS_TOKENS, AD_CLUSTER_TOKENS, AD_SYNTHESIS_TOKENS,
    vote_and_synthesize_ags, cluster_and_synthesize_ads,
    verify_suspicious_ags, check_over_abstraction, classify_nfrs,
)

# ── Configuration ──────────────────────────────────────────────────────────────

MAX_TOKENS = 32_000   # per chunk analysis
CHUNK_SIZE = 350_000  # chars per analysis chunk (~87K tokens)

# ── Analysis prompts ───────────────────────────────────────────────────────────

SYSTEM_PROMPT = """\
You are an expert software architect performing retrospective architectural analysis
using Bashar Nuseibeh's Twin Peaks model. You recover Non-Functional Requirements
(NFRs) and Architectural Decisions (ADs) from source code, configuration, and
build artefacts — along with typed trace links that make co-evolution explicit.

FUNDAMENTAL RULE — abstraction level:
  AG (Architectural Goal) = WHAT quality the system must exhibit, stated
    independently of any implementation. An AG statement must remain valid
    even if the entire technology stack were replaced.
  AD (Architectural Decision) = HOW the system achieves a quality. Technology
    names, protocol names, library names, class names, parameter values, Hz rates,
    topic strings, lock types, and implementation identifiers belong in ADs,
    never in AG statements.

STATABILITY TEST — apply before emitting any AG:
  Ask: could an architect name this quality concern without having read the code?
  Is it a property they would independently recognise as important for this kind
  of system? If naming the concern requires knowing a specific class, method,
  lock type, or implementation mechanism, it is an AD — reclassify it.
    ✗  "Reentrant Lock Protection of MessageSender State"     → AD (class + mechanism)
    ✗  "Timer-Based Message Sender for Timed State Durations" → AD (class + mechanism)
    ✓  "Thread-Safe Concurrent Message Dispatch"              → AG (quality property)
    ✓  "Deterministic Timed State Transition Execution"       → AG (quality property)

Return ONLY a valid JSON object. No markdown fences, no explanation, no preamble.
"""

def build_user_prompt(code_content: str, repo_names: list[str], today: str,
                      chunk_num: int = 1, total_chunks: int = 1) -> str:
    chunk_note = ""
    if total_chunks > 1:
        chunk_note = (f"\nNOTE: This is chunk {chunk_num} of {total_chunks} of the codebase. "
                      f"Extract all NFRs and ADs visible in this portion. "
                      f"Use placeholder IDs (NFR-01, AD-01, etc.) — they will be reconciled later.\n")

    return f"""\
Analyze the codebase below and recover:{chunk_note}

── ABSTRACTION LEVEL RULES — apply to every AG you write ─────────────────────

RULE 1 — Solution-independence and the statability test: An AG statement must
survive a complete technology-stack replacement. If it names a specific
technology, protocol, library, class name, method name, lock type, topic
string, parameter name, or numeric threshold, rephrase it to name the quality
property instead. Apply the statability test: could an architect name this
concern without having read the code? If not, it is an AD.
  ✗  "The MQTT layer must reconnect after broker disconnection."
  ✓  "The communication layer must recover from transient network interruptions
     without operator intervention."
  ✗  "Setpoints must be delivered at no less than 10 Hz."
  ✓  "Setpoints must be delivered at sufficient frequency to maintain stable
     autonomous flight control."
  ✗  "The system must support PX4 and ArduPilot flight controllers."
  ✓  "The system must support interchangeable operation across multiple flight
     controller types without requiring code changes."
  ✗  "Reentrant Lock Protection of MessageSender Internal State."
  ✓  "The message dispatch subsystem must protect shared state from concurrent
     access without data corruption or deadlock."
  ✗  "Timer-Based Message Sender for Timed State Durations."
  ✓  "The system must execute timed state transitions with precise, deterministic
     timing guarantees."

RULE 2 — One concern per AG: If a label or statement needs "and", a comma, or
a semicolon to join two distinct quality attributes, split them into separate AGs.
  ✗  "Deployability, Testability, and Code Quality Enforcement"  (→ three AGs)

RULE 3 — System-specific floor: Still be specific to THIS system's context.
Do not collapse distinct, independently-motivatable concerns into one vague AG.
  ✗  "The system must be accurate."
  ✓  "The system must compute geodetically accurate 3-D coordinates for
     airspace boundary enforcement."

Technology names, protocol names, numeric thresholds, and implementation
identifiers belong in ADs, not in AG statements.

──────────────────────────────────────────────────────────────────────────────

1. ARCHITECTURAL GOALS (AGs)
   Quality requirements implicit or explicit in the code: safety, real-time
   performance, geodetic/spatial accuracy, availability, fault detection,
   deployability, maintainability, observability, extensibility, security,
   AI/ML readiness, motion quality, hardware portability, testability, etc.

2. ARCHITECTURAL DECISIONS (ADs)
   High-level design choices: communication mechanisms, data stores, deployment
   topology, language choices, external dependencies, safety mechanisms,
   configuration strategy, testing approach, etc.

3. TYPED LINKS (direction: AD → AG)
   - makes  : AD is the primary mechanism implementing or enacting this AG
   - helps  : AD contributes positively to this AG but is not the primary mechanism
   - harms  : AD negatively affects this AG (an accepted trade-off in the existing architecture)

Confidence levels:
   - explicit           : directly visible in source/config, no interpretation needed
   - strongly_inferred  : consistent pattern across multiple files; intent is clear
   - weakly_inferred    : suggested by a dependency or partial pattern; extent unconfirmed

Return ONLY this JSON schema (fill in all angle-bracketed fields):

{{
  "meta": {{
    "schema_version": "1.0",
    "created": "{today}",
    "last_modified": "{today}",
    "description": "Twin Peaks traceability graph. Nodes are AGs and ADs. Edges run AD→AG. Intended as a living artefact: update as requirements and architecture co-evolve.",
    "repositories": {json.dumps(repo_names)},
    "link_types": {{
      "makes": "The AD is the primary mechanism implementing or enacting this AG.",
      "helps": "The AD contributes positively to this AG but is not the primary mechanism.",
      "harms": "The AD negatively affects this AG (an accepted trade-off in the existing architecture)."
    }},
    "confidence_levels": {{
      "explicit": "Directly visible in source code, configuration, or build artefacts with no interpretation required.",
      "strongly_inferred": "Consistent architectural pattern across multiple files; intent is clear but not formally stated.",
      "weakly_inferred": "Suggested by presence of a dependency or partial pattern; extent or intent is unconfirmed."
    }}
  }},
  "nodes": [
    {{
      "id": "AG-01",
      "kind": "ag",
      "label": "<short descriptive name>",
      "statement": "<full prose requirement — technology-independent, single concern>",
      "evidence": "<specific files, functions, constants, or config keys that provide evidence>",
      "rationale": "<why this AG matters for this system>",
      "confidence": "explicit|strongly_inferred|weakly_inferred",
      "safety_critical": true,
      "affected_components": ["<component1>", "<component2>"]
    }},
    {{
      "id": "AD-01",
      "kind": "ad",
      "label": "<short descriptive name>",
      "decision": "<full prose description of the architectural decision and what was chosen>",
      "evidence": "<specific files, functions, or config keys that evidence this decision>",
      "rationale": "<why this decision was made and what alternatives were foregone>",
      "confidence": "explicit|strongly_inferred|weakly_inferred",
      "safety_critical": false,
      "affected_components": ["<component1>"]
    }}
  ],
  "edges": [
    {{ "source": "AD-01", "target": "AG-01", "link_type": "makes" }}
  ]
}}

Include as many AGs and ADs as the evidence supports. Aim for completeness.
Prefer more AGs at the correct abstraction level over fewer bundled or
technology-contaminated ones. Safety-critical AGs should come first.

IMPORTANT — JSON OUTPUT FORMAT:
- Return raw JSON only — no markdown fences, no preamble, no explanation.
- Within any JSON string value, escape double-quote characters as \".
  Correct:   "evidence": "uses \\"alien\\" widget approach"
  Incorrect: "evidence": "uses "alien" widget approach"

CODEBASE
========
{code_content}
"""


def build_merge_prompt(partials: list[dict], repo_names: list[str], today: str) -> str:
    partials_json = json.dumps(partials, indent=2)
    src_ag = sum(1 for p in partials for n in p.get('nodes', []) if n.get('kind') == 'ag')
    src_ad = sum(1 for p in partials for n in p.get('nodes', []) if n.get('kind') == 'ad')

    return f"""\
Below are {len(partials)} partial Twin Peaks analyses produced from different chunks
of the same codebase. Merge them into a single comprehensive, deduplicated result.

The input contains {src_ag} AG nodes (kind="ag") and {src_ad} AD nodes (kind="ad").
Your output MUST include BOTH kinds. Dropping all AD nodes is an error.

Rules:
- Consolidate AG nodes (kind="ag") that describe the same quality concern into one,
  keeping the best evidence and statement from all partials.
- Consolidate AD nodes (kind="ad") that describe the same design decision into one.
- Reassign clean sequential IDs: AG-01, AG-02, ... for AGs; AD-01, AD-02, ... for ADs.
- Node order: safety-critical AGs first, then remaining AGs, then all ADs.
- Rebuild the edges list using the new IDs; deduplicate identical edges.
- Merge evidence fields by combining unique file/function references.
- Use the highest confidence level seen across partials for each node.
- Set meta.repositories to {json.dumps(repo_names)}, meta.created and
  meta.last_modified to "{today}".
- Apply the statability test to every AG: if its statement names a class, method,
  lock type, or mechanism (e.g. "Reentrant Lock", "MessageSender", "Timer-Based"),
  reclassify it as an AD and create a thin quality-property AG in its place.

Return ONLY the merged JSON object. No markdown fences, no explanation.

PARTIAL ANALYSES
================
{partials_json}
"""


# ── Claude calls ───────────────────────────────────────────────────────────────


def _merge_pair(client: anthropic.Anthropic, a: dict, b: dict,
                repo_names: list[str], today: str,
                output_dir: Path, label: str, prefix: str = 'twin-peaks',
                temperature: float = 1.0) -> dict:
    save_path = output_dir / f'{prefix}-{label}.json'
    if save_path.exists():
        print(f"  {label}: resuming from checkpoint")
        return json.loads(save_path.read_text())
    print(f"  {label}: merging — please wait...")
    t0 = time.time()
    raw = call_claude(client, build_merge_prompt([a, b], repo_names, today),
                      max_tokens=MERGE_TOKENS, betas=MERGE_BETAS, temperature=temperature)
    print(f"  {label} done in {time.time()-t0:.1f}s")
    merged = parse_json_response(raw, label, output_dir)

    # Sanity check: if the inputs had AD nodes the output must too
    src_ad = sum(1 for n in a.get('nodes', []) + b.get('nodes', []) if n.get('kind') == 'ad')
    out_ad = sum(1 for n in merged.get('nodes', []) if n.get('kind') == 'ad')
    if src_ad > 0 and out_ad == 0:
        print(f"\n  WARNING: merge {label} dropped all {src_ad} AD nodes — retrying once...", file=sys.stderr)
        raw = call_claude(client, build_merge_prompt([a, b], repo_names, today),
                          max_tokens=MERGE_TOKENS, betas=MERGE_BETAS, temperature=temperature)
        merged = parse_json_response(raw, label + '-retry', output_dir)
        out_ad = sum(1 for n in merged.get('nodes', []) if n.get('kind') == 'ad')
        if out_ad == 0:
            print(f"  ERROR: retry also produced 0 AD nodes. Raw output saved. Exiting.", file=sys.stderr)
            sys.exit(1)

    save_path.write_text(json.dumps(merged, indent=2))
    print(f"  Saved: {save_path.name}")
    return merged


def tree_merge(client: anthropic.Anthropic, partials: list[dict],
               repo_names: list[str], today: str,
               output_dir: Path, prefix: str = 'twin-peaks',
               temperature: float = 1.0) -> dict:
    """Merge partials in a binary tree — O(log N) levels, pairs at each level run in parallel."""
    level = partials
    level_num = 0
    while len(level) > 1:
        pairs  = list(zip(level[0::2], level[1::2]))
        odd    = [level[-1]] if len(level) % 2 else []
        labels = [f'tree-L{level_num}-{i}' for i in range(len(pairs))]
        print(f"Merge level {level_num}: {len(pairs)} pair(s), {len(odd)} carry-over")

        results = [None] * len(pairs)
        with ThreadPoolExecutor(max_workers=len(pairs)) as ex:
            futures = {
                ex.submit(_merge_pair, client, a, b, repo_names, today, output_dir, lbl, prefix, temperature): idx
                for idx, ((a, b), lbl) in enumerate(zip(pairs, labels))
            }
            for fut in as_completed(futures):
                results[futures[fut]] = fut.result()

        level = results + odd
        level_num += 1
    return level[0]

# ── Voting pipeline ────────────────────────────────────────────────────────────

def _analyze_chunks(client: anthropic.Anthropic, chunks: list[str],
                    repo_names: list[str], today: str,
                    output_dir: Path, prefix: str, temperature: float) -> dict:
    """Run one chunk-analysis + merge pass. Returns a raw twin-peaks dict."""
    total_chunks = len(chunks)
    if total_chunks == 1:
        for attempt in range(1, 3):
            print(f"Sending to Claude ({MODEL})...")
            t0  = time.time()
            raw = call_claude(client, build_user_prompt(chunks[0], repo_names, today),
                              max_tokens=MERGE_TOKENS, betas=MERGE_BETAS, temperature=temperature)
            print(f"  done in {time.time()-t0:.1f}s")
            result = _try_parse_json(raw, f'{prefix}-single', output_dir)
            if result is not None:
                return result
            if attempt == 1:
                print(f"  WARNING: JSON parse failed — retrying once...", file=sys.stderr)
        print(f"\nFailed to parse Claude response ({prefix}-single) after 2 attempts", file=sys.stderr)
        sys.exit(1)
    else:
        partials = []
        for i, chunk in enumerate(chunks, 1):
            partial_path = output_dir / f'{prefix}-partial-{i}.json'
            if partial_path.exists():
                print(f"Chunk {i}/{total_chunks}: loading cached {partial_path.name}")
                partials.append(json.loads(partial_path.read_text()))
                continue
            print(f"Analyzing chunk {i}/{total_chunks} ({len(chunk):,} chars) — please wait...")
            t0  = time.time()
            raw = call_claude(client,
                              build_user_prompt(chunk, repo_names, today, i, total_chunks),
                              max_tokens=MAX_TOKENS, temperature=temperature)
            print(f"  chunk {i} done in {time.time()-t0:.1f}s")
            partial = parse_json_response(raw, f'chunk{i}', output_dir)
            partial_path.write_text(json.dumps(partial, indent=2))
            print(f"  Saved: {partial_path.name}")
            partials.append(partial)
        print(f"Merging {total_chunks} partial results...")
        return tree_merge(client, partials, repo_names, today, output_dir, prefix, temperature)



# ── Main logic ────────────────────────────────────────────────────────────────

def run(folders: list[str], output_path: Path, client: anthropic.Anthropic,
        retry_merge: bool = False, temperature: float = 1.0,
        exclude_dirs: set[str] | None = None,
        num_runs: int = 1, vote_threshold: int | None = None,
        kb_path: Path | None = None) -> None:
    """Analyse folders and write a Twin Peaks baseline model to output_path."""
    prefix       = output_path.stem
    output_dir   = output_path.parent
    code_content = None   # set in multi-run and single-run paths; None for retry
    output_dir.mkdir(parents=True, exist_ok=True)
    today = date.today().isoformat()

    print(f"  Temperature: {temperature}")

    if num_runs > 1:
        # ── Multi-run voting ────────────────────────────────────────────────────
        threshold = vote_threshold if vote_threshold is not None else math.ceil(num_runs / 2)
        print(f"  Voting mode: {num_runs} runs, threshold ≥ {threshold}/{num_runs}")

        repo_names   = [Path(f).expanduser().resolve().name for f in folders]
        print(f"\nCollecting files from: {folders}")
        code_content = collect_files(folders, exclude_dirs=exclude_dirs)
        if not code_content.strip():
            print("No source files found. Check folder paths and extensions.")
            sys.exit(1)
        chunks = split_chunks(code_content, CHUNK_SIZE)
        print(f"  Split into {len(chunks)} chunk(s).")

        # Stage 1: N independent extraction passes
        print(f"\n── Stage 1/4: Extraction ({num_runs} independent runs) ──────────────────")
        run_results: list[dict] = []
        for i in range(1, num_runs + 1):
            run_dir  = output_dir / f'{prefix}-run-{i}'
            run_path = run_dir / f'{prefix}.json'
            run_dir.mkdir(parents=True, exist_ok=True)
            if run_path.exists():
                print(f"\nRun {i}/{num_runs}: loading checkpoint {run_path.name}")
                run_results.append(json.loads(run_path.read_text()))
            else:
                print(f"\n{'─' * 60}\nRun {i}/{num_runs}\n{'─' * 60}")
                result = _analyze_chunks(client, chunks, repo_names, today,
                                          run_dir, prefix, temperature)
                run_path.write_text(json.dumps(result, indent=2))
                print(f"  Saved: {run_path}")
                run_results.append(result)

        # Stage 2: AG clustering, voting, audit
        print(f"\n── Stage 2/4: AG voting + cluster audit ────────────────────────────────")
        canonical_ags, ag_id_map = vote_and_synthesize_ags(
            client, run_results, threshold, output_dir, prefix
        )

        # Stage 3: AD clustering and synthesis
        total_ads = sum(
            sum(1 for n in r.get('nodes', []) if n.get('kind') == 'ad')
            for r in run_results
        )
        print(f"\n── Stage 3/4: AD clustering ({total_ads} ADs across {num_runs} runs) ──")
        deduped_ads, new_edges = cluster_and_synthesize_ads(
            client, run_results, ag_id_map, output_dir, prefix
        )
        print(f"  ADs after clustering: {len(deduped_ads)}")

        first_meta = run_results[0].get('meta', {})
        result = {
            'meta': {
                'schema_version':    '1.0',
                'created':           today,
                'last_modified':     today,
                'description':       (f"Twin Peaks model assembled from {num_runs} independent "
                                      f"runs (vote threshold ≥ {threshold}/{num_runs})."),
                'repositories':      repo_names,
                'link_types':        first_meta.get('link_types', {}),
                'confidence_levels': first_meta.get('confidence_levels', {}),
            },
            'nodes': canonical_ags + deduped_ads,
            'edges': new_edges,
        }
        result = prune_orphan_ags(result)

        # Challenge weakly-inferred AGs against the source code
        weak_ags = [n for n in result['nodes']
                    if n.get('kind') == 'ag'
                    and n.get('confidence') == 'weakly_inferred']
        if weak_ags:
            verified = verify_suspicious_ags(client, weak_ags, code_content,
                                             output_dir, prefix)
            by_id = {ag['id']: ag for ag in verified}
            for node in result['nodes']:
                if node['id'] in by_id:
                    node.update(by_id[node['id']])

    elif retry_merge:
        # ── Retry from saved partials ───────────────────────────────────────────
        partial_files = sorted(output_dir.glob(f'{prefix}-partial-*.json'),
                               key=lambda p: int(p.stem.rsplit('-', 1)[-1]))
        if not partial_files:
            print(f"Error: no {prefix}-partial-*.json files found in output dir.", file=sys.stderr)
            sys.exit(1)
        print(f"Retrying merge from {len(partial_files)} saved partial(s)...")
        partials   = [json.loads(p.read_text()) for p in partial_files]
        repo_names = partials[0].get('meta', {}).get('repositories', [])
        result     = tree_merge(client, partials, repo_names, today, output_dir, prefix, temperature)

    else:
        # ── Single run ─────────────────────────────────────────────────────────
        repo_names = [Path(f).expanduser().resolve().name for f in folders]

        print(f"\nCollecting files from: {folders}")
        code_content = collect_files(folders, exclude_dirs=exclude_dirs)
        if not code_content.strip():
            print("No source files found. Check folder paths and extensions.")
            sys.exit(1)

        chunks       = split_chunks(code_content, CHUNK_SIZE)
        total_chunks = len(chunks)
        print(f"  Split into {total_chunks} chunk(s) of ~{CHUNK_SIZE:,} chars each.")

        if total_chunks == 1:
            print(f"Sending to Claude ({MODEL})...")
            t0  = time.time()
            raw = call_claude(client, build_user_prompt(code_content, repo_names, today),
                              max_tokens=MERGE_TOKENS, betas=MERGE_BETAS,
                              temperature=temperature)
            print(f"  done in {time.time()-t0:.1f}s")
            result = parse_json_response(raw, f'{prefix}-single', output_dir)
        else:
            partials = []
            for i, chunk in enumerate(chunks, 1):
                partial_path = output_dir / f'{prefix}-partial-{i}.json'
                if partial_path.exists():
                    print(f"Chunk {i}/{total_chunks}: loading cached {partial_path.name}")
                    partials.append(json.loads(partial_path.read_text()))
                    continue
                print(f"Analyzing chunk {i}/{total_chunks} ({len(chunk):,} chars) — please wait...")
                t0  = time.time()
                raw = call_claude(client,
                                  build_user_prompt(chunk, repo_names, today, i, total_chunks),
                                  max_tokens=MAX_TOKENS, temperature=temperature)
                print(f"  chunk {i} done in {time.time()-t0:.1f}s")
                partial = parse_json_response(raw, f'chunk{i}', output_dir)
                partial_path.write_text(json.dumps(partial, indent=2))
                print(f"  Saved: {partial_path.name}")
                partials.append(partial)

            print(f"Merging {total_chunks} partial results...")
            result = tree_merge(client, partials, repo_names, today, output_dir, prefix, temperature)

    print(f"\n── Stage 4/4: Post-processing (consolidate → refine → verify → assess) ──")
    result = consolidate_goals(result, client, output_dir, temperature)
    result, _ = refine_goals(result, client, output_dir, temperature)
    result = prune_orphan_ags(result)

    # Check AGs with thin evidence for over-abstraction (requires source code)
    if code_content is not None:
        result = check_over_abstraction(client, result, code_content,
                                        output_dir, prefix)

    result = assess_ad_connections(result, client, output_dir, temperature)

    if kb_path is None:
        kb_path = Path(__file__).resolve().parent.parent / "KB-iso.json"
    result = classify_nfrs(result, client, kb_path, output_dir, prefix)

    # Strip internal annotation keys before writing
    _INTERNAL_KEYS = {'_verification', '_abstraction_check', '_is_new'}
    for node in result.get('nodes', []):
        for k in _INTERNAL_KEYS:
            node.pop(k, None)

    node_count = len(result.get('nodes', []))
    edge_count = len(result.get('edges', []))
    ag_count   = sum(1 for n in result.get('nodes', []) if n.get('kind') == 'ag')
    ad_count   = sum(1 for n in result.get('nodes', []) if n.get('kind') == 'ad')

    output_path.write_text(json.dumps(result, indent=2))
    print(f"\nDone.")
    print(f"  {ag_count} AGs + {ad_count} ADs ({node_count} nodes), {edge_count} edges")
    print(f"  Output: {output_path}")


# ── CLI entry point ────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Recover Twin Peaks NFRs and ADs from source code via Claude API.')
    parser.add_argument('folders', nargs='*',
                        help='Source folders to analyse (interactive if omitted)')
    parser.add_argument('--output', '-o', default='twin-peaks.json',
                        help='Output file path for the Twin Peaks model (default: twin-peaks.json)')
    parser.add_argument('--retry-merge', action='store_true',
                        help='Skip chunk analysis; re-merge from saved partial files')
    parser.add_argument('--temperature', '-t', type=float, default=1.0,
                        help='Sampling temperature for Claude (0.0–1.0, default: 1.0)')
    parser.add_argument('--runs', '-n', type=int, default=1,
                        help='Number of independent analysis runs for voting (default: 1). '
                             'When > 1, AGs are clustered and vote-filtered for stability.')
    parser.add_argument('--vote-threshold', type=int, default=None,
                        help='Min runs an AG must appear in to survive the chop '
                             '(default: majority = ceil(runs/2)). Only used when --runs > 1.')
    parser.add_argument('--config', '-c', default=None,
                        help='Path to experiment config JSON. Reads version.folders and '
                             'version.exclude_dirs; CLI args take precedence.')
    parser.add_argument('--kb', default=str(Path(__file__).resolve().parent.parent / "KB-iso.json"),
                        help='Path to NFR taxonomy JSON for AG classification '
                             '(default: KB-iso.json at the package root)')
    args = parser.parse_args()

    api_key = os.environ.get('ANTHROPIC_API_KEY')
    if not api_key:
        print("Error: ANTHROPIC_API_KEY environment variable not set.", file=sys.stderr)
        sys.exit(1)

    # Load config if provided
    config_folders:      list[str]     = []
    config_exclude_dirs: set[str]      = set()
    if args.config:
        config_path = Path(args.config).expanduser().resolve()
        if not config_path.exists():
            print(f"Error: config not found: {config_path}", file=sys.stderr)
            sys.exit(1)
        cfg     = json.loads(config_path.read_text())
        version = cfg.get('version', cfg)          # support both flat and versioned formats
        config_folders      = [str(Path(f).expanduser()) for f in version.get('folders', [])]
        config_exclude_dirs = set(version.get('exclude_dirs', []))
        print(f"Config: {config_path.name}")
        if config_exclude_dirs:
            print(f"  Excluding: {sorted(config_exclude_dirs)}")

    output_path = Path(args.output).expanduser().resolve()
    raw_client  = anthropic.Anthropic(api_key=api_key)

    # ── Cost tracking: wrap both .messages.stream and .beta.messages.stream ──
    _calls = [0]; _in_tok = [0]; _out_tok = [0]
    _orig_stream      = raw_client.messages.stream
    _orig_beta_stream = raw_client.beta.messages.stream

    class _TrackingCM:
        def __init__(self, ctx):
            self._ctx = ctx; self._s = None
        def __enter__(self):
            self._s = self._ctx.__enter__(); return self._s
        def __exit__(self, *a):
            r = self._ctx.__exit__(*a)
            try:
                msg = self._s.get_final_message()
                u   = msg.usage
                _calls[0]  += 1
                _in_tok[0]  += u.input_tokens
                _out_tok[0] += u.output_tokens
            except Exception:
                pass
            return r

    def _tracked_stream(**kw):
        return _TrackingCM(_orig_stream(**kw))

    def _tracked_beta_stream(**kw):
        return _TrackingCM(_orig_beta_stream(**kw))

    raw_client.messages.stream      = _tracked_stream
    raw_client.beta.messages.stream = _tracked_beta_stream
    client = raw_client
    # ─────────────────────────────────────────────────────────────────────────

    folders = list(args.folders) or config_folders
    if not folders and not args.retry_merge:
        print("No folders specified. Enter folder paths one per line (blank line to finish):")
        while True:
            line = input("  Folder: ").strip()
            if not line:
                break
            folders.append(line)
        if not folders:
            print("No folders provided. Exiting.")
            sys.exit(1)

    run(folders, output_path, client,
        retry_merge=args.retry_merge,
        temperature=args.temperature,
        num_runs=args.runs,
        vote_threshold=args.vote_threshold,
        exclude_dirs=config_exclude_dirs or None,
        kb_path=Path(args.kb).expanduser().resolve())

    PRICE_IN  = 3.0  / 1_000_000
    PRICE_OUT = 15.0 / 1_000_000
    cost = _in_tok[0] * PRICE_IN + _out_tok[0] * PRICE_OUT
    print(f"\nCost: ${cost:.4f}  ({_calls[0]} calls, "
          f"{_in_tok[0]:,} in / {_out_tok[0]:,} out tokens)")
    tokens_path = output_path.with_name(output_path.stem + "-tokens.json")
    tokens_path.write_text(json.dumps({
        "calls": _calls[0], "input_tokens": _in_tok[0],
        "output_tokens": _out_tok[0], "total_tokens": _in_tok[0] + _out_tok[0],
    }, indent=2))

    output_dir = output_path.parent
    print(f"\nView with:  cd {output_dir} && python3 -m http.server 8000")
    print(f"Then open:  http://localhost:8000/twin-peaks.html")


if __name__ == '__main__':
    main()
