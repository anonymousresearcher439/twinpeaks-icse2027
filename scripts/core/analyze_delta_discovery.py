#!/usr/bin/env python3
"""
analyze_delta_discovery.py — Stage 1 of delta analysis: discover ASGs/ADs from changed code.

Compares a new version source directory against a prior version source directory,
extracts only the files that are new or changed, then runs N independent extraction
passes (same as analyze_architecture.py Stages 1-3) on just those delta files.

The output is a plain twin-peaks JSON containing only the ASGs/ADs discovered
from the delta — not yet merged with the prior model. This is the input to the
next step (alignment with prior model).

Usage:
    python3 -u analyze_delta_discovery.py <new-source-dir> \\
        --prior-source <old-source-dir> \\
        --output <output-dir>/delta-discovery.json \\
        [--runs 5] [--kb ../KB-iso.json]

Output:
    <output>.json           — twin-peaks model of delta discoveries
    <output-dir>/           — per-run checkpoints and intermediate files
"""

import argparse
import hashlib
import json
import math
import os
import sys
import time
from datetime import date
from pathlib import Path

import anthropic
from dotenv import load_dotenv

load_dotenv()

from twin_peaks_postprocess import (
    collect_files, split_chunks,
    call_claude, parse_json_response, _try_parse_json,
    MODEL, MERGE_TOKENS, MERGE_BETAS,
    vote_and_synthesize_ags, cluster_and_synthesize_ads,
    prune_orphan_ags,
)

# ── re-use the same extraction prompt as analyze_architecture.py ──────────────

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
    ✓  "Thread-Safe Concurrent Message Dispatch"              → AG (quality property)

Return ONLY a valid JSON object. No markdown fences, no explanation, no preamble.
Within any JSON string value, escape double-quote characters as \".
  Correct:   "evidence": "uses \\"alien\\" widget approach"
  Incorrect: "evidence": "uses "alien" widget approach"
"""

MAX_TOKENS = 32_000
CHUNK_SIZE = 350_000


def _file_hash(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def collect_delta_files(new_dir: Path, prior_dir: Path,
                        exclude_dirs: set[str] | None = None) -> str:
    """
    Walk new_dir and collect files that are either:
      - not present in prior_dir (new files), or
      - present in both but with different content (modified files).

    Returns the collected content in the same format as collect_files().
    """
    EXTENSIONS = {
        '.py', '.java', '.kt', '.js', '.ts', '.cpp', '.c', '.h',
        '.go', '.rs', '.rb', '.cs', '.scala', '.swift', '.yaml',
        '.yml', '.json', '.xml', '.gradle', '.cmake', '.sh',
        '.cfg', '.conf', '.ini', '.toml', '.md',
    }
    exclude_dirs = exclude_dirs or set()

    delta_files: list[Path] = []
    new_dir = new_dir.resolve()
    prior_dir = prior_dir.resolve()

    for root, dirs, files in os.walk(new_dir):
        root_path = Path(root)
        rel_root = root_path.relative_to(new_dir)

        # Prune excluded dirs
        dirs[:] = [d for d in dirs
                   if d not in exclude_dirs and not d.startswith('.')]

        for fname in sorted(files):
            fpath = root_path / fname
            if fpath.suffix.lower() not in EXTENSIONS:
                continue

            rel_path = rel_root / fname
            prior_path = prior_dir / rel_path

            if not prior_path.exists():
                delta_files.append(fpath)  # new file
            elif _file_hash(fpath) != _file_hash(prior_path):
                delta_files.append(fpath)  # modified file

    if not delta_files:
        return ""

    parts = []
    total_chars = 0
    for fpath in delta_files:
        try:
            text = fpath.read_text(errors='replace')
        except Exception:
            continue
        rel = fpath.relative_to(new_dir)
        parts.append(f"=== {rel} ===\n{text}\n")
        total_chars += len(text)

    print(f"  Delta: {len(delta_files)} new/changed files / {total_chars:,} characters.")
    return "".join(parts)


def _build_extraction_prompt(code_content: str, repo_names: list[str],
                             today: str, chunk_idx: int = 0,
                             total_chunks: int = 1) -> str:
    chunk_note = ""
    if total_chunks > 1:
        chunk_note = (
            f"\nNOTE: This is chunk {chunk_idx} of {total_chunks}. "
            "Analyse only the code provided; a separate merge step will unify results.\n"
        )

    return f"""\
You are performing Twin Peaks architectural analysis on a set of source files.
{chunk_note}
These files represent CHANGED or NEW code in a new version of the system.
Extract all Architectural Goals (AGs) and Architectural Decisions (ADs) that are
evidenced in these files.

Repository: {', '.join(repo_names)}
Date: {today}

An AG is a technology-independent quality requirement (WHAT).
An AD is a concrete design decision (HOW) — names implementations, libraries, patterns.

Each AG must pass the STATABILITY TEST: an architect could name this concern
without having read the code.

1. AGs — quality properties evidenced in this code:
   For each AG include: id, kind="ag", label, statement, evidence, rationale,
   confidence (explicit/strongly_inferred/weakly_inferred), safety_critical,
   affected_components.

2. ADs — concrete decisions evidenced in this code:
   For each AD include: id, kind="ad", label, decision, evidence, rationale,
   confidence, safety_critical, affected_components.

3. TYPED LINKS (direction: AD → AG)
   - makes  : AD is the primary mechanism implementing or enacting this AG
   - helps  : AD contributes positively to this AG but is not the primary mechanism
   - harms  : AD negatively affects this AG (an accepted trade-off)

Return ONLY this JSON schema:

{{
  "meta": {{
    "schema_version": "1.0",
    "created": "{today}",
    "last_modified": "{today}",
    "description": "Twin Peaks extraction from delta code.",
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
      "affected_components": ["<component1>"]
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
    {{ "source": "AD-01", "target": "AG-01", "link_type": "makes", "connection_rationale": "<one sentence: how/why this AD satisfies or contributes to this AG>" }}
  ]
}}

Include as many AGs and ADs as the evidence supports.

DELTA CODE
==========
{code_content}
"""


def _build_merge_prompt(partials: list[dict], repo_names: list[str],
                        today: str) -> str:
    nodes_json = json.dumps([p.get('nodes', []) for p in partials], indent=2)
    edges_json = json.dumps([p.get('edges', []) for p in partials], indent=2)
    return f"""\
You are merging {len(partials)} partial Twin Peaks extractions from different
chunks of the same delta codebase into a single unified result.

Repository: {', '.join(repo_names)}
Date: {today}

Rules:
- Merge duplicate or overlapping AGs into a single canonical AG.
- Merge duplicate or overlapping ADs into a single canonical AD.
- Preserve all distinct AGs and ADs; do not drop anything without a clear reason.
- Re-sequence IDs: AGs as AG-01, AG-02, ...; ADs as AD-01, AD-02, ...
- Rebuild edges referencing the new canonical IDs.
- Return ONLY valid JSON matching the twin-peaks schema. No markdown, no explanation.

PARTIAL NODES (one list per chunk):
{nodes_json}

PARTIAL EDGES (one list per chunk):
{edges_json}
"""


def _analyze_chunks(client: anthropic.Anthropic, chunks: list[str],
                    repo_names: list[str], today: str,
                    output_dir: Path, prefix: str,
                    temperature: float) -> dict:
    """One extraction + merge pass. Returns a raw twin-peaks dict."""
    total = len(chunks)
    if total == 1:
        for attempt in range(1, 3):
            print(f"  Sending to Claude ({MODEL})...")
            t0  = time.time()
            raw = call_claude(
                client,
                _build_extraction_prompt(chunks[0], repo_names, today),
                system=SYSTEM_PROMPT,
                max_tokens=MERGE_TOKENS,
                betas=MERGE_BETAS,
                temperature=temperature,
            )
            print(f"  done in {time.time()-t0:.1f}s")
            result = _try_parse_json(raw, f'{prefix}-single', output_dir)
            if result is not None:
                return result
            if attempt == 1:
                print(f"  WARNING: JSON parse failed — retrying once...", file=sys.stderr)
        print(f"\nFailed to parse Claude response ({prefix}-single) after 2 attempts", file=sys.stderr)
        sys.exit(1)

    partials = []
    for i, chunk in enumerate(chunks, 1):
        partial_path = output_dir / f'{prefix}-partial-{i}.json'
        if partial_path.exists():
            print(f"  Chunk {i}/{total}: loading cached {partial_path.name}")
            partials.append(json.loads(partial_path.read_text()))
            continue
        print(f"  Chunk {i}/{total} ({len(chunk):,} chars)...")
        t0  = time.time()
        raw = call_claude(
            client,
            _build_extraction_prompt(chunk, repo_names, today, i, total),
            system=SYSTEM_PROMPT,
            max_tokens=MAX_TOKENS,
            temperature=temperature,
        )
        print(f"  chunk {i} done in {time.time()-t0:.1f}s")
        partial = parse_json_response(raw, f'chunk{i}', output_dir)
        partial_path.write_text(json.dumps(partial, indent=2))
        partials.append(partial)

    print(f"  Merging {total} partials...")
    t0  = time.time()
    raw = call_claude(
        client,
        _build_merge_prompt(partials, repo_names, today),
        system=SYSTEM_PROMPT,
        max_tokens=MERGE_TOKENS,
        betas=MERGE_BETAS,
        temperature=0.0,
    )
    print(f"  merge done in {time.time()-t0:.1f}s")
    return parse_json_response(raw, f'{prefix}-merged', output_dir)


def run(new_source: Path, prior_source: Path | None,
        output_path: Path, client: anthropic.Anthropic,
        num_runs: int = 5, vote_threshold: int | None = None,
        exclude_dirs: set[str] | None = None,
        kb_path: Path | None = None) -> None:

    prefix     = output_path.stem
    output_dir = output_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)
    today      = date.today().isoformat()
    threshold  = vote_threshold if vote_threshold is not None else math.ceil(num_runs / 2)

    print(f"Delta discovery: {new_source.name}")
    print(f"Prior source   : {prior_source.name if prior_source else '(none — full analysis)'}")
    print(f"Output         : {output_path}")
    print(f"Runs           : {num_runs}  threshold ≥ {threshold}/{num_runs}")

    # ── Collect delta content ──────────────────────────────────────────────────
    if prior_source:
        print(f"\nComputing delta vs {prior_source} ...")
        code_content = collect_delta_files(new_source, prior_source, exclude_dirs)
    else:
        print(f"\nCollecting all files from {new_source} ...")
        code_content = collect_files([str(new_source)], exclude_dirs=exclude_dirs)
        char_count = len(code_content)
        file_count = code_content.count('\nFILE:') if code_content else 0
        print(f"  Collected {char_count:,} characters.")

    if not code_content or not code_content.strip():
        print("No delta files found — versions appear identical. Nothing to do.")
        sys.exit(0)

    chunks     = split_chunks(code_content, CHUNK_SIZE)
    repo_names = [new_source.name]
    print(f"  Split into {len(chunks)} chunk(s).")

    # ── Stage 1: N independent extraction runs ─────────────────────────────────
    print(f"\n── Stage 1: Extraction ({num_runs} independent runs) ──────────────────────")
    run_results: list[dict] = []
    for i in range(1, num_runs + 1):
        run_dir  = output_dir / f'{prefix}-run-{i}'
        run_path = run_dir / f'{prefix}.json'
        run_dir.mkdir(parents=True, exist_ok=True)
        if run_path.exists():
            print(f"\nRun {i}/{num_runs}: loading checkpoint {run_path.name}")
            run_results.append(json.loads(run_path.read_text()))
        else:
            print(f"\n{'─'*60}\nRun {i}/{num_runs}\n{'─'*60}")
            result = _analyze_chunks(client, chunks, repo_names, today,
                                     run_dir, prefix, temperature=1.0)
            run_path.write_text(json.dumps(result, indent=2))
            print(f"  Saved: {run_path}")
            run_results.append(result)

    # ── Stage 2: AG clustering and voting ─────────────────────────────────────
    print(f"\n── Stage 2: AG voting + cluster audit ─────────────────────────────────────")
    canonical_ags, ag_id_map = vote_and_synthesize_ags(
        client, run_results, threshold, output_dir, prefix
    )

    # ── Stage 3: AD clustering ─────────────────────────────────────────────────
    total_ads = sum(
        sum(1 for n in r.get('nodes', []) if n.get('kind') == 'ad')
        for r in run_results
    )
    print(f"\n── Stage 3: AD clustering ({total_ads} ADs across {num_runs} runs) ────────")
    deduped_ads, new_edges = cluster_and_synthesize_ads(
        client, run_results, ag_id_map, output_dir, prefix
    )
    print(f"  ADs after clustering: {len(deduped_ads)}")

    # ── Assemble output ────────────────────────────────────────────────────────
    first_meta = run_results[0].get('meta', {})
    result = {
        'meta': {
            'schema_version':    '1.0',
            'created':           today,
            'last_modified':     today,
            'description':       (
                f"Twin Peaks delta discovery: ASGs/ADs found in new/changed files "
                f"({new_source.name} vs {prior_source.name if prior_source else 'N/A'}). "
                f"Assembled from {num_runs} independent runs (threshold ≥ {threshold}/{num_runs})."
            ),
            'repositories':      repo_names,
            'delta_source':      str(new_source),
            'prior_source':      str(prior_source) if prior_source else None,
            'link_types':        first_meta.get('link_types', {}),
            'confidence_levels': first_meta.get('confidence_levels', {}),
        },
        'nodes': canonical_ags + deduped_ads,
        'edges': new_edges,
    }
    result = prune_orphan_ags(result)

    ags_out = [n for n in result['nodes'] if n.get('kind') == 'ag']
    ads_out = [n for n in result['nodes'] if n.get('kind') == 'ad']
    print(f"\nDelta discovery complete: {len(ags_out)} AGs + {len(ads_out)} ADs "
          f"({len(result['edges'])} edges)")

    output_path.write_text(json.dumps(result, indent=2))
    print(f"  Saved → {output_path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Discover ASGs/ADs from delta code (new/changed files between versions).'
    )
    parser.add_argument('new_source', help='New version source directory')
    parser.add_argument('--prior-source', required=False,
                        help='Prior version source directory (omit to analyse all files)')
    parser.add_argument('--output', required=True,
                        help='Output JSON path, e.g. v1/delta-discovery.json')
    parser.add_argument('--runs', type=int, default=5,
                        help='Number of independent extraction runs (default: 5)')
    parser.add_argument('--vote-threshold', type=int, default=None,
                        help='Min runs a cluster must appear in (default: ceil(runs/2))')
    parser.add_argument('--exclude-dirs', nargs='*', default=[],
                        help='Directory names to exclude')
    parser.add_argument('--kb', default=None,
                        help='Path to KB-iso.json for NFR classification (currently unused in this step)')
    args = parser.parse_args()

    api_key = os.environ.get('ANTHROPIC_API_KEY')
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

    new_source   = Path(args.new_source).expanduser().resolve()
    prior_source = Path(args.prior_source).expanduser().resolve() if args.prior_source else None
    output_path  = Path(args.output).expanduser()
    kb_path      = Path(args.kb).expanduser() if args.kb else None

    if not new_source.is_dir():
        print(f"ERROR: new source not found: {new_source}", file=sys.stderr)
        sys.exit(1)
    if prior_source and not prior_source.is_dir():
        print(f"ERROR: prior source not found: {prior_source}", file=sys.stderr)
        sys.exit(1)

    run(
        new_source=new_source,
        prior_source=prior_source,
        output_path=output_path,
        client=client,
        num_runs=args.runs,
        vote_threshold=args.vote_threshold,
        exclude_dirs=set(args.exclude_dirs) if args.exclude_dirs else None,
        kb_path=kb_path,
    )

    tokens_path = output_path.parent / f"{output_path.stem}-tokens.json"
    tokens_path.write_text(json.dumps({
        "calls": _calls[0], "input_tokens": _in_tok[0],
        "output_tokens": _out_tok[0], "total_tokens": _in_tok[0] + _out_tok[0],
    }, indent=2))


if __name__ == '__main__':
    main()
