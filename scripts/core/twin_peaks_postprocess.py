#!/usr/bin/env python3
"""
twin_peaks_postprocess.py — Shared post-processing passes for Twin Peaks models

Used by both analyze_architecture.py and analyze_next_version.py.

Passes (applied in order):
  1. consolidate_goals     — merge over-granular AGs that address the same concern
  2. refine_goals          — fix abstraction level: relabel contaminated AGs, split
                             bundled AGs, convert misclassified ADs
  3. prune_orphan_ags      — remove AGs with no incoming AD edges
  4. assess_ad_connections — classify each AD→AG edge as makes/helps/harms, add rationale,
                             prune weak edges (keeping at least one per AG)
"""

import json
import math
import re
import sys
import time
from collections import defaultdict
from pathlib import Path

import httpx

import anthropic

# ── Configuration ──────────────────────────────────────────────────────────────

MODEL            = "claude-sonnet-4-6"
MERGE_TOKENS     = 100_000
MERGE_BETAS      = ["output-300k-2026-03-24"]
CLUSTER_TOKENS      = 32_000
SYNTHESIS_TOKENS    = 32_000
AD_CLUSTER_TOKENS   = 32_000
AD_SYNTHESIS_TOKENS = 32_000

CONFIDENCE_RANK = {"explicit": 0, "strongly_inferred": 1, "weakly_inferred": 2}

# ── File collection ────────────────────────────────────────────────────────────

SOURCE_EXTENSIONS = {
    '.py', '.js', '.ts', '.jsx', '.tsx',
    '.java', '.go', '.rs', '.cpp', '.c', '.h',
    '.yaml', '.yml', '.toml', '.env', '.cfg', '.ini',
    '.md', '.sh', '.bash',
}

SKIP_DIRS = {
    'venv', '.venv', 'env', 'node_modules', '.git',
    '__pycache__', '.pytest_cache', 'dist', 'build',
    '.next', 'coverage', 'vendor', 'target', '.mypy_cache',
    'test', 'tests', '__tests__', 'migrations', 'fixtures',
}

SKIP_FILE_PATTERNS = {'test_', '_test.', '.test.', '.spec.'}

MAX_FILE_BYTES      = 500_000
TRUNCATE_FILE_CHARS = 5_000  # fallback for non-Java files

_JAVA_TYPE_DECL = re.compile(r'\b(class|interface|enum|record)\b|@interface')


def _count_braces_java(line: str) -> tuple[int, int]:
    """Count { and } in a line, ignoring string literals and // comments."""
    opens = closes = 0
    in_str = in_char = False
    i = 0
    while i < len(line):
        c = line[i]
        if in_str:
            if c == '\\':
                i += 1
            elif c == '"':
                in_str = False
        elif in_char:
            if c == '\\':
                i += 1
            elif c == "'":
                in_char = False
        elif c == '"':
            in_str = True
        elif c == "'":
            in_char = True
        elif c == '/' and i + 1 < len(line) and line[i + 1] == '/':
            break
        elif c == '{':
            opens += 1
        elif c == '}':
            closes += 1
        i += 1
    return opens, closes


def _extract_java_structure(text: str) -> str:
    """Return the structural skeleton of a Java file.

    Keeps package/imports, type declarations, field declarations, annotations,
    and method signatures. Drops method bodies to save context space.
    Multi-line signatures are preserved intact; one-liner methods are kept as-is.
    """
    out: list[str] = []
    depth = 0
    skip_until: int | None = None  # skip lines until depth returns to this value

    for raw in text.splitlines():
        stripped = raw.strip()
        opens, closes = _count_braces_java(stripped)

        if skip_until is not None:
            depth += opens - closes
            if depth <= skip_until:
                skip_until = None
            continue

        # Depth 0: package, imports, top-level comments/annotations — keep everything
        if depth == 0:
            out.append(raw)
            depth += opens - closes
            continue

        # Inside a type body (depth >= 1)
        if not stripped:
            continue  # drop blank lines to save space

        # A line that opens a new scope and is NOT a type declaration is a
        # method/constructor/initializer body — emit signature only, skip body.
        # One-liner methods (opens == closes) fall through and are kept whole.
        if opens > closes and not _JAVA_TYPE_DECL.search(stripped):
            brace_pos = raw.rfind('{')
            out.append(raw[:brace_pos].rstrip() + ' {...}')
            skip_until = depth
            depth += opens - closes
            continue

        out.append(raw)
        depth += opens - closes

    return '\n'.join(out)


def _extract_file_content(text: str, suffix: str,
                           truncate_chars: int = TRUNCATE_FILE_CHARS) -> str:
    """Extract architectural content from a source file by extension."""
    if suffix == '.java':
        return _extract_java_structure(text)
    if len(text) > truncate_chars:
        return text[:truncate_chars] + '\n... [truncated]\n'
    return text


def collect_files(folders: list[str],
                  truncate_chars: int = TRUNCATE_FILE_CHARS,
                  exclude_dirs: set[str] | None = None) -> str:
    """Walk folders and concatenate architectural content from source files."""
    skip_dirs = SKIP_DIRS | (exclude_dirs or set())
    parts: list[str] = []
    total_chars = 0
    skipped = 0

    for folder in folders:
        root_path = Path(folder).expanduser().resolve()
        if not root_path.exists():
            print(f"  Warning: folder not found: {folder}", file=sys.stderr)
            continue

        for path in sorted(root_path.rglob('*')):
            if any(d in path.parts for d in skip_dirs):
                continue
            if not path.is_file():
                continue
            if path.suffix.lower() not in SOURCE_EXTENSIONS:
                continue
            if any(pat in path.name for pat in SKIP_FILE_PATTERNS):
                continue
            if path.stat().st_size > MAX_FILE_BYTES:
                skipped += 1
                print(f"  Skipping (too large): {path}", file=sys.stderr)
                continue

            try:
                text = path.read_text(errors='ignore')
            except OSError:
                continue

            content = _extract_file_content(text, path.suffix.lower(), truncate_chars)
            rel     = path.relative_to(root_path.parent)
            block   = f"=== {rel} ===\n{content}\n"
            parts.append(block)
            total_chars += len(block)

    if skipped:
        print(f"  Skipped {skipped} file(s) exceeding {MAX_FILE_BYTES:,} bytes.",
              file=sys.stderr)
    print(f"  Collected {len(parts)} files / {total_chars:,} characters.", file=sys.stderr)
    return '\n'.join(parts)


def split_chunks(content: str, chunk_size: int) -> list[str]:
    """Split content into chunks at file boundaries where possible."""
    if len(content) <= chunk_size:
        return [content]

    chunks: list[str] = []
    current: list[str] = []
    current_len = 0

    for block in content.split('\n=== '):
        block = ('' if not current else '\n=== ') + block
        if current_len + len(block) > chunk_size and current:
            chunks.append(''.join(current))
            current = [block]
            current_len = len(block)
        else:
            current.append(block)
            current_len += len(block)

    if current:
        chunks.append(''.join(current))

    return chunks


# ── Shared utilities ───────────────────────────────────────────────────────────

_TRANSIENT_ERRORS = (
    httpx.RemoteProtocolError,
    httpx.ReadError,
    httpx.ConnectError,
    anthropic.APIConnectionError,
    anthropic.APITimeoutError,
    anthropic.InternalServerError,
    anthropic.APIStatusError,
)

def call_claude(client: anthropic.Anthropic, user_prompt: str,
                max_tokens: int,
                system: str = '',
                betas: list[str] | None = None,
                temperature: float = 1.0,
                max_retries: int = 2) -> str:
    kwargs = dict(
        model=MODEL,
        max_tokens=max_tokens,
        temperature=temperature,
        system=system,
        messages=[{"role": "user", "content": user_prompt}],
    )
    stream_fn = client.beta.messages.stream if betas else client.messages.stream
    if betas:
        kwargs["betas"] = betas
    for attempt in range(max_retries + 1):
        try:
            with stream_fn(**kwargs) as stream:
                return stream.get_final_text().strip()
        except _TRANSIENT_ERRORS as e:
            if attempt == max_retries:
                raise
            wait = 15 * (attempt + 1)
            print(f"\n  Transient error (attempt {attempt + 1}/{max_retries + 1}): {e}",
                  file=sys.stderr)
            print(f"  Retrying in {wait}s...", file=sys.stderr)
            time.sleep(wait)


def _close_truncated_json(text: str) -> dict | None:
    """Try to recover from a response truncated at the token limit.

    Scans the text character-by-character tracking bracket/string state, then
    appends the missing closing brackets/braces. Returns parsed dict on success,
    None if the text is too broken to repair.
    """
    stack:            list[str] = []
    in_string:        bool      = False
    escape:           bool      = False
    last_string_open: int       = -1  # index of the most recent unmatched opening "
    for i, ch in enumerate(text):
        if escape:
            escape = False
            continue
        if ch == '\\' and in_string:
            escape = True
            continue
        if ch == '"':
            if not in_string:
                last_string_open = i
            in_string = not in_string
            continue
        if in_string:
            continue
        if ch in '{[':
            stack.append('}' if ch == '{' else ']')
        elif ch in '}]':
            if stack and stack[-1] == ch:
                stack.pop()
    if not stack:
        return None   # already closed (or something else is wrong)
    # Drop any trailing partial token (incomplete string, dangling comma/colon)
    # by trimming back to the last complete value boundary.
    trimmed = text.rstrip()
    if in_string and last_string_open >= 0:
        # Truncated inside a string.  Cut back to just before the opening quote.
        trimmed = text[:last_string_open].rstrip()
        # Strip trailing separator chars (the ": " that preceded the value string,
        # or the "," that preceded the key string).
        while trimmed and trimmed[-1] in ',: \t\n':
            trimmed = trimmed[:-1]
        # If we're now sitting on a closing quote, that means we trimmed a value
        # and are now sitting at the end of its key string — e.g. ..."key".
        # Keep stripping key-value pairs until we're at a safe boundary
        # ({, [, or the end of a *value* string whose preceding char is ':').
        while trimmed and trimmed[-1] == '"':
            end   = len(trimmed) - 1
            start = trimmed.rfind('"', 0, end)
            if start < 0:
                break
            # Look at what precedes this string to decide if it's a key or value.
            pre_char = trimmed[:start].rstrip()[-1:] if trimmed[:start].rstrip() else ''
            if pre_char == ':':
                # It's a value string — safe boundary, stop stripping.
                break
            # It's a key string (preceded by ',', '{', '[', or start) — strip it.
            trimmed = trimmed[:start].rstrip()
            while trimmed and trimmed[-1] in ',: \t\n':
                trimmed = trimmed[:-1]
    while trimmed and trimmed[-1] in ',: \t\n':
        trimmed = trimmed[:-1]
    repaired = trimmed + ''.join(reversed(stack))
    try:
        return json.loads(repaired)
    except json.JSONDecodeError:
        return None


def _fix_invalid_json_escapes(text: str) -> str:
    r"""Replace backslash sequences that are invalid in JSON (e.g. \' or \_) with just the character."""
    return re.sub(r'\\([^"\\/bfnrtu])', r'\1', text)


def parse_json_response(raw: str, label: str, output_dir: Path = Path('.')) -> dict:
    import re as _re

    # Strip markdown fences if present
    text = raw
    if text.startswith('```'):
        text = text.split('\n', 1)[1]
        text = text.rsplit('```', 1)[0]

    # Fast path: response is pure JSON
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        pass

    # Escape-fix path: Claude sometimes emits invalid escape sequences like \' or \_
    text_fixed = _fix_invalid_json_escapes(text)
    try:
        return json.loads(text_fixed)
    except json.JSONDecodeError:
        pass

    # Robust path: model may have prefixed reasoning or added trailing text.
    # Find every { or [ that opens a line, try raw_decode from each, keep the
    # candidate whose JSON span is longest (i.e. the outermost object/array).
    decoder   = json.JSONDecoder()
    best_obj  = None
    best_span = -1
    for m in _re.finditer(r'^[\[{]', text_fixed, _re.MULTILINE):
        try:
            obj, end = decoder.raw_decode(text_fixed, m.start())
            if end > best_span:
                best_span = end
                best_obj  = obj
        except json.JSONDecodeError:
            continue
    if best_obj is not None:
        return best_obj

    # Repair path: response was truncated at the token limit.
    repaired = _close_truncated_json(text_fixed)
    if repaired is not None:
        print(f"  WARNING: {label} response was truncated — repaired by closing open brackets.",
              file=sys.stderr)
        return repaired

    dump_path = output_dir / f'raw-{label}.txt'
    print(f"\nFailed to parse Claude response ({label}) as JSON", file=sys.stderr)
    dump_path.write_text(raw)
    print(f"Raw response saved to {dump_path}", file=sys.stderr)
    sys.exit(1)


def _try_parse_json(raw: str, label: str, output_dir: Path) -> dict | None:
    """Like parse_json_response but returns None on failure instead of exiting."""
    import re as _re
    text = raw
    if text.startswith('```'):
        text = text.split('\n', 1)[1]
        text = text.rsplit('```', 1)[0]
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        pass
    # Escape-fix path: Claude sometimes emits invalid escape sequences like \' or \_
    text = _fix_invalid_json_escapes(text)
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        pass
    decoder = json.JSONDecoder()
    best_obj, best_span = None, -1
    for m in _re.finditer(r'^[\[{]', text, _re.MULTILINE):
        try:
            obj, end = decoder.raw_decode(text, m.start())
            if end > best_span:
                best_span, best_obj = end, obj
        except json.JSONDecodeError:
            continue
    if best_obj is not None:
        return best_obj
    repaired = _close_truncated_json(text)
    if repaired is not None:
        print(f"  WARNING: {label} response was truncated — repaired.", file=sys.stderr)
        return repaired
    # Progressive-strip repair: model sometimes emits extra closing brackets at the end
    # (e.g. `]}}` instead of `]]}`) which _close_truncated_json can't fix by appending.
    # Strip trailing bracket/whitespace chars one at a time and retry.
    stripped = text.rstrip()
    for _ in range(20):
        if not stripped or stripped[-1] not in '}]':
            break
        stripped = stripped[:-1].rstrip()
        while stripped and stripped[-1] in ',: \t\n':
            stripped = stripped[:-1]
        repaired = _close_truncated_json(stripped)
        if repaired is not None:
            print(f"  WARNING: {label} response had malformed trailing brackets — repaired.",
                  file=sys.stderr)
            return repaired
    # Save raw for inspection but do NOT exit
    dump_path = output_dir / f'raw-{label}.txt'
    dump_path.write_text(raw)
    print(f"  WARNING: failed to parse {label} response — raw saved to {dump_path.name}",
          file=sys.stderr)
    return None

# ── Goal consolidation ─────────────────────────────────────────────────────────

def _build_grouping_prompt(ags: list[dict]) -> str:
    lines = [
        f"  {ag['id']} | {ag.get('label','')} | "
        f"{ag.get('statement','')[:150].replace(chr(10),' ')}..."
        for ag in ags
    ]
    return (
        "Review these Architectural Goals extracted from a multi-repo codebase.\n"
        "Some are over-granular — sub-aspects of the same higher-level goal.\n\n"
        "AG list  (id | label | statement excerpt):\n"
        + "\n".join(lines)
        + "\n\nIdentify groups that should be merged into a single higher-level AG.\n"
        "Only include AGs that genuinely need merging; each group must have ≥ 2 members.\n\n"
        "Return a JSON array (may be empty):\n"
        '[{"ids": ["AG-XX", "AG-YY"], "reason": "one-line explanation"}, ...]'
    )


def _build_consolidation_merge_prompt(members: list[dict]) -> str:
    return (
        "Merge the following Architectural Goals into one consolidated AG.\n"
        "Produce a comprehensive yet concise label, statement, and rationale.\n"
        "The merged statement must be technology-independent — no protocol names,\n"
        "library names, or numeric thresholds.\n\n"
        f"Source AGs:\n{json.dumps(members, indent=2)}\n\n"
        'Return ONLY a JSON object: {"label": "...", "statement": "...", "rationale": "..."}'
    )


def _lowest_confidence(values: list[str]) -> str:
    return max(values, key=lambda c: CONFIDENCE_RANK.get(c, 99))


def consolidate_goals(result: dict, client: anthropic.Anthropic,
                      output_dir: Path, temperature: float = 1.0) -> dict:
    """Merge over-granular AG nodes; returns updated result dict."""
    nodes = result["nodes"]
    edges = result["edges"]
    ags = [n for n in nodes if n.get("kind") == "ag"]
    ads = [n for n in nodes if n.get("kind") == "ad"]

    print(f"\nConsolidating {len(ags)} AGs...")
    sys_prompt = ("You are an expert software architect. "
                  "Return ONLY valid JSON — no markdown fences, no explanation.")

    raw = call_claude(client, _build_grouping_prompt(ags), max_tokens=4096,
                      system=sys_prompt, temperature=temperature)
    groups: list[dict] = parse_json_response(raw, "consolidate-groups", output_dir)
    if not isinstance(groups, list):
        print("  Warning: unexpected grouping response — skipping consolidation.")
        return result

    ag_lookup = {ag["id"]: ag for ag in ags}

    # Collect edges from valid groups (≥2 real ids), then resolve overlapping
    # groups into disjoint components via union-find so no id appears in two groups.
    parent: dict[str, str] = {ag_id: ag_id for ag_id in ag_lookup}

    def find(x: str) -> str:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(x: str, y: str) -> None:
        px, py = find(x), find(y)
        if px != py:
            parent[px] = py

    any_merged = False
    for g in groups:
        ids = [i for i in g.get("ids", []) if i in ag_lookup]
        if len(ids) >= 2:
            for ag_id in ids[1:]:
                union(ids[0], ag_id)
            any_merged = True

    if not any_merged:
        print("  Nothing to consolidate.")
        return result

    # Build disjoint merge components (only those with ≥2 members)
    components: dict[str, list[str]] = defaultdict(list)
    for ag_id in ag_lookup:
        components[find(ag_id)].append(ag_id)
    merge_groups = [sorted(ids) for ids in components.values() if len(ids) >= 2]

    print(f"  {len(merge_groups)} group(s) identified")

    absorbed: set[str] = set()
    replacements: dict[str, dict] = {}

    for ids in merge_groups:
        canonical_id = ids[0]
        absorbed.update(ids[1:])
        members = [ag_lookup[i] for i in ids]

        raw = call_claude(client, _build_consolidation_merge_prompt(members),
                          max_tokens=2048, system=sys_prompt, temperature=temperature)
        synth: dict = parse_json_response(raw, f"consolidate-{canonical_id}", output_dir)

        all_evidence = "; ".join(filter(None, (m.get("evidence", "") for m in members)))
        all_components = list(dict.fromkeys(
            c for m in members for c in m.get("affected_components", [])
        ))
        replacements[canonical_id] = {
            "id": canonical_id,
            "kind": "ag",
            "label": synth.get("label", members[0].get("label", "")),
            "statement": synth.get("statement", ""),
            "evidence": all_evidence,
            "rationale": synth.get("rationale", ""),
            "confidence": _lowest_confidence(
                [m.get("confidence", "weakly_inferred") for m in members]
            ),
            "safety_critical": any(m.get("safety_critical", False) for m in members),
            "affected_components": all_components,
            **( {"_is_new": all(m.get("_is_new", False) for m in members)}
                if any("_is_new" in m for m in members) else {} ),
        }

    final_ags = [
        replacements.get(ag["id"], ag)
        for ag in ags if ag["id"] not in absorbed
    ]
    id_remap: dict[str, str] = {}
    for i, ag in enumerate(final_ags, 1):
        new_id = f"AG-{i:02d}"
        id_remap[ag["id"]] = new_id
        ag["id"] = new_id
    for ids in merge_groups:
        canonical_new = id_remap[ids[0]]
        for old_id in ids[1:]:
            id_remap[old_id] = canonical_new

    seen: set[tuple] = set()
    new_edges = []
    for edge in edges:
        tgt = id_remap.get(edge["target"], edge["target"])
        key = (edge["source"], tgt, edge["link_type"])
        if key not in seen:
            seen.add(key)
            new_edges.append({"source": edge["source"], "target": tgt,
                               "link_type": edge["link_type"]})

    result["nodes"] = final_ags + ads
    result["edges"] = new_edges
    result["meta"]["description"] += (
        f" Consolidated from {len(ags)} to {len(final_ags)} AGs."
    )
    print(f"  {len(ags)} AGs → {len(final_ags)} AGs  "
          f"({len(ags) - len(final_ags)} removed, {len(edges) - len(new_edges)} edges deduped)")
    return result


# ── Goal purity (refine) ───────────────────────────────────────────────────────

def _build_prior_ag_split_prompt(candidates: list[tuple]) -> str:
    """
    candidates: list of (ag, all_ads, new_ads) where new_ads ⊆ all_ads.
    Asks Claude whether each prior AG's ADs form distinct sub-concerns.
    Returns a JSON array — one entry per candidate.
    """
    sections = []
    for i, (ag, all_ads, new_ads) in enumerate(candidates, 1):
        ad_lines = [
            f"    {ad['id']}{'  [NEW]' if ad.get('_is_new') else ''}:  "
            f"{ad.get('label', '?')} — "
            f"{ad.get('decision', '')[:120].replace(chr(10), ' ')}"
            for ad in all_ads
        ]
        sections.append(
            f"── Candidate {i}:  {ag['id']} — {ag.get('label', '')} ──\n"
            f"  Statement : {ag.get('statement', '')[:200].replace(chr(10), ' ')}\n"
            f"  ADs total : {len(all_ads)}   (new this version: {len(new_ads)})\n"
            + "\n".join(ad_lines)
        )

    return (
        "The following prior Architectural Goals have accumulated new Architectural\n"
        "Decisions (ADs) in the current version. Assess whether each AG's supporting\n"
        "ADs cluster into meaningfully distinct sub-concerns that justify splitting.\n\n"
        "Only recommend split: true when the ADs clearly address two or more\n"
        "independently-motivatable quality concerns — not merely different mechanisms\n"
        "for the same concern. AG statements must remain technology-independent and\n"
        "each child must address exactly one quality concern.\n\n"
        + "\n\n".join(sections)
        + "\n\nFor each candidate return split: false or split: true with child definitions.\n"
          "When splitting, assign EVERY AD to exactly one child via ad_ids.\n\n"
          "Return ONLY this JSON array:\n"
          "[\n"
          "  {\"ag_id\": \"<prior AG id>\", \"split\": false},\n"
          "  {\n"
          "    \"ag_id\": \"<prior AG id>\",\n"
          "    \"split\": true,\n"
          "    \"reason\": \"<one sentence: why this AG warrants splitting>\",\n"
          "    \"children\": [\n"
          "      {\n"
          "        \"label\": \"<short descriptive name>\",\n"
          "        \"statement\": \"<technology-independent, one quality concern>\",\n"
          "        \"rationale\": \"<why this is a distinct concern>\",\n"
          "        \"ad_ids\": [\"<AD-id belonging to this child>\"]\n"
          "      }\n"
          "    ]\n"
          "  }\n"
          "]\n"
    )


def _build_meta_check_prompt(ags: list[dict]) -> str:
    return (
        "You are auditing Architectural Goals (AGs) from a Twin Peaks traceability graph.\n\n"
        "DISTINCTION:\n"
        "  AG — a QUALITY REQUIREMENT: states WHAT quality the system must achieve,\n"
        "       independently of how it is implemented. An AG statement must remain\n"
        "       valid even if the entire technology stack were replaced.\n"
        "  AD — a DESIGN DECISION: states HOW (specific technology, algorithm, pattern).\n\n"
        "STATABILITY TEST — apply before assigning valid_ag:\n"
        "  Could an architect name this quality concern without having read the code —\n"
        "  is it a property they would independently recognise as important for this\n"
        "  kind of system? If naming the concern requires knowing a specific class,\n"
        "  method, lock type, mechanism, or implementation detail, assign convert_to_ad.\n"
        "    ✗  'Reentrant Lock Protection of MessageSender State'     → convert_to_ad\n"
        "    ✗  'Timer-Based Message Sender for Timed State Durations' → convert_to_ad\n"
        "    ✓  'Thread-Safe Concurrent Message Dispatch'              → valid_ag\n\n"
        "For each AG assign one of four verdicts:\n\n"
        '  "valid_ag"      — genuinely a quality requirement, correctly abstracted.\n\n'
        '  "relabel"       — the underlying concern is a valid AG, but the label or\n'
        "                    statement is contaminated with technology names, protocol\n"
        "                    names, numeric thresholds, parameter names, or topic strings.\n"
        "                    Use relabel when a simple rewording fixes it.\n"
        "                    Provide: new_label, new_statement, and optionally new_rationale.\n"
        "                    Example: 'The MQTT layer must reconnect' →\n"
        "                             'The communication layer must recover from transient\n"
        "                              network interruptions without operator intervention.'\n\n"
        '  "split"         — the AG bundles two or more distinct quality concerns\n'
        "                    (signal: label or statement contains 'and', a comma, or\n"
        "                    a semicolon joining separate attributes).\n"
        "                    Provide: new_ags — a list of focused AG objects, one per concern:\n"
        "                    [{label, statement, rationale}, ...]\n\n"
        '  "convert_to_ad" — fails the statability test: names a class, method, lock\n'
        "                    type, mechanism, or implementation artifact rather than a\n"
        "                    quality property. It answers HOW, not WHAT quality is needed.\n"
        "                    Common signals:\n"
        "                      • Label contains a class name (MessageSender, Handler, Registry)\n"
        "                      • Label contains a mechanism (Reentrant Lock, Thread Pool, Timer)\n"
        "                      • The concern only makes sense after reading specific code files\n"
        "                    Provide:\n"
        "                      new_ag: the underlying quality requirement stated technology-\n"
        "                              independently (label names the QUALITY, not the mechanism)\n"
        "                      new_ad: the original AG rephrased as a design decision\n\n"
        f"AGs to audit:\n{json.dumps(ags, indent=2)}\n\n"
        "Return ONLY this JSON (one entry per AG, in the same order):\n"
        '{"assessments": [\n'
        '  {"id": "AG-XX", "verdict": "valid_ag"},\n'
        '  {"id": "AG-XX", "verdict": "relabel",\n'
        '   "new_label": "...", "new_statement": "...", "new_rationale": "..."},\n'
        '  {"id": "AG-XX", "verdict": "split",\n'
        '   "new_ags": [{"label": "...", "statement": "...", "rationale": "..."}]},\n'
        '  {"id": "AG-XX", "verdict": "convert_to_ad",\n'
        '   "new_ag": {"label": "...", "statement": "...", "rationale": "..."},\n'
        '   "new_ad": {"label": "...", "decision": "...", "rationale": "..."}}\n'
        ']}'
    )


def refine_goals(result: dict, client: anthropic.Anthropic, output_dir: Path,
                 temperature: float = 1.0) -> tuple[dict, list[dict]]:
    """AG purity pass. Returns (updated_result, split_provenance).

    Baseline context (no _is_new flags on nodes): all four verdicts apply to
    every AG — existing behaviour, unchanged.

    Evolution context (_is_new present):
      - Prior AGs (is_new=False or flag absent): valid_ag only, OR split when
        new_ad_count >= 2 or total_ad_count > 4 AND Claude confirms sub-clusters.
      - New AGs (is_new=True): all four verdicts unchanged.

    split_provenance is a list of split-event dicts for the evolution_report;
    always empty in baseline context or when no splits occur.
    """
    nodes = result["nodes"]
    edges = result["edges"]
    ags   = [n for n in nodes if n.get("kind") == "ag"]
    ads   = [n for n in nodes if n.get("kind") == "ad"]

    is_evolution = any("_is_new" in n for n in ags)
    print(f"\nRefining {len(ags)} AGs (purity pass"
          + ("  — evolution mode" if is_evolution else "") + ")...")

    sys_refine = (
        "You are an expert software architect applying the Twin Peaks model. "
        "Return ONLY valid JSON — no markdown fences, no explanation."
    )
    ag_lookup = {ag["id"]: ag for ag in ags}
    ad_lookup = {ad["id"]: ad for ad in ads}

    _ag_ctr = [len(ags) + 1]
    _ad_ctr = [len(ads) + 1]

    def next_tmp_ag() -> str:
        tid = f"AG-TMP-{_ag_ctr[0]}"; _ag_ctr[0] += 1; return tid

    def next_tmp_ad() -> str:
        tid = f"AD-TMP-{_ad_ctr[0]}"; _ad_ctr[0] += 1; return tid

    # ── Build assess_map ──────────────────────────────────────────────────────
    if is_evolution:
        prior_ags        = [ag for ag in ags if not ag.get("_is_new", False)]
        new_ags_to_check = [ag for ag in ags if ag.get("_is_new", False)]

        # Split candidates: new_ad_count >= 2 OR total_ad_count > 4
        ag_to_all_ads: dict[str, list[dict]] = defaultdict(list)
        for edge in edges:
            src, tgt = edge.get("source"), edge.get("target")
            if src in ad_lookup and tgt in ag_lookup:
                ag_to_all_ads[tgt].append(ad_lookup[src])

        split_candidates = [
            (ag,
             ag_to_all_ads[ag["id"]],
             [ad for ad in ag_to_all_ads[ag["id"]] if ad.get("_is_new")])
            for ag in prior_ags
            if (len([ad for ad in ag_to_all_ads[ag["id"]] if ad.get("_is_new")]) >= 2
                or len(ag_to_all_ads[ag["id"]]) > 4)
        ]

        split_decisions: dict[str, dict] = {}
        if split_candidates:
            ids_str = ", ".join(ag["id"] for ag, _, _ in split_candidates)
            print(f"  Prior AG split candidates: {len(split_candidates)}  ({ids_str})")
            t0  = time.time()
            raw = call_claude(
                client, _build_prior_ag_split_prompt(split_candidates),
                max_tokens=8192, system=sys_refine, temperature=0.0,
            )
            print(f"  Split check done in {time.time()-t0:.1f}s")
            response = parse_json_response(raw, "refine-prior-split", output_dir)
            entries  = response if isinstance(response, list) else response.get("results", [])
            for entry in (entries if isinstance(entries, list) else []):
                if isinstance(entry, dict) and "ag_id" in entry:
                    split_decisions[entry["ag_id"]] = entry

        # Prior AGs: only split (if confirmed) or valid_ag
        assess_map: dict[str, dict] = {}
        for ag in prior_ags:
            aid = ag["id"]
            dec = split_decisions.get(aid, {})
            if dec.get("split") and len(dec.get("children", [])) >= 2:
                assess_map[aid] = {
                    "id": aid, "verdict": "split",
                    "new_ags": dec["children"],
                    "_split_reason": dec.get("reason", ""),
                }
            else:
                assess_map[aid] = {"id": aid, "verdict": "valid_ag"}

        # New AGs: full four-verdict pass
        if new_ags_to_check:
            t0  = time.time()
            raw = call_claude(client, _build_meta_check_prompt(new_ags_to_check),
                              max_tokens=8192, system=sys_refine, temperature=temperature)
            print(f"  New AG purity check done in {time.time()-t0:.1f}s")
            resp = parse_json_response(raw, "refine-meta-check", output_dir)
            for a in (resp.get("assessments", []) if isinstance(resp, dict) else []):
                assess_map[a["id"]] = a

    else:
        # Baseline: one call covering all AGs
        t0  = time.time()
        raw = call_claude(client, _build_meta_check_prompt(ags), max_tokens=8192,
                          system=sys_refine, temperature=temperature)
        print(f"  AG purity check done in {time.time()-t0:.1f}s")
        resp       = parse_json_response(raw, "refine-meta-check", output_dir)
        assess_map = {a["id"]: a
                      for a in (resp.get("assessments", []) if isinstance(resp, dict) else [])}

    verdicts: dict[str, int] = {"valid_ag": 0, "relabel": 0, "split": 0, "convert_to_ad": 0}
    for ag in ags:
        v = assess_map.get(ag["id"], {}).get("verdict", "valid_ag")
        verdicts[v] = verdicts.get(v, 0) + 1
    print(f"  valid={verdicts['valid_ag']}  relabel={verdicts['relabel']}  "
          f"split={verdicts['split']}  convert_to_ad={verdicts['convert_to_ad']}")

    # ── Process each AG ───────────────────────────────────────────────────────
    new_ags:         list[dict]     = []
    new_ads_extra:   list[tuple]    = []   # (ad_dict, target_tmp_ag_id)
    orig_to_tmp:     dict[str, str] = {}
    ad_to_child_tmp: dict[str, str] = {}   # ad_id -> child tmp AG id (split edge routing)
    split_raw:       list[dict]     = []   # provenance with tmp IDs; resolved after renumber

    for ag in ags:
        aid     = ag["id"]
        a       = assess_map.get(aid, {})
        verdict = a.get("verdict", "valid_ag")

        if verdict in ("valid_ag", "missing"):
            new_ags.append(ag)
            orig_to_tmp[aid] = aid

        elif verdict == "relabel":
            updated = dict(ag)
            updated["label"]     = a.get("new_label",     ag["label"])
            updated["statement"] = a.get("new_statement", ag["statement"])
            if "new_rationale" in a:
                updated["rationale"] = a["new_rationale"]
            new_ags.append(updated)
            orig_to_tmp[aid] = aid

        elif verdict == "split":
            split_ags = a.get("new_ags", [])
            if len(split_ags) < 2:
                new_ags.append(ag)
                orig_to_tmp[aid] = aid
            else:
                child_tmps: list[str] = []
                for ag_data in split_ags:
                    tmp = next_tmp_ag()
                    child_tmps.append(tmp)
                    new_ags.append({
                        "id": tmp, "kind": "ag",
                        "label":               ag_data.get("label",    ag["label"]),
                        "statement":           ag_data.get("statement", ""),
                        "evidence":            ag.get("evidence", ""),
                        "rationale":           ag_data.get("rationale", ag.get("rationale", "")),
                        "confidence":          ag.get("confidence", "strongly_inferred"),
                        "safety_critical":     ag.get("safety_critical", False),
                        "affected_components": ag.get("affected_components", []),
                    })
                    for ad_id in ag_data.get("ad_ids", []):
                        ad_to_child_tmp[ad_id] = tmp
                orig_to_tmp[aid] = child_tmps[0]
                split_raw.append({
                    "from_id":     aid,
                    "from_label":  ag.get("label", ""),
                    "reason":      a.get("_split_reason", ""),
                    "child_tmps":  child_tmps,
                    "child_labels": [d.get("label", "") for d in split_ags],
                })

        elif verdict == "convert_to_ad":
            new_ag_data = a.get("new_ag", {})
            new_ad_data = a.get("new_ad", {})
            if not isinstance(new_ag_data, dict):
                print(f"  WARNING: convert_to_ad for {aid} — new_ag was {type(new_ag_data).__name__!r}, "
                      f"not a dict. Raw value: {new_ag_data!r:.120}", file=sys.stderr)
                new_ag_data = {}
            if not isinstance(new_ad_data, dict):
                print(f"  WARNING: convert_to_ad for {aid} — new_ad was {type(new_ad_data).__name__!r}, "
                      f"not a dict. Raw value: {new_ad_data!r:.120}", file=sys.stderr)
                new_ad_data = {}
            thin_id = next_tmp_ag()
            new_ags.append({
                "id": thin_id, "kind": "ag",
                "label":               new_ag_data.get("label",    ag["label"]),
                "statement":           new_ag_data.get("statement", ""),
                "evidence":            "",
                "rationale":           new_ag_data.get("rationale", ag.get("rationale", "")),
                "confidence":          ag.get("confidence", "strongly_inferred"),
                "safety_critical":     ag.get("safety_critical", False),
                "affected_components": ag.get("affected_components", []),
            })
            new_ads_extra.append(({
                "id": next_tmp_ad(), "kind": "ad",
                "label":               new_ad_data.get("label",    ag["label"]),
                "decision":            new_ad_data.get("decision", ag.get("statement", "")),
                "evidence":            ag.get("evidence", ""),
                "rationale":           new_ad_data.get("rationale", ag.get("rationale", "")),
                "confidence":          ag.get("confidence", "strongly_inferred"),
                "safety_critical":     ag.get("safety_critical", False),
                "affected_components": ag.get("affected_components", []),
            }, thin_id))
            orig_to_tmp[aid] = thin_id

    # ── Renumber AGs ─────────────────────────────────────────────────────────
    tmp_to_ag: dict[str, str] = {}
    if is_evolution:
        # Prior AGs keep their original IDs; only split children and genuinely new AGs
        # get new sequential IDs above max_prior_ag.
        prior_kept_ags = [ag for ag in new_ags
                          if not ag.get("_is_new", False) and not ag["id"].startswith("AG-TMP-")]
        new_tmp_ags    = [ag for ag in new_ags
                          if ag.get("_is_new", False) or ag["id"].startswith("AG-TMP-")]
        max_prior = 0
        for ag in prior_kept_ags:
            m = re.match(r"AG-(\d+)", ag["id"])
            if m:
                max_prior = max(max_prior, int(m.group(1)))
        for ag in prior_kept_ags:
            tmp_to_ag[ag["id"]] = ag["id"]   # identity: prior IDs are frozen
        new_tmp_ags.sort(key=lambda n: (0 if n.get("safety_critical") else 1))
        for i, ag in enumerate(new_tmp_ags, max_prior + 1):
            new_id = f"AG-{i:02d}"
            tmp_to_ag[ag["id"]] = new_id
            ag["id"] = new_id
    else:
        new_ags.sort(key=lambda n: (0 if n.get("safety_critical") else 1))
        for i, ag in enumerate(new_ags, 1):
            new_id = f"AG-{i:02d}"
            tmp_to_ag[ag["id"]] = new_id
            ag["id"] = new_id
    id_remap = {orig: tmp_to_ag.get(tmp, tmp) for orig, tmp in orig_to_tmp.items()}

    # ── Renumber ADs ─────────────────────────────────────────────────────────
    all_ads = list(ads)
    for i, ad in enumerate(all_ads, 1):
        ad["id"] = f"AD-{i:02d}"
    for idx, (new_ad, _) in enumerate(new_ads_extra, len(all_ads) + 1):
        new_ad["id"] = f"AD-{idx:02d}"
        all_ads.append(new_ad)

    # ── Remap edges ───────────────────────────────────────────────────────────
    # For split AGs, route each AD to its assigned child when specified.
    seen: set[tuple] = set()
    new_edges: list[dict] = []
    for edge in edges:
        src     = edge["source"]
        old_tgt = edge["target"]
        if src in ad_to_child_tmp:
            child_tmp = ad_to_child_tmp[src]
            tgt = tmp_to_ag.get(child_tmp, id_remap.get(old_tgt, old_tgt))
        else:
            tgt = id_remap.get(old_tgt, old_tgt)
        key = (src, tgt, edge["link_type"])
        if key not in seen:
            seen.add(key)
            new_edges.append({"source": src, "target": tgt, "link_type": edge["link_type"]})
    for new_ad, thin_tmp in new_ads_extra:
        tgt = tmp_to_ag.get(thin_tmp, thin_tmp)
        key = (new_ad["id"], tgt, "makes")
        if key not in seen:
            seen.add(key)
            new_edges.append({"source": new_ad["id"], "target": tgt,
                               "link_type": "makes"})

    # ── Resolve split provenance to final IDs ─────────────────────────────────
    split_provenance: list[dict] = [
        {
            "from_id":    sr["from_id"],
            "from_label": sr["from_label"],
            "reason":     sr["reason"],
            "children":   [
                {"id": tmp_to_ag.get(tmp, tmp), "label": lbl}
                for tmp, lbl in zip(sr["child_tmps"], sr["child_labels"])
            ],
        }
        for sr in split_raw
    ]

    result["nodes"] = new_ags + all_ads
    result["edges"] = new_edges
    result["meta"]["description"] += (
        f" Purity pass: {verdicts['relabel']} relabelled, "
        f"{verdicts['split']} split, "
        f"{verdicts['convert_to_ad']} converted to ADs."
    )
    print(f"  AGs: {len(ags)} → {len(new_ags)}  |  ADs: {len(ads)} → {len(all_ads)}")
    return result, split_provenance


# ── Orphan AG pruning ───────────────────────────────────────────────────────────

def prune_orphan_ags(result: dict) -> dict:
    """Remove AGs with no incoming AD edges (no implementation evidence)."""
    nodes = result["nodes"]
    edges = result["edges"]
    ags = [n for n in nodes if n.get("kind") == "ag"]
    ads = [n for n in nodes if n.get("kind") == "ad"]

    linked = {e["target"] for e in edges}
    kept   = [ag for ag in ags if ag["id"] in linked]
    pruned = [ag for ag in ags if ag["id"] not in linked]

    print(f"\nPruning orphan AGs (no linked AD)...")
    if not pruned:
        print("  None found.")
        return result

    for ag in pruned:
        print(f"  Pruned {ag['id']}: \"{ag['label']}\"")

    is_evolution = any("_is_new" in n for n in ags)

    if is_evolution:
        # In evolution mode, prior AG IDs must be preserved — just drop the orphans,
        # keep all edges (orphaned AGs have no incoming edges by definition).
        result["nodes"] = kept + ads
        result["edges"] = edges
    else:
        id_remap: dict[str, str] = {}
        for i, ag in enumerate(kept, 1):
            new_id = f"AG-{i:02d}"
            id_remap[ag["id"]] = new_id
            ag["id"] = new_id
        for i, ad in enumerate(ads, 1):
            ad["id"] = f"AD-{i:02d}"
        result["nodes"] = kept + ads
        result["edges"] = [
            {"source": e["source"],
             "target": id_remap.get(e["target"], e["target"]),
             "link_type": e["link_type"]}
            for e in edges
        ]

    result["meta"]["description"] += f" Pruned {len(pruned)} unimplemented AG(s)."
    print(f"  {len(ags)} AGs → {len(kept)} AGs  ({len(pruned)} removed)")
    return result


# ── Cross-run AG voting pipeline ──────────────────────────────────────────────

CLUSTER_SYSTEM = """\
You are a software architecture expert performing a consistency analysis.
Group Architectural Goals from multiple independent analysis runs by semantic
equivalence — same underlying quality concern, regardless of wording.
Return ONLY valid JSON. No markdown fences, no explanation, no preamble.
"""

SYNTHESIS_SYSTEM = """\
You are an expert software architect synthesizing Architectural Goals from multiple
independent analyses of the same codebase into a single canonical set.
Return ONLY valid JSON. No markdown fences, no explanation, no preamble.
"""


def build_ag_cluster_prompt(run_ags: dict[str, list[dict]]) -> str:
    n_runs    = len(run_ags)
    total_ags = sum(len(ags) for ags in run_ags.values())

    sections = ""
    for run_label, ags in run_ags.items():
        sections += f"\n{run_label}  ({len(ags)} AGs):\n"
        for ag in ags:
            stmt = ag.get("statement", "").replace("\n", " ")[:250]
            sections += f"  {ag['id']}: {ag.get('label', '?')} — {stmt}\n"

    return f"""\
Below are AGs from {n_runs} independent analysis runs of the same codebase.
Each run was fully independent (no shared state, no memory of other runs).
{sections}
TASK
====
Group AGs that address the same underlying quality concern, regardless of wording.
Two AGs belong in the same cluster if a knowledgeable software architect would agree
they describe the same quality property (e.g., both address communication reliability,
or both address runtime safety monitoring).

Err toward MERGING rather than splitting: group AGs together whenever there is a
reasonable case they address the same quality property, even if framing or scope
differs. Only create separate clusters when concerns are clearly addressing different
quality properties.

Rules:
- Every AG from every run must appear in exactly one cluster.
- Single-member clusters are allowed for AGs with no match in any other run.
- Use the run label exactly as given (e.g. Run1, Run2).

Return ONLY this JSON ({total_ags} AGs total — all must appear):
{{
  "clusters": [
    {{
      "canonical_concern": "<concise name for the quality concern>",
      "members": [
        {{"run": "<run label>", "id": "<AG id>", "label": "<AG label>"}}
      ]
    }}
  ]
}}
"""


def build_ag_synthesis_prompt(survivor_clusters: list[dict],
                               run_results: list[dict]) -> str:
    ag_lookup: dict[tuple[str, str], dict] = {}
    for i, result in enumerate(run_results):
        run_label = f"Run{i + 1}"
        for node in result.get('nodes', []):
            if node.get('kind') == 'ag':
                ag_lookup[(run_label, node['id'])] = node

    sections = ""
    for idx, cluster in enumerate(survivor_clusters, 1):
        sections += f"\n── Cluster {idx}: {cluster['canonical_concern']} ──\n"
        for member in cluster['members']:
            run  = member['run']
            agid = member['id']
            full = ag_lookup.get((run, agid), {})
            sections += f"  [{run}]\n"
            sections += f"    label          : {full.get('label', member.get('label', '?'))}\n"
            sections += f"    statement      : {full.get('statement', '').replace(chr(10), ' ')}\n"
            sections += f"    evidence       : {full.get('evidence', '').replace(chr(10), ' ')}\n"
            sections += f"    rationale      : {full.get('rationale', '').replace(chr(10), ' ')}\n"
            sections += f"    confidence     : {full.get('confidence', 'strongly_inferred')}\n"
            sections += f"    safety_critical: {full.get('safety_critical', False)}\n"
            comps = full.get('affected_components', [])
            if comps:
                sections += f"    components     : {', '.join(comps)}\n"

    return f"""\
Below are {len(survivor_clusters)} clusters of semantically equivalent Architectural
Goals, each containing variants from different analysis runs of the same codebase.

For each cluster, synthesize ONE canonical AG that:
- Uses the best-worded statement from all variants (or combines them)
- Unions the evidence across all variants (all specific files/functions/constants)
- Uses the best rationale
- Sets confidence to the highest level seen (explicit > strongly_inferred > weakly_inferred)
- Sets safety_critical = true if ANY variant marks it true
- Follows Twin Peaks abstraction rules:
    * Technology-independent — no protocol names, library names, numeric thresholds
    * Exactly one quality concern — no "and" or comma-joined concerns
{sections}
Return ONLY this JSON (one entry per cluster, in cluster order):
{{
  "canonical_ags": [
    {{
      "cluster_index": <1-based integer matching the cluster number above>,
      "label": "<short descriptive name>",
      "statement": "<full prose requirement — technology-independent, single concern>",
      "evidence": "<union of all evidence across variants>",
      "rationale": "<best rationale from all variants>",
      "confidence": "explicit|strongly_inferred|weakly_inferred",
      "safety_critical": true|false,
      "affected_components": ["<component>"]
    }}
  ]
}}
"""


def _build_cluster_audit_prompt(clusters: list[dict],
                                survivor_clusters: list[dict]) -> str:
    survivor_indices = {id(c) for c in survivor_clusters}
    lines = []
    for idx, c in enumerate(clusters, 1):
        survived = any(id(c) == id(s) for s in survivor_clusters)
        tag = "[survived]" if survived else "[pruned]"
        lines.append(f"\nCluster {idx} {tag}: {c['canonical_concern']}")
        for m in c["members"]:
            lines.append(f"  {m['run']} — {m['label']}")
    cluster_text = "\n".join(lines)

    return f"""\
You are auditing AG clusters from a Twin Peaks voting pipeline for boundary errors.

{cluster_text}

ROLE A — False Merge Detector:
For each survived cluster, check whether its member labels describe two clearly
distinct quality concerns that were grouped only because they share surface
vocabulary. Signal: members split into two non-overlapping concern groups with
different quality properties (e.g., safety vs. performance vs. extensibility).

ROLE B — False Split Detector:
Check pairs of survived clusters. Flag a pair as a false split when both
canonical_concern strings describe the same underlying quality property from
different angles — merging them would produce a single coherent concern.

ROLE C — Synthesis:
For each finding, assign severity (high/medium/low) and a proposed resolution.

Return ONLY this JSON (empty lists if no issues found):
{{
  "false_merges": [
    {{
      "cluster_index": <1-based int>,
      "severity": "high|medium|low",
      "reason": "<one sentence>",
      "proposed_split": ["<concern A name>", "<concern B name>"]
    }}
  ],
  "false_splits": [
    {{
      "cluster_indices": [<int>, <int>],
      "severity": "high|medium|low",
      "reason": "<one sentence>",
      "proposed_merge_name": "<unified concern name>"
    }}
  ]
}}
"""


def audit_cluster_boundaries(
        client: anthropic.Anthropic,
        clusters: list[dict],
        survivor_clusters: list[dict],
        output_dir: Path,
        prefix: str,
) -> list[dict]:
    """Detect false merges and false splits in the AG cluster set.

    Writes {prefix}-cluster-audit.json with findings (flags only, does not
    modify survivor_clusters). Returns the flags list.
    """
    audit_path = output_dir / f"{prefix}-cluster-audit.json"
    if audit_path.exists():
        print(f"  Loading cached cluster audit: {audit_path.name}")
        return json.loads(audit_path.read_text())

    print(f"  Auditing {len(survivor_clusters)} survivor clusters "
          f"({len(clusters)} total) for boundary errors...")
    t0  = time.time()
    raw = call_claude(
        client,
        _build_cluster_audit_prompt(clusters, survivor_clusters),
        max_tokens=4096,
        system="You are a software architecture expert auditing cluster groupings. "
               "Return ONLY valid JSON. No markdown fences, no explanation.",
        temperature=0.0,
    )
    print(f"  Cluster audit done in {time.time() - t0:.1f}s")
    result = parse_json_response(raw, "cluster-audit", output_dir)
    audit_path.write_text(json.dumps(result, indent=2))

    merges = result.get("false_merges", [])
    splits = result.get("false_splits", [])
    print(f"  Cluster audit: {len(merges)} false merge(s), {len(splits)} false split(s) flagged")
    if merges or splits:
        for m in merges:
            sev = m.get("severity", "?")
            print(f"    [merge/{sev}] cluster {m.get('cluster_index')}: {m.get('reason','')}")
        for s in splits:
            sev = s.get("severity", "?")
            idxs = s.get("cluster_indices", [])
            print(f"    [split/{sev}] clusters {idxs}: {s.get('reason','')}")
    print(f"  Saved: {audit_path.name}")
    return result


def vote_and_synthesize_ags(
        client: anthropic.Anthropic,
        run_results: list[dict],
        threshold: int,
        output_dir: Path,
        prefix: str,
) -> tuple[list[dict], dict[tuple[str, str], str]]:
    """
    Cluster AGs across N runs, apply chop threshold, synthesize canonical AGs.

    Returns:
        canonical_ags — list of AG node dicts with final sequential IDs
        ag_id_map     — {(run_label, old_ag_id): new_canonical_id} for edge remapping
    """
    n_runs = len(run_results)

    cluster_path = output_dir / f'{prefix}-vote-clusters.json'
    if cluster_path.exists():
        print(f"  Loading cached AG clusters: {cluster_path.name}")
        cluster_data = json.loads(cluster_path.read_text())
    else:
        run_ags = {
            f"Run{i + 1}": [n for n in r.get('nodes', []) if n.get('kind') == 'ag']
            for i, r in enumerate(run_results)
        }
        total_ags = sum(len(v) for v in run_ags.values())
        print(f"  Clustering {total_ags} AGs across {n_runs} runs...")
        t0  = time.time()
        raw = call_claude(client, build_ag_cluster_prompt(run_ags),
                          max_tokens=CLUSTER_TOKENS, system=CLUSTER_SYSTEM, temperature=0.0)
        print(f"  clustering done in {time.time() - t0:.1f}s")
        cluster_data = parse_json_response(raw, 'vote-clusters', output_dir)
        cluster_path.write_text(json.dumps(cluster_data, indent=2))
        print(f"  Saved: {cluster_path.name}  ({len(cluster_data['clusters'])} clusters)")

    clusters = cluster_data['clusters']

    survivor_clusters = [
        c for c in clusters
        if len(set(m['run'] for m in c['members'])) >= threshold
    ]
    pruned = len(clusters) - len(survivor_clusters)
    print(f"  Chop at ≥ {threshold}/{n_runs} runs: "
          f"{len(survivor_clusters)} survivors, {pruned} pruned")

    audit_cluster_boundaries(client, clusters, survivor_clusters, output_dir, prefix)

    synthesis_path = output_dir / f'{prefix}-vote-synthesis.json'
    if synthesis_path.exists():
        print(f"  Loading cached AG synthesis: {synthesis_path.name}")
        synthesis_data = json.loads(synthesis_path.read_text())
    else:
        print(f"  Synthesizing {len(survivor_clusters)} canonical AGs...")
        t0  = time.time()
        raw = call_claude(client,
                          build_ag_synthesis_prompt(survivor_clusters, run_results),
                          max_tokens=SYNTHESIS_TOKENS, system=SYNTHESIS_SYSTEM,
                          temperature=0.0)
        print(f"  synthesis done in {time.time() - t0:.1f}s")
        synthesis_data = parse_json_response(raw, 'vote-synthesis', output_dir)
        synthesis_path.write_text(json.dumps(synthesis_data, indent=2))

    raw_ags = synthesis_data.get('canonical_ags', [])
    if len(raw_ags) != len(survivor_clusters):
        print(f"  WARNING: synthesis returned {len(raw_ags)} AGs, "
              f"expected {len(survivor_clusters)}", file=sys.stderr)

    cluster_by_idx = {i + 1: c for i, c in enumerate(survivor_clusters)}
    ag_entries: list[tuple[dict, list]] = []
    for ag_data in raw_ags:
        cluster = cluster_by_idx.get(ag_data.get('cluster_index', 0), {})
        members = cluster.get('members', [])
        ag_entries.append((ag_data, members))

    ag_entries.sort(key=lambda e: (0 if e[0].get('safety_critical') else 1))

    canonical_ags: list[dict] = []
    ag_id_map: dict[tuple[str, str], str] = {}

    for i, (ag_data, members) in enumerate(ag_entries, 1):
        new_id = f"AG-{i:02d}"
        canonical_ags.append({
            'id':                  new_id,
            'kind':                'ag',
            'label':               ag_data.get('label', ''),
            'statement':           ag_data.get('statement', ''),
            'evidence':            ag_data.get('evidence', ''),
            'rationale':           ag_data.get('rationale', ''),
            'confidence':          ag_data.get('confidence', 'strongly_inferred'),
            'safety_critical':     ag_data.get('safety_critical', False),
            'affected_components': ag_data.get('affected_components', []),
        })
        for member in members:
            ag_id_map[(member['run'], member['id'])] = new_id

    return canonical_ags, ag_id_map


# ── Cross-run AD voting pipeline ──────────────────────────────────────────────

AD_CLUSTER_SYSTEM = """\
You are a software architecture expert performing a consistency analysis.
Group Architectural Decisions from multiple independent analysis runs by semantic
equivalence — same concrete design choice, regardless of wording.
Return ONLY valid JSON. No markdown fences, no explanation, no preamble.
"""

AD_SYNTHESIS_SYSTEM = """\
You are an expert software architect synthesizing Architectural Decisions from multiple
independent analyses of the same codebase into a single canonical set.
Return ONLY valid JSON. No markdown fences, no explanation, no preamble.
"""


def build_ad_cluster_prompt(run_ads: dict[str, list[dict]]) -> str:
    n_runs    = len(run_ads)
    total_ads = sum(len(ads) for ads in run_ads.values())

    sections = ""
    for run_label, ads in run_ads.items():
        sections += f"\n{run_label}  ({len(ads)} ADs):\n"
        for ad in ads:
            decision = ad.get("decision", "").replace("\n", " ")[:300]
            sections += f"  {ad['id']}: {ad.get('label', '?')} — {decision}\n"

    return f"""\
Below are ADs from {n_runs} independent analysis runs of the same codebase.
Each run was fully independent (no shared state, no memory of other runs).
{sections}
TASK
====
Group ADs that describe the same concrete design decision, regardless of wording.
Two ADs belong in the same cluster if a knowledgeable software architect would agree
they describe the exact same design choice (e.g., the same GoF pattern applied to the
same problem, the same protocol selection, the same configuration strategy).

Err toward SPLITTING rather than merging: keep ADs separate if there is any meaningful
difference in the technology, pattern, scope, or mechanism described. Only cluster when
the decisions are truly interchangeable descriptions of the same concrete choice.

Rules:
- Every AD from every run must appear in exactly one cluster.
- Single-member clusters are allowed for ADs with no match in any other run.
- Use the run label exactly as given (e.g. Run1, Run2).

Return ONLY this JSON ({total_ads} ADs total — all must appear):
{{
  "clusters": [
    {{
      "canonical_concern": "<concise name for the design decision>",
      "members": [
        {{"run": "<run label>", "id": "<AD id>", "label": "<AD label>"}}
      ]
    }}
  ]
}}
"""


def build_ad_synthesis_prompt(clusters: list[dict], run_results: list[dict]) -> str:
    ad_lookup: dict[tuple[str, str], dict] = {}
    for i, result in enumerate(run_results):
        run_label = f"Run{i + 1}"
        for node in result.get('nodes', []):
            if node.get('kind') == 'ad':
                ad_lookup[(run_label, node['id'])] = node

    sections = ""
    for idx, cluster in enumerate(clusters, 1):
        sections += f"\n── Cluster {idx}: {cluster['canonical_concern']} ──\n"
        for member in cluster['members']:
            run  = member['run']
            adid = member['id']
            full = ad_lookup.get((run, adid), {})
            sections += f"  [{run}]\n"
            sections += f"    label          : {full.get('label', member.get('label', '?'))}\n"
            sections += f"    decision       : {full.get('decision', '').replace(chr(10), ' ')}\n"
            sections += f"    rationale      : {full.get('rationale', '').replace(chr(10), ' ')}\n"
            sections += f"    confidence     : {full.get('confidence', 'strongly_inferred')}\n"
            sections += f"    safety_critical: {full.get('safety_critical', False)}\n"
            comps = full.get('affected_components', [])
            if comps:
                sections += f"    components     : {', '.join(comps)}\n"

    return f"""\
Below are {len(clusters)} clusters of semantically equivalent Architectural
Decisions, each containing variants from different analysis runs of the same codebase.
{sections}
For each cluster, synthesize ONE canonical AD that:
- Has a clear, concise label naming the design decision
- Has a complete decision field combining the best wording from all variants
- Has a rationale explaining why this choice was made and what alternatives were foregone
- Sets confidence to the highest level seen (explicit > strongly_inferred > weakly_inferred)
- Sets safety_critical = true if ANY variant marks it true
- Sets affected_components to the UNION of all variants' lists

Do NOT include evidence — that will be merged separately from the source runs.

Return ONLY this JSON (one entry per cluster, in cluster order):
{{
  "canonical_ads": [
    {{
      "cluster_index": <1-based integer matching the cluster number above>,
      "label": "<concise decision name>",
      "decision": "<full prose description of the design decision>",
      "rationale": "<why this decision was made>",
      "confidence": "explicit|strongly_inferred|weakly_inferred",
      "safety_critical": true|false,
      "affected_components": ["<component>"]
    }}
  ]
}}
"""


def cluster_and_synthesize_ads(
        client: anthropic.Anthropic,
        run_results: list[dict],
        ag_id_map: dict[tuple[str, str], str],
        output_dir: Path,
        prefix: str,
) -> tuple[list[dict], list[dict]]:
    """
    Cluster ADs semantically across N runs via Claude, synthesize canonical ADs,
    remap edges to canonical AD and AG IDs.

    Returns (canonical_ads, new_edges).
    """
    n_runs = len(run_results)

    cluster_path = output_dir / f'{prefix}-vote-ad-clusters.json'
    if cluster_path.exists():
        print(f"  Loading cached AD clusters: {cluster_path.name}")
        cluster_data = json.loads(cluster_path.read_text())
    else:
        run_ads = {
            f"Run{i + 1}": [n for n in r.get('nodes', []) if n.get('kind') == 'ad']
            for i, r in enumerate(run_results)
        }
        total_ads = sum(len(v) for v in run_ads.values())
        print(f"  Clustering {total_ads} ADs across {n_runs} runs...")
        t0  = time.time()
        raw = call_claude(client, build_ad_cluster_prompt(run_ads),
                          max_tokens=AD_CLUSTER_TOKENS, system=AD_CLUSTER_SYSTEM,
                          temperature=0.0)
        print(f"  AD clustering done in {time.time() - t0:.1f}s")
        cluster_data = parse_json_response(raw, 'vote-ad-clusters', output_dir)
        cluster_path.write_text(json.dumps(cluster_data, indent=2))
        print(f"  Saved: {cluster_path.name}  ({len(cluster_data['clusters'])} clusters)")

    clusters = cluster_data['clusters']

    synthesis_path = output_dir / f'{prefix}-vote-ad-synthesis.json'
    if synthesis_path.exists():
        print(f"  Loading cached AD synthesis: {synthesis_path.name}")
        synthesis_data = json.loads(synthesis_path.read_text())
    else:
        print(f"  Synthesizing {len(clusters)} canonical ADs...")
        t0  = time.time()
        raw = call_claude(client,
                          build_ad_synthesis_prompt(clusters, run_results),
                          max_tokens=AD_SYNTHESIS_TOKENS, system=AD_SYNTHESIS_SYSTEM,
                          temperature=0.0)
        print(f"  AD synthesis done in {time.time() - t0:.1f}s")
        synthesis_data = parse_json_response(raw, 'vote-ad-synthesis', output_dir)
        synthesis_path.write_text(json.dumps(synthesis_data, indent=2))

    raw_ads = synthesis_data.get('canonical_ads', [])
    if len(raw_ads) != len(clusters):
        print(f"  WARNING: AD synthesis returned {len(raw_ads)} ADs, "
              f"expected {len(clusters)}", file=sys.stderr)

    run_ad_lookup: dict[str, dict[str, dict]] = {
        f"Run{i + 1}": {n['id']: n for n in r.get('nodes', []) if n.get('kind') == 'ad'}
        for i, r in enumerate(run_results)
    }
    cluster_by_idx = {i + 1: c for i, c in enumerate(clusters)}

    canonical_ads: list[dict]             = []
    ad_id_map: dict[tuple[str, str], str] = {}

    for i, ad_data in enumerate(raw_ads, 1):
        cluster = cluster_by_idx.get(ad_data.get('cluster_index', 0), {})
        members = cluster.get('members', [])
        new_id  = f"AD-{i:02d}"

        evidence_parts: list[str] = []
        for member in members:
            ev = run_ad_lookup.get(member['run'], {}).get(member['id'], {}).get('evidence', '').strip()
            if ev and ev not in evidence_parts:
                evidence_parts.append(ev)

        canonical_ads.append({
            'id':                  new_id,
            'kind':                'ad',
            'label':               ad_data.get('label', ''),
            'decision':            ad_data.get('decision', ''),
            'evidence':            '; '.join(evidence_parts),
            'rationale':           ad_data.get('rationale', ''),
            'confidence':          ad_data.get('confidence', 'strongly_inferred'),
            'safety_critical':     ad_data.get('safety_critical', False),
            'affected_components': ad_data.get('affected_components', []),
        })
        for member in members:
            ad_id_map[(member['run'], member['id'])] = new_id

    seen_edges: set[tuple] = set()
    new_edges:  list[dict] = []
    for i, result in enumerate(run_results):
        run_label = f"Run{i + 1}"
        for edge in result.get('edges', []):
            new_src = ad_id_map.get((run_label, edge['source']))
            new_tgt = ag_id_map.get((run_label, edge['target']))
            if new_src is None or new_tgt is None:
                continue
            key = (new_src, new_tgt, edge['link_type'])
            if key not in seen_edges:
                seen_edges.add(key)
                new_edges.append({'source': new_src, 'target': new_tgt,
                                   'link_type': edge['link_type']})

    return canonical_ads, new_edges


# ── AD connection strength ─────────────────────────────────────────────────────

STRENGTH_SYSTEM = """\
You are a software architecture expert evaluating trace links between Architectural
Decisions (ADs) and Architectural Goals (AGs) in a Twin Peaks traceability graph.
Return ONLY valid JSON. No markdown fences, no explanation, no preamble.
"""


def _build_strength_prompt(edges: list[dict], ad_lookup: dict, ag_lookup: dict) -> str:
    items = ""
    for edge in edges:
        ad        = ad_lookup.get(edge['source'], {})
        ag        = ag_lookup.get(edge['target'], {})
        decision  = ad.get('decision', '').replace('\n', ' ')[:250]
        statement = ag.get('statement', '').replace('\n', ' ')[:200]
        items += (
            f"\n  source: {edge['source']}  target: {edge['target']}\n"
            f"    AD label  : {ad.get('label', '?')}\n"
            f"    AD decision (excerpt): {decision}\n"
            f"    AG label  : {ag.get('label', '?')}\n"
            f"    AG statement (excerpt): {statement}\n"
        )

    return f"""\
Below are {len(edges)} AD→AG trace links from a Twin Peaks traceability graph
recovered by reverse engineering an existing, deployed architecture.

For each link, classify the relationship and provide a one-sentence rationale.
{items}
Link type definitions (this is a reverse-engineered system — all ADs were accepted):
  makes  — the AD is the primary mechanism implementing or enacting this AG;
            an architect reviewing the code would immediately identify it as the main
            mechanism for the goal
  helps  — the AD contributes positively to this AG but is not the primary mechanism;
            the contribution is real but partial or indirect
  harms  — the AD negatively affects this AG (an accepted trade-off in the existing
            architecture; the system shipped with this tension)
  none   — the link is spurious; the AD has no meaningful relationship to this AG

Return ONLY this JSON ({len(edges)} assessments — one per link, in the same order):
{{
  "assessments": [
    {{
      "source": "<AD id>",
      "target": "<AG id>",
      "link_type": "makes|helps|harms|none",
      "connection_rationale": "<one sentence explaining the relationship>"
    }}
  ]
}}
"""


def verify_suspicious_ags(
        client: anthropic.Anthropic,
        suspicious_ags: list[dict],
        code_content: str,
        output_dir: Path,
        prefix: str,
) -> list[dict]:
    """Challenge weakly-inferred AGs by asking Claude to verify against the source code.

    Annotates each AG with _verification: {verdict, rationale}.
    Writes {prefix}-verification.json. Returns the annotated list.
    Does NOT remove AGs — callers can filter on _verification.verdict if desired.
    """
    verify_path = output_dir / f"{prefix}-verification.json"
    if verify_path.exists():
        saved = json.loads(verify_path.read_text())
        by_id = {v["id"]: v for v in saved.get("verifications", [])}
        for ag in suspicious_ags:
            if ag["id"] in by_id:
                ag["_verification"] = by_id[ag["id"]]
        print(f"  Loaded cached verification for {len(by_id)} AG(s)")
        return suspicious_ags

    ag_summary = [
        {"id": ag["id"], "label": ag.get("label", ""),
         "statement": ag.get("statement", ""), "evidence": ag.get("evidence", "")}
        for ag in suspicious_ags
    ]
    code_excerpt = code_content[:100_000] if len(code_content) > 100_000 else code_content

    prompt = (
        "You are verifying whether the following weakly-inferred Architectural Goals\n"
        "have genuine evidence in the source code provided.\n\n"
        "For each AG:\n"
        "  confirmed   — evidence field cites real files/functions present in the code\n"
        "                and the quality concern is genuinely present\n"
        "  unconfirmed — evidence is absent, vague, or the concern appears to be\n"
        "                an artefact of the extraction process\n"
        "  uncertain   — partial evidence exists; human review warranted\n\n"
        f"AGs to verify:\n{json.dumps(ag_summary, indent=2)}\n\n"
        f"SOURCE CODE (excerpt):\n{code_excerpt}\n\n"
        'Return ONLY this JSON:\n'
        '{"verifications": ['
        '{"id": "AG-XX", "verdict": "confirmed|unconfirmed|uncertain", '
        '"rationale": "<one sentence citing specific files or explaining absence>"}'
        ']}'
    )

    print(f"  Verifying {len(suspicious_ags)} weakly-inferred AG(s)...")
    t0  = time.time()
    raw = call_claude(
        client, prompt, max_tokens=4096,
        system="You are a software architect verifying architectural claims against source code. "
               "Return ONLY valid JSON. No markdown fences, no explanation.",
        temperature=0.0,
    )
    print(f"  Verification done in {time.time() - t0:.1f}s")
    result = parse_json_response(raw, "verification", output_dir)
    verify_path.write_text(json.dumps(result, indent=2))

    by_id = {v["id"]: v for v in result.get("verifications", [])}
    confirmed = unconfirmed = uncertain = 0
    for ag in suspicious_ags:
        v = by_id.get(ag["id"], {"verdict": "uncertain", "rationale": "no response"})
        ag["_verification"] = v
        if v["verdict"] == "confirmed":     confirmed   += 1
        elif v["verdict"] == "unconfirmed": unconfirmed += 1
        else:                               uncertain   += 1

    print(f"  Verification: {confirmed} confirmed, {unconfirmed} unconfirmed, "
          f"{uncertain} uncertain")
    if unconfirmed:
        for ag in suspicious_ags:
            if ag.get("_verification", {}).get("verdict") == "unconfirmed":
                print(f"    [unconfirmed] {ag['id']}: {ag.get('label','?')}")
    return suspicious_ags


def check_over_abstraction(
        client: anthropic.Anthropic,
        result: dict,
        code_content: str,
        output_dir: Path,
        prefix: str,
) -> dict:
    """Flag AGs whose statements are too generic to be system-specific.

    Checks AGs that lack explicit file evidence and aren't already at the
    'explicit' confidence level. Annotates them with _abstraction_check in-place.
    Writes {prefix}-abstraction-check.json. Returns the updated result dict.
    """
    check_path = output_dir / f"{prefix}-abstraction-check.json"
    nodes = result.get("nodes", [])
    ags   = [n for n in nodes if n.get("kind") == "ag"]

    # Only check AGs where evidence is thin and confidence is non-explicit
    vague = [
        ag for ag in ags
        if ag.get("confidence", "") != "explicit"
        and not any(
            kw in ag.get("evidence", "").lower()
            for kw in (".py", ".cpp", ".java", ".go", ".ts", ".js", ".yaml", ".yml",
                       ".sh", "def ", "class ", "function", "config")
        )
    ]

    if not vague:
        print("  Over-abstraction check: no vague AG candidates.")
        return result

    if check_path.exists():
        saved = json.loads(check_path.read_text())
        by_id = {c["id"]: c for c in saved.get("checks", [])}
        for ag in nodes:
            if ag.get("kind") == "ag" and ag["id"] in by_id:
                ag["_abstraction_check"] = by_id[ag["id"]]
        print(f"  Loaded cached abstraction check for {len(by_id)} AG(s)")
        return result

    ag_summary = [
        {"id": ag["id"], "label": ag.get("label", ""),
         "statement": ag.get("statement", "")}
        for ag in vague
    ]
    code_excerpt = code_content[:80_000] if len(code_content) > 80_000 else code_content

    prompt = (
        "You are checking whether each Architectural Goal is specific to this\n"
        "codebase or so generic it would apply equally to any software system.\n\n"
        "For each AG, decide:\n"
        "  appropriate — specific to this system's domain, constraints, or concerns\n"
        "  too_vague   — could be copy-pasted into any system's architecture doc\n"
        "                (e.g. 'The system must be maintainable')\n\n"
        "When too_vague, provide a suggested_statement that grounds the concern\n"
        "in the system's actual context — still technology-independent, but\n"
        "referencing what makes this system's version of the concern distinctive.\n\n"
        f"AGs to check:\n{json.dumps(ag_summary, indent=2)}\n\n"
        f"Relevant source code (excerpt):\n{code_excerpt}\n\n"
        'Return ONLY this JSON:\n'
        '{"checks": ['
        '{"id": "AG-XX", "verdict": "appropriate|too_vague", '
        '"suggested_statement": "<grounded statement or null>"}'
        ']}'
    )

    print(f"  Checking {len(vague)} AG(s) for over-abstraction...")
    t0  = time.time()
    raw = call_claude(
        client, prompt, max_tokens=4096,
        system="You are a software architect reviewing architectural goals for specificity. "
               "Return ONLY valid JSON. No markdown fences, no explanation.",
        temperature=0.0,
    )
    print(f"  Over-abstraction check done in {time.time() - t0:.1f}s")
    check_result = parse_json_response(raw, "abstraction-check", output_dir)
    check_path.write_text(json.dumps(check_result, indent=2))

    by_id     = {c["id"]: c for c in check_result.get("checks", [])}
    too_vague = sum(1 for c in check_result.get("checks", []) if c.get("verdict") == "too_vague")
    print(f"  Over-abstraction check: {too_vague}/{len(vague)} flagged as too_vague")
    for ag in nodes:
        if ag.get("kind") == "ag" and ag["id"] in by_id:
            ag["_abstraction_check"] = by_id[ag["id"]]
            if by_id[ag["id"]].get("verdict") == "too_vague":
                suggested = by_id[ag["id"]].get("suggested_statement", "")
                print(f"    [too_vague] {ag['id']}: {ag.get('label','?')}")
                if suggested:
                    ag["statement"] = suggested   # auto-apply the grounded statement
    return result


def assess_ad_connections(result: dict, client: anthropic.Anthropic,
                           output_dir: Path, temperature: float = 1.0) -> dict:
    """
    Classify each AD→AG edge as makes/helps/harms (or prune as spurious),
    add a connection_rationale field, and guarantee every AG retains at least
    one incoming edge.
    """
    nodes = result['nodes']
    edges = result['edges']

    ad_lookup = {n['id']: n for n in nodes if n.get('kind') == 'ad'}
    ag_lookup = {n['id']: n for n in nodes if n.get('kind') == 'ag'}

    if not edges:
        print("\nAssessing AD connections: no edges found, skipping.")
        return result

    assess_path = output_dir / 'ad-connections.json'
    assessment_data: dict | None = None
    if assess_path.exists():
        assessment_data = json.loads(assess_path.read_text())
        print(f"\nAssessing {len(edges)} AD→AG connection(s): loaded from checkpoint")
    else:
        print(f"\nAssessing {len(edges)} AD→AG connection(s)...")
        for attempt in range(2):
            t0  = time.time()
            raw = call_claude(client, _build_strength_prompt(edges, ad_lookup, ag_lookup),
                              max_tokens=32_000, system=STRENGTH_SYSTEM, temperature=0.0)
            print(f"  assessment done in {time.time() - t0:.1f}s")
            assessment_data = _try_parse_json(raw, 'ad-connections', output_dir)
            if assessment_data is not None:
                assess_path.write_text(json.dumps(assessment_data, indent=2))
                break
            if attempt == 0:
                print("  ad-connections parse failed — retrying once...", file=sys.stderr)
        if assessment_data is None:
            print("  ad-connections assessment skipped after 2 failed attempts — all edges kept as helps.",
                  file=sys.stderr)
            assessment_data = {'assessments': []}

    assessments = assessment_data.get('assessments', [])

    rating: dict[tuple[str, str], dict] = {
        (a['source'], a['target']): a for a in assessments
        if 'source' in a and 'target' in a
    }

    VALID_TYPES = {'makes', 'helps', 'harms'}

    # Annotate all edges with reassessed link_type + rationale
    annotated: list[dict] = []
    for edge in edges:
        key    = (edge['source'], edge['target'])
        assess = rating.get(key, {})
        assessed_type = assess.get('link_type', 'helps')
        # Fall back to original link_type if assessment is missing or invalid
        if assessed_type not in VALID_TYPES and assessed_type != 'none':
            assessed_type = edge.get('link_type', 'helps')
        annotated.append({
            **edge,
            'link_type':            assessed_type,
            'connection_rationale': assess.get('connection_rationale', ''),
        })

    # Decide which edges to keep: all non-spurious, plus safety-net for fully-spurious AGs
    edges_by_ag: dict[str, list[dict]] = defaultdict(list)
    for edge in annotated:
        edges_by_ag[edge['target']].append(edge)

    keep: set[tuple[str, str]] = set()
    for ag_id, ag_edges in edges_by_ag.items():
        real = [e for e in ag_edges if e['link_type'] in VALID_TYPES]
        if real:
            for e in real:
                keep.add((e['source'], e['target']))
        else:
            # All edges were rated spurious — keep the best-confidence one as 'helps'
            best = max(
                ag_edges,
                key=lambda e: -CONFIDENCE_RANK.get(
                    ad_lookup.get(e['source'], {}).get('confidence', 'weakly_inferred'), 2
                ),
            )
            best['link_type'] = 'helps'
            keep.add((best['source'], best['target']))
            print(f"  Safety net: retaining {best['source']}→{ag_id} as 'helps' (all edges were spurious)")

    kept_edges   = [e for e in annotated
                    if e['link_type'] in VALID_TYPES or (e['source'], e['target']) in keep]
    pruned_count = len(annotated) - len(kept_edges)

    makes_count = sum(1 for e in kept_edges if e['link_type'] == 'makes')
    helps_count = sum(1 for e in kept_edges if e['link_type'] == 'helps')
    harms_count = sum(1 for e in kept_edges if e['link_type'] == 'harms')
    print(f"  Edges: {makes_count} makes, {helps_count} helps, {harms_count} harms, "
          f"{pruned_count} pruned (spurious)")

    result['edges'] = kept_edges
    return result


def _build_nfr_kb_text(kb: dict) -> str:
    out = ""
    for tag, defn in kb.items():
        out += f"\n{tag.upper()}\n"
        out += f"  Core concern: {defn['core_concern']}\n"
        out += "  Classify if:\n"
        for rule in defn["classify_if"]:
            out += f"    - {rule}\n"
        out += f"  Boundary notes: {defn['boundary_notes']}\n"
    return out


def _nfr_prompt(kb_text: str, ag_list: str) -> str:
    return (
        "Assign ISO 25010 NFR taxonomy tags to each Architectural Goal below.\n\n"
        "RULES:\n"
        "- Assign exactly ONE tag. This is the default and covers the vast majority of cases.\n"
        "- A second tag is permitted ONLY when the AG's statement contains two clearly distinct "
        "quality concerns that each map to a different NFR category AND both are central to the "
        "AG — not merely implied, adjacent, or a consequence of the primary concern. "
        "If there is any doubt, assign one tag.\n"
        "- Use the boundary notes to resolve ambiguous cases.\n\n"
        f"NFR TAXONOMY\n============\n{kb_text}\n"
        f"AGs TO CLASSIFY\n===============\n{ag_list}\n\n"
        "Return ONLY this JSON (one entry per AG, in the same order):\n"
        '{"tags": [{"id": "AG-XX", "nfr_tags": ["<primary>"], '
        '"nfr_tag_rationales": ["<one sentence>"]}]}\n'
        "For the rare dual-category AG: "
        '{"id": "AG-XX", "nfr_tags": ["<primary>", "<secondary>"], '
        '"nfr_tag_rationales": ["<reason for primary>", "<reason for secondary>"]}'
    )


def _apply_nfr_entries(ags: list[dict], by_id: dict) -> int:
    """Write nfr_tags / nfr_tag_rationales onto AG nodes from a by-id lookup."""
    tagged = 0
    for ag in ags:
        entry = by_id.get(ag["id"])
        if entry:
            # Accept both new multi-tag format and legacy single-tag checkpoint
            tags      = entry.get("nfr_tags") or [entry.get("nfr_tag", "unclassified")]
            rationales = entry.get("nfr_tag_rationales") or \
                         ([entry.get("rationale", "")] * len(tags))
            ag["nfr_tags"]           = tags
            ag["nfr_tag_rationales"] = rationales
            tagged += 1
        else:
            ag["nfr_tags"]           = ["unclassified"]
            ag["nfr_tag_rationales"] = [""]
    return tagged


def classify_nfrs(result: dict, client: anthropic.Anthropic,
                   kb_path: Path, output_dir: Path, prefix: str) -> dict:
    """Assign nfr_tags (list) to each AG node using the NFR taxonomy in kb_path.

    Supports multi-classification: an AG that genuinely spans two NFR categories
    receives both. Writes {prefix}-nfr-tags.json as a checkpoint. Stale single-tag
    checkpoints (written by an earlier version) are detected and regenerated.
    """
    if not kb_path.exists():
        print(f"  NFR classification skipped — KB not found: {kb_path}", file=sys.stderr)
        return result

    ags = [n for n in result.get("nodes", []) if n.get("kind") == "ag"]
    if not ags:
        return result

    tag_path = output_dir / f"{prefix}-nfr-tags.json"
    if tag_path.exists():
        saved = json.loads(tag_path.read_text())
        entries = saved.get("tags", [])
        # Detect stale single-tag format (has "nfr_tag" key, not "nfr_tags")
        if entries and "nfr_tag" in entries[0] and "nfr_tags" not in entries[0]:
            print(f"  NFR checkpoint is single-tag format — regenerating with multi-tag...")
            tag_path.unlink()
        else:
            by_id  = {t["id"]: t for t in entries}
            tagged = _apply_nfr_entries(ags, by_id)
            print(f"  NFR tags: loaded {tagged} from checkpoint")
            return result

    kb      = json.loads(kb_path.read_text())
    kb_text = _build_nfr_kb_text(kb)
    ag_list = "\n".join(
        f"  {ag['id']}: {ag.get('label','')} — {ag.get('statement','')[:200]}"
        for ag in ags
    )

    system_prompt = ("You are classifying architectural goals against an NFR taxonomy. "
                     "Return ONLY valid JSON. No markdown fences, no explanation.")
    print(f"  Classifying {len(ags)} AGs against NFR taxonomy (multi-tag enabled)...")
    data = None
    for attempt in range(2):
        t0  = time.time()
        raw = call_claude(client, _nfr_prompt(kb_text, ag_list), max_tokens=16384,
                          system=system_prompt, temperature=0.0)
        print(f"  NFR classification done in {time.time()-t0:.1f}s")
        data = _try_parse_json(raw, "nfr-tags", output_dir)
        if data is not None:
            break
        if attempt == 0:
            print(f"  NFR parse failed — retrying once...", file=sys.stderr)
    if data is None:
        print(f"  NFR classification skipped after 2 failed attempts.", file=sys.stderr)
        return result
    tag_path.write_text(json.dumps(data, indent=2))

    by_id  = {t["id"]: t for t in data.get("tags", [])}
    _apply_nfr_entries(ags, by_id)
    for ag in ags:
        tags_str = ", ".join(ag.get("nfr_tags", ["?"]))
        print(f"    {ag['id']}: {ag.get('label','?')}  → {tags_str}")

    return result
