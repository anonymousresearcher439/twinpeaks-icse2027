#!/usr/bin/env bash
# Step 1 of RQ1: generate N independent single-run V0 models (the voter pool).
# Each run uses --runs 1 so analyze_architecture.py produces one raw model
# (no voting, no synthesis). Voting happens later in run_RQ1_experiment.py.
#
# Usage (from RQ1/):  bash generate_voters.sh <dataset>
#   Reads source, pool_dir and n_voters for <dataset> from rq1_config.json.
# Output: <pool_dir>/run1/V0.json ... <pool_dir>/runN/V0.json
# Runs that already have a V0.json are skipped, so the script can be resumed.

set -eo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"

DATASET="${1:?usage: bash generate_voters.sh <dataset>}"
PYTHON="${PYTHON:-python3}"
export TP_DATASETS="${TP_DATASETS:-$(cd "$HERE/../.." && pwd)/datasets}"
SCRIPT="../core/analyze_architecture.py"
KB="../KB-iso.json"

read -r SOURCE POOL N_VOTERS < <($PYTHON - "$DATASET" <<'PY'
import json, os, sys
cfg = json.load(open("rq1_config.json"))["datasets"].get(sys.argv[1])
if cfg is None:
    sys.exit(f"ERROR: '{sys.argv[1]}' not found in rq1_config.json")
src = os.path.expanduser(os.path.expandvars(cfg["source"]))
if "$" in src:
    sys.exit("ERROR: set TP_DATASETS to the folder containing the datasets")
print(src, cfg["pool_dir"], cfg["n_voters"])
PY
)

[ -d "$SOURCE" ] || { echo "ERROR: source not found: $SOURCE"; exit 1; }

for i in $(seq 1 "$N_VOTERS"); do
    OUT="$POOL/run${i}"
    if [ -f "$OUT/V0.json" ]; then
        echo "[$(date '+%H:%M:%S')] $OUT: already complete, skipping."
        continue
    fi
    mkdir -p "$OUT"
    echo ""
    echo "════════════════════════════════════════"
    echo "[$(date '+%H:%M:%S')] $DATASET voter ${i}/${N_VOTERS} (log: $OUT/run.log)"
    echo "════════════════════════════════════════"
    $PYTHON "$SCRIPT" "$SOURCE" --runs 1 --output "$OUT/V0.json" --kb "$KB" \
        2>&1 | tee "$OUT/run.log"
done

echo ""
echo "[$(date '+%H:%M:%S')] Voter pool complete: $POOL/run1..run${N_VOTERS}"
