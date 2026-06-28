#!/usr/bin/env bash
set -Eeuo pipefail

# ------------------------------------------------------------------------------
# Docker Deep Clean: stop & remove ALL containers, images, volumes, networks,
# and builder cache. Then optionally rebuild docker compose services.
#
# Usage:
#   deep_clean.sh [--force] [--dry-run] [--no-rebuild]
# ------------------------------------------------------------------------------

FORCE=false
DRY_RUN=false
DO_REBUILD=true

for arg in "$@"; do
  case "$arg" in
    --force) FORCE=true ;;
    --dry-run) DRY_RUN=true ;;
    --no-rebuild) DO_REBUILD=false ;;
    *) echo "Unknown option: $arg" >&2; exit 2 ;;
  esac
done

log() { printf "\n==> %s\n" "$*"; }

# Execute a command with proper arg splitting, no eval.
run() {
  if $DRY_RUN; then
    printf "[dry-run]"
    printf " %q" "$@"
    printf "\n"
  else
    "$@"
  fi
}

# Ensure Docker is reachable
if ! docker info >/dev/null 2>&1; then
  echo "Docker daemon does not appear to be running or accessible." >&2
  exit 1
fi

cat <<'WARN'
You are about to perform a DEEP CLEAN of Docker resources on this machine:

- Stop and remove ALL containers
- Remove ALL images
- Remove ALL volumes
- Prune ALL unused networks (keeps default: bridge, host, none)
- Prune ALL builder cache
- Final 'docker system prune -a --volumes' sweep

This is destructive and cannot be undone.
WARN

if ! $FORCE; then
  read -r -p "Type 'yes' to proceed: " RESP
  if [[ "${RESP:-}" != "yes" ]]; then
    echo "Aborted."
    exit 0
  fi
fi

# Stop and remove all containers
CONTAINERS="$(docker ps -aq || true)"
if [[ -n "$CONTAINERS" ]]; then
  log "Stopping all containers..."
  # If any stop fails, keep going
  run docker stop $CONTAINERS || true

  log "Removing all containers..."
  run docker rm -f $CONTAINERS || true
else
  log "No containers found."
fi

# Remove ALL images
IMAGES="$(docker images -aq || true)"
if [[ -n "$IMAGES" ]]; then
  log "Removing all images..."
  run docker rmi -f $IMAGES || true
else
  log "No images found."
fi

# Remove ALL volumes
VOLUMES="$(docker volume ls -q || true)"
if [[ -n "$VOLUMES" ]]; then
  log "Removing all volumes..."
  run docker volume rm $VOLUMES || true
else
  log "No volumes found."
fi

# Prune non-default networks
log "Pruning unused networks (default networks are preserved)..."
run docker network prune -f

# Prune builder cache
log "Pruning builder cache..."
run docker builder prune -af

# Belt-and-suspenders final sweep
log "Final sweep: docker system prune -af --volumes..."
run docker system prune -af --volumes

# Optional rebuild
if $DO_REBUILD; then
  if docker compose version >/dev/null 2>&1; then
    COMPOSE_CMD="docker compose"
  elif command -v docker-compose >/dev/null 2>&1; then
    COMPOSE_CMD="docker-compose"
  else
    COMPOSE_CMD=""
  fi

  if [[ -n "${COMPOSE_CMD}" ]]; then
    if [[ -f "docker-compose.yml" || -f "compose.yml" || -f "docker-compose.yaml" || -f "compose.yaml" ]]; then
      log "Rebuilding services with $COMPOSE_CMD --no-cache..."
      run $COMPOSE_CMD build --no-cache
    else
      log "No compose file found in current directory. Skipping rebuild."
    fi
  else
    log "Neither 'docker compose' nor 'docker-compose' found. Skipping rebuild."
  fi
else
  log "Rebuild disabled (--no-rebuild)."
fi

log "Deep clean completed."