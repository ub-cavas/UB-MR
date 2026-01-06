#!/usr/bin/env bash
set -euo pipefail

# Simple helper to get a shell in an already running container.
# If a container name/ID is provided, use that.
# Otherwise, attach to the first running container created from IMAGE_NAME (default: ub-mr).

IMAGE_NAME="${IMAGE_NAME:-ub-mr}"

if [[ $# -ge 1 ]]; then
  CONTAINER_ID="$1"
  shift
else
  CONTAINER_ID="$(docker ps --filter "ancestor=${IMAGE_NAME}" --format '{{.ID}}' | head -n 1 || true)"
fi

if [[ -z "${CONTAINER_ID}" ]]; then
  echo "No running container found."
  echo "Start it first (e.g., via run_ub_mr.sh) or pass a container name/ID:"
  echo "  $0 <container-name-or-id>"
  exit 1
fi

docker exec -it "${CONTAINER_ID}" /bin/bash "$@"
