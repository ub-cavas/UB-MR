#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${SCRIPT_DIR}"

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <build-folder-name-under-Builds> [additional docker args...]"
  echo "Example: $0 Ubuntu2204"
  exit 1
fi

BUILD_FOLDER="$1"
shift

HOST_BUILD_DIR="${REPO_ROOT}/Builds/${BUILD_FOLDER}"

if [[ ! -d "${HOST_BUILD_DIR}" ]]; then
  echo "Error: build directory '${HOST_BUILD_DIR}' does not exist."
  exit 1
fi

mkdir -p "${REPO_ROOT}/Docker/Logs"

IMAGE_NAME="${IMAGE_NAME:-ub-mr}"

docker run --rm -it --gpus all \
  --net=host \
  -e NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility \
  -e DISPLAY="${DISPLAY}" \
  -e UB_MR_PLAYER_DIR=/app/UB-MR-Player \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v "${REPO_ROOT}/Docker/Logs:/app/Logs" \
  -v "${REPO_ROOT}/Agents:/root/.config/unity3d/UB-CAVAS/UB-MR:ro" \
  -v "${HOST_BUILD_DIR}:/app/UB-MR-Player:ro" \
  "${IMAGE_NAME}" "$@"

