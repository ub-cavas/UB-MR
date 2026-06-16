#!/usr/bin/env bash
set -euo pipefail

# Do nothing if the container is already running
CONTAINER_NAME="ub-mr-container"

if docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
  echo "Container '${CONTAINER_NAME}' is already running. Exiting!"
  exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${SCRIPT_DIR}"
USE_LOCAL_MR_PKG=0
BUILD_FOLDER=""
CONTAINER_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    use-local-mr-pkg|--use-local-mr-pkg)
      USE_LOCAL_MR_PKG=1
      shift
      ;;
    --)
      shift
      CONTAINER_ARGS+=("$@")
      break
      ;;
    *)
      if [[ -z "${BUILD_FOLDER}" ]]; then
        BUILD_FOLDER="$1"
      else
        CONTAINER_ARGS+=("$1")
      fi
      shift
      ;;
  esac
done

if [[ -z "${BUILD_FOLDER}" ]]; then
  echo "Usage: $0 [use-local-mr-pkg] <build-folder-name-under-Builds> [container command...]"
  echo "Example: $0 0.0.1"
  echo "Example: $0 use-local-mr-pkg 0.0.1"
  exit 1
fi

# Enable graphics forwarding to host machine
xhost +local:root

HOST_BUILD_DIR="${REPO_ROOT}/Builds/${BUILD_FOLDER}"

if [[ ! -d "${HOST_BUILD_DIR}" ]]; then
  echo "Error: build directory '${HOST_BUILD_DIR}' does not exist."
  exit 1
fi

DOCKER_ARGS=()

if [[ "${USE_LOCAL_MR_PKG}" == "1" ]]; then
  HOST_LOCAL_MR_PKG_DIR="${REPO_ROOT}/submodules/mr_pkg"

  if [[ ! -d "${HOST_LOCAL_MR_PKG_DIR}" ]]; then
    echo "Error: local mr_pkg directory '${HOST_LOCAL_MR_PKG_DIR}' does not exist."
    exit 1
  fi

  DOCKER_ARGS+=(
    -e UB_MR_USE_LOCAL_MR_PKG=1
    -v "${HOST_LOCAL_MR_PKG_DIR}:/ub_mr_local_mr_pkg_ws/src/mr_pkg"
  )
fi

IMAGE_NAME="${IMAGE_NAME:-ub-mr}"
CONTAINER_COMMAND=()
DOCKER_TTY_ARGS=()

if [[ -t 0 && -t 1 ]]; then
  DOCKER_TTY_ARGS=(-it)
fi

if [[ "${USE_LOCAL_MR_PKG}" == "1" ]]; then
  if [[ ${#CONTAINER_ARGS[@]} -gt 0 ]]; then
    CONTAINER_COMMAND=(/app/ub-mr-shell.sh "${CONTAINER_ARGS[@]}")
  fi
elif [[ ${#CONTAINER_ARGS[@]} -gt 0 ]]; then
  CONTAINER_COMMAND=("${CONTAINER_ARGS[@]}")
fi

docker run --rm "${DOCKER_TTY_ARGS[@]}" \
  --net=host \
  --name "${CONTAINER_NAME}" \
  --runtime=nvidia \
  --privileged=true \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY="${DISPLAY}" \
  -e UB_MR_PLAYER_DIR=/app/UB-MR-Player \
  -e UB_MR_ROS_ENV_SCRIPT=/app/Scripts/host_ros2_env.bash \
  -e UB_MR_SCREEN_FULLSCREEN="${UB_MR_SCREEN_FULLSCREEN:-0}" \
  -e UB_MR_SCREEN_WIDTH="${UB_MR_SCREEN_WIDTH:-1920}" \
  -e UB_MR_SCREEN_HEIGHT="${UB_MR_SCREEN_HEIGHT:-1080}" \
  -e CYCLONEDDS_URI=file:///etc/cyclonedds.xml \
  -v ${HOME}/cyclonedds.xml:/etc/cyclonedds.xml:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v "${REPO_ROOT}/Docker/Scripts/ub-mr.sh:/app/ub-mr.sh:ro" \
  -v "${REPO_ROOT}/Docker/Logs:/app/Logs" \
  -v "${REPO_ROOT}/Scripts:/app/Scripts:ro" \
  -v "${REPO_ROOT}/Agents:/root/.config/unity3d/UB-CAVAS/UB-MR:ro" \
  -v "${HOST_BUILD_DIR}:/app/UB-MR-Player" \
  "${DOCKER_ARGS[@]}" \
  "${IMAGE_NAME}" "${CONTAINER_COMMAND[@]}"
