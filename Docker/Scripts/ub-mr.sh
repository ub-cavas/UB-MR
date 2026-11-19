#!/usr/bin/env bash
set -eo pipefail

# Run from /app (matches your WORKDIR)
cd /app

PLAYER_EXECUTABLE="${UB_MR_PLAYER_DIR}/UB-MR.x86_64"

if [[ ! -f "${PLAYER_EXECUTABLE}" ]]; then
  echo "Unity player executable not found at ${PLAYER_EXECUTABLE}" >&2
  exit 1
fi

# Ensure the Unity player is executable
chmod +x "${PLAYER_EXECUTABLE}"
mkdir -p /app/Logs
UB_MR_METRICS_ENABLED="${UB_EXPERIMENT_ENABLED:-${UB_MR_METRICS_ENABLED:-1}}"

SCREEN_FULLSCREEN="${UB_MR_SCREEN_FULLSCREEN:-0}"
SCREEN_WIDTH="${UB_MR_SCREEN_WIDTH:-1920}"
SCREEN_HEIGHT="${UB_MR_SCREEN_HEIGHT:-1080}"

if [[ "${UB_MR_METRICS_ENABLED:-1}" != "0" && "${UB_EXPERIMENT_PLOTS:-1}" != "0" ]]; then
  if ! /usr/bin/python3 -c 'import matplotlib' >/dev/null 2>&1; then
    echo "[ERROR] Experiment graphs require python3-matplotlib in the Docker image." >&2
    echo "Exit this container shell, then run on the host from UB-MR/:" >&2
    echo "  docker build -f Docker/Dockerfile -t ub-mr ." >&2
    echo "  ./run_ub_mr.sh <build-folder>" >&2
    echo "Exit the old container before relaunching; rebuilding an image does not update a running container." >&2
    exit 1
  fi
fi

# Configure ROS 2/DDS before Unity starts.
ROS_ENV_SCRIPT="${UB_MR_ROS_ENV_SCRIPT:-/app/Scripts/host_ros2_env.bash}"
if [[ -f "${ROS_ENV_SCRIPT}" ]]; then
  source "${ROS_ENV_SCRIPT}"
else
  echo "[WARN] ROS environment script not found at '${ROS_ENV_SCRIPT}'. Falling back to ROS Humble setup."
  source /opt/ros/humble/setup.bash
fi

if [[ "${UB_MR_METRICS_ENABLED:-1}" != "0" ]]; then
  METRICS_PARENT="${UB_EXPERIMENT_DIR:-${UB_MR_METRICS_DIR:-/app/Logs/experiments}}"
  mkdir -p "${METRICS_PARENT}"
  METRICS_HOST_NAME="${UB_EXPERIMENT_HOST_NAME:-${UB_MR_HOST_NAME:-$(hostname)}}"
  METRICS_HOST_NAME="$(printf '%s' "${METRICS_HOST_NAME}" | LC_ALL=C tr -c 'A-Za-z0-9._-' '_')"
  METRICS_HOST_NAME="${METRICS_HOST_NAME:0:63}"
  export UB_MR_METRICS_RUN_DIR
  UB_MR_METRICS_RUN_DIR="$(mktemp -d "${METRICS_PARENT}/run-$(date -u +%Y%m%d-%H%M%S)Z-${METRICS_HOST_NAME}-XXXXXX")"
  export UB_EXPERIMENT_RUN_DIR="${UB_MR_METRICS_RUN_DIR}"
  # mktemp defaults to 0700; allow host review even when Unity runs as root.
  chmod 0755 "${UB_MR_METRICS_RUN_DIR}"
  if [[ -n "${UB_MR_HOST_UID:-}" && -n "${UB_MR_HOST_GID:-}" ]]; then
    chown "${UB_MR_HOST_UID}:${UB_MR_HOST_GID}" "${UB_MR_METRICS_RUN_DIR}"
  fi
  echo "Experiment output: ${UB_MR_METRICS_RUN_DIR} (host Logs mount)"
fi

# Forward shutdown to Unity, then render from the flushed CSV before returning.
PLAYER_PID=""
forward_stop() { [[ -z "${PLAYER_PID}" ]] || kill -TERM "${PLAYER_PID}" 2>/dev/null || true; }
trap forward_stop TERM INT
"${UB_MR_PLAYER_DIR}/UB-MR.x86_64" \
  -screen-fullscreen "${SCREEN_FULLSCREEN}" \
  -screen-width "${SCREEN_WIDTH}" \
  -screen-height "${SCREEN_HEIGHT}" \
  -logFile ./Logs/ub-mr-player.log \
  -force-glcore &
PLAYER_PID=$!
set +e
wait "${PLAYER_PID}"
PLAYER_STATUS=$?
# A signal can interrupt wait before the player has finished shutting down.
if kill -0 "${PLAYER_PID}" 2>/dev/null; then
  wait "${PLAYER_PID}"
  PLAYER_STATUS=$?
fi
PLAYER_PID=""
if [[ "${UB_MR_METRICS_ENABLED:-1}" != "0" ]]; then
  if [[ "${UB_EXPERIMENT_PLOTS:-1}" != "0" ]]; then
    /usr/bin/python3 /app/Scripts/plot_experiment_metrics.py "${UB_MR_METRICS_RUN_DIR}" || \
      echo "[WARN] Graph generation failed. CSV files remain in ${UB_MR_METRICS_RUN_DIR}; rerun plot_experiment_metrics.py on the host."
  fi
  if [[ -n "${UB_MR_HOST_UID:-}" && -n "${UB_MR_HOST_GID:-}" ]]; then
    chown -R "${UB_MR_HOST_UID}:${UB_MR_HOST_GID}" "${UB_MR_METRICS_RUN_DIR}" || \
      echo "[WARN] Could not assign experiment file ownership to the host user."
  fi
fi
exit "${PLAYER_STATUS}"
