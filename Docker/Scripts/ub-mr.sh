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

SCREEN_FULLSCREEN="${UB_MR_SCREEN_FULLSCREEN:-0}"
SCREEN_WIDTH="${UB_MR_SCREEN_WIDTH:-1920}"
SCREEN_HEIGHT="${UB_MR_SCREEN_HEIGHT:-1080}"

# Configure ROS 2/DDS before Unity starts.
ROS_ENV_SCRIPT="${UB_MR_ROS_ENV_SCRIPT:-/app/Scripts/host_ros2_env.bash}"
if [[ -f "${ROS_ENV_SCRIPT}" ]]; then
  source "${ROS_ENV_SCRIPT}"
else
  echo "[WARN] ROS environment script not found at '${ROS_ENV_SCRIPT}'. Falling back to ROS Humble setup."
  source /opt/ros/humble/setup.bash
fi

# Start Unity in the foreground (container lifetime == Unity lifetime)
exec "${UB_MR_PLAYER_DIR}/UB-MR.x86_64" \
  -screen-fullscreen "${SCREEN_FULLSCREEN}" \
  -screen-width "${SCREEN_WIDTH}" \
  -screen-height "${SCREEN_HEIGHT}" \
  -logFile ./Logs/ub-mr-player.log \
  -force-glcore
