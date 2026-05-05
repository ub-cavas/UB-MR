#!/usr/bin/env bash
set -eo pipefail

# Run from /app (matches your WORKDIR)
cd /app

# Ensure the Unity player is executable
chmod +x "${UB_MR_PLAYER_DIR}/UB-MR.x86_64"

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
  -screen-fullscreen 0 \
  -screen-width 1920 \
  -screen-height 1080 \
  -logFile ./Logs/ub-mr-player.log \
  -force-glcore
