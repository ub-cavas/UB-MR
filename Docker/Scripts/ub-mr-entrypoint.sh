#!/usr/bin/env bash
set -eo pipefail

source /app/ub-mr-env.sh

ROS_ENV_SCRIPT="${UB_MR_ROS_ENV_SCRIPT:-/app/Scripts/host_ros2_env.bash}"
if [[ -f "${ROS_ENV_SCRIPT}" ]]; then
  source "${ROS_ENV_SCRIPT}"
else
  echo "[WARN] ROS environment script not found at '${ROS_ENV_SCRIPT}'." >&2
fi

exec "$@"
