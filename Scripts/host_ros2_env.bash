#!/bin/bash

# ROS2 Environment Setup for Autoware Docker Communication
# Source this script to enable ROS2 topic visibility between host and container
#
# Usage: source UB-MR/Scripts/host_ros2_env.bash

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Match the container's DDS middleware (CycloneDDS).
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Use the same file configuration on the host and in the host-networked
# container.  The launcher supplies UB_MR_CYCLONEDDS_URI for its /etc mount;
# when sourced on the host, use this repository's tracked configuration.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST_CONFIG="${SCRIPT_DIR}/../Resources/cyclonedds.xml"
CONTAINER_CONFIG="/etc/cyclonedds.xml"

if [[ -n "${UB_MR_CYCLONEDDS_URI:-}" ]]; then
  export CYCLONEDDS_URI="${UB_MR_CYCLONEDDS_URI}"
elif [[ -f "${CONTAINER_CONFIG}" ]]; then
  export CYCLONEDDS_URI="file://${CONTAINER_CONFIG}"
elif [[ -f "${HOST_CONFIG}" ]]; then
  export CYCLONEDDS_URI="file://${HOST_CONFIG}"
else
  echo "[ERROR] UB-MR CycloneDDS configuration not found: ${HOST_CONFIG}" >&2
  return 1 2>/dev/null || exit 1
fi

echo "ROS2 environment configured for Autoware Docker communication"
echo "  RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "  CYCLONEDDS_URI: $CYCLONEDDS_URI"

# --- Restart ROS 2 discovery daemon (helps after changing RMW/CycloneDDS settings) ---
if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop >/dev/null 2>&1 || true
  sleep 0.2
  ros2 daemon start >/dev/null 2>&1 || true
  echo "  ros2 daemon        : restarted"
else
  echo "[WARN] 'ros2' command not found in PATH (did ROS2 setup source correctly?)"
fi
