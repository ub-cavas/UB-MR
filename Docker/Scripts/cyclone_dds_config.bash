#!/bin/bash

# ROS2 Environment Setup for Autoware Docker Communication
# Source this script to enable ROS2 topic visibility between host and container
#
# Usage: source ./scripts/cyclone_dds_config.bash

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Match container's DDS middleware (CycloneDDS)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Match container's CycloneDDS configuration (loopback interface binding)
export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface autodetermine="false" name="lo" priority="default" multicast="default" /></Interfaces><AllowMulticast>default</AllowMulticast><MaxMessageSize>65500B</MaxMessageSize></General></Domain></CycloneDDS>'

echo "ROS2 environment configured for Autoware Docker communication"
echo "  RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "  CYCLONEDDS_URI: (loopback interface binding)"

# --- Restart ROS 2 discovery daemon (helps after changing RMW/CycloneDDS settings) ---
if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop >/dev/null 2>&1 || true
  sleep 0.2
  ros2 daemon start >/dev/null 2>&1 || true
  echo "  ros2 daemon        : restarted"
else
  echo "[WARN] 'ros2' command not found in PATH (did ROS2 setup source correctly?)"
fi
