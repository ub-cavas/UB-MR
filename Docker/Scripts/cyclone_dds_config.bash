#!/bin/bash

# ROS2 Environment Setup for Autoware Docker Communication
# Source this script to enable ROS2 topic visibility between host and container
#
# Usage: source /app/cyclone_dds_config.bash
#
# Keep this legacy entrypoint as a wrapper so every code path uses the same
# file-based configuration rather than an inline configuration that can drift.
source /app/Scripts/host_ros2_env.bash
