#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

exec "$@"
