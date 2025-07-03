#!/usr/bin/env bash
set -euo pipefail

# Configuration
ROS_DISTRO="${ROS_DISTRO:-humble}"
WORKSPACE_DIR="${WORKSPACE_DIR:-./Externals/ros2_for_unity_ws}"
REPOS_FILE_URL="https://raw.githubusercontent.com/RobotecAI/ros2-for-unity/develop/ros2_for_unity_custom_messages.repos"
ROS2CS_REPOS_URL="https://raw.githubusercontent.com/RobotecAI/ros2-for-unity/develop/ros2cs.repos"

echo "=== Ubuntu setup for ROS2 For Unity ($ROS_DISTRO) ==="

echo "-- Update and install ROS2 dependencies"
sudo apt update
sudo apt install -y \
  ros-"$ROS_DISTRO"-test-msgs \
  ros-"$ROS_DISTRO"-fastrtps \
  ros-"$ROS_DISTRO"-rmw-fastrtps-cpp \
  vcstool \
  dotnet-sdk-3.1 \
  libspdlog-dev \
  libtinyxml2-dev

echo "-- Prepare workspace directory at $WORKSPACE_DIR"
mkdir -p "$WORKSPACE_DIR/src"
cd "$WORKSPACE_DIR"

echo "-- Import repositories"
vcs import src < <(curl -sL "$REPOS_FILE_URL")
vcs import src < <(curl -sL "$ROS2CS_REPOS_URL")

echo "-- Build asset (this will also build ros2cs)"
bash src/ros2-for-unity/build.sh --overlay

echo "-- Install Unity asset into ~/ros2_for_unity_ws/install/asset"
echo "Now open or create a Unity project and copy:"
echo "  $WORKSPACE_DIR/install/asset/Ros2ForUnity → <YourProject>/Assets/"

echo "Setup complete! To use overlay mode, remember to:"
echo "  source /opt/ros/$ROS_DISTRO/setup.bash"
echo "before launching the Unity Editor."

echo "Enjoy ROS2‑enabled Unity 🚀"