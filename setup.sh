#!/bin/bash
set -e

WORKSPACE_DIR="${WORKSPACE_DIR:-Externals/ros2_for_unity_ws}"
UNITY_DIR="${WORKSPACE_DIR:-Assets}"

# Install ros2-for-unity repo
if [ -d "$WORKSPACE_DIR" ]; then
    echo "Found ros2-for-unity workspace"
else
    git clone https://github.com/oakley-Thomas/ros2-for-unity.git  "$WORKSPACE_DIR"
fi
cd "$WORKSPACE_DIR"
git checkout virtual-object-detection

# source ros2
source /opt/ros/humble/setup.bash

ROS_DISTRO="${ROS_DISTRO:-humble}"
REPOS_FILE_URL="https://raw.githubusercontent.com/RobotecAI/ros2-for-unity/develop/ros2_for_unity_custom_messages.repos"
ROS2CS_REPOS_URL="https://raw.githubusercontent.com/RobotecAI/ros2-for-unity/develop/ros2cs.repos"

echo "=== Ubuntu setup for ROS2 For Unity ($ROS_DISTRO) ==="

echo "-- Update and install ROS2 dependencies"
sudo apt update
sudo apt install -y python3-pip
sudo apt install python3-vcstool
sudo apt install dotnet-sdk-7.0
sudo apt install patchelf
sudo apt install -y \
  ros-"$ROS_DISTRO"-test-msgs \
  ros-"$ROS_DISTRO"-fastrtps \
  ros-"$ROS_DISTRO"-rmw-fastrtps-cpp 
sudo apt update
echo "=== DONE ($ROS_DISTRO) ==="

echo "-- Preparing workspace directory at $WORKSPACE_DIR"

echo "-- Importing repositories"
chmod +x pull_repositories.sh
bash pull_repositories.sh

echo "-- Building Unity asset"
bash build.sh --overlay 
echo "Copy over the Ros2ForUnity folder from Externals/ros2_for_untiy_ws/install/asset"
echo "Setup complete! To use overlay mode, remember to:"
echo "  source /opt/ros/$ROS_DISTRO/setup.bash"
echo "before launching the Unity Editor."