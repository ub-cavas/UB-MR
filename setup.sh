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

echo "=== Ubuntu setup for ROS2 For Unity ($ROS_DISTRO) ==="
echo "-- Update and install ROS2 dependencies"
sudo apt update
sudo apt install -y \
  python3-pip \
  python3-vcstool \
  patchelf \
  dotnet-sdk-7.0 \
  ros-"$ROS_DISTRO"-test-msgs \
  ros-"$ROS_DISTRO"-fastrtps \
  ros-"$ROS_DISTRO"-rmw-fastrtps-cpp 
sudo apt update

echo "-- Importing repositories"
chmod +x pull_repositories.sh
bash pull_repositories.sh

echo "-- Building Unity asset"
bash build.sh --overlay --clean-install
if [ -d "Assets/Ros2ForUnity" ]; then
    rm -rf Assets/Ros2ForUnity
fi
cp -r Externals/ros2_for_unity_ws/install/asset/Ros2ForUnity Assets

echo "Setup complete! To use overlay mode, remember to:"
echo "  source /opt/ros/$ROS_DISTRO/setup.bash"
echo "before launching the Unity Editor."