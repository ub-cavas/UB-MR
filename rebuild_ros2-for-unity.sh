#!/bin/bash
set -e

bash build.sh --overlay --clean-install
if [ -d "Assets/Ros2ForUnity" ]; then
    rm -rf Assets/Ros2ForUnity
fi

cp -r Externals/ros2_for_unity_ws/install/asset/Ros2ForUnity Assets