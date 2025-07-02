# CONFIGURATION
Unity 6000.0.36f1 on Windows 11
Ubuntu 22.04.5 LTS on WSL2
ROS2 Humble

# INSTALLATION
# Install ros2 humble onto WSL2 Ubuntu 22.04.5 LTS
https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html

# Clone this package into a ros2 workspace (WSL2 Ubuntu 22.05.5 LTS)
git clone https://github.com/ub-cavas/mr_pkg.git

# Clone the engine + source code (Windows 11)
1.) Install Unity HUB: https://unity.com/download
2.) Open Unity HUB
3.) Install Unity 6000.0.36f1 Editor from the Unity HUB
4.) Install the Mixed Reality Engine
link to repo: https://github.com/ub-cavas/UB-MR
git clone https://github.com/ub-cavas/UB-MR.git


# Clone the engine (binaries)
TODO

# ENABLE MIXED REALITY
# Build the ros2 package (only required when package is updated)
cd ros2_ws
colcon build

# Source ROS 2 and the workspace (WSL2 Ubuntu 22.05.5 LTS)
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run Mixed Reality Package (all nodes) in one terminal (WSL2 Ubuntu 22.05.5 LTS)
ros2 run mr_pkg world_transform

# Option 1: Live Stream Mixed Reality
TODO
# Option 2: Replay a rosbag
UB Testing Facility rosbag: TODO

open a new WSL2 Ubuntu 22.05.5 LTS
source /opt/ros/humble/setup.bash
ros2 bag play {path-to-rosbag} --start-offset 200

# Run Engine (Windows 11)
Run UB-MR.exe
or 
Play the service_center_loop scene from the Unity Editor
