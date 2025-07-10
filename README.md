# Mixed Reality Autonomous Vehicle Digital Twin

## Table of Contents

- [Configuration](#configuration)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
  - [ROS 2 Humble on WSL2 Ubuntu 22.04.5 LTS](#ros-2-humble-on-wsl2-ubuntu-22045-lts)
  - [Clone Repositories](#clone-repositories)
- [Building the ROS 2 Package](#building-the-ros-2-package)
- [Sourcing Your Environment](#sourcing-your-environment)
- [Running the Mixed Reality Package](#running-the-mixed-reality-package)
  - [Option 1: Live Stream Mixed Reality](#option-1-live-stream-mixed-reality)
  - [Option 2: Replay a ROS Bag](#option-2-replay-a-ros-bag)
- [Running the Unity Engine](#running-the-unity-engine)
- [Troubleshooting & Tips](#troubleshooting--tips)

---

## Configuration
- **Ubuntu 2022.04.5** (WSL not officially supported, but may work)
- **Unity Editor:** 6000.0.36f1
- **ROS 2 Distribution:** Humble

## ROS2 Installation
Follow the official guide to install ROS 2 Humble (packaged version):
```bash
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
```

---
## Install Mixed Reality Packages

1. **ROS 2 Package** (~/ros2_ws/src):
   ```bash
   git clone https://github.com/ub-cavas/mr_pkg.git
   ```
   Whenever you update the ROS 2 package, rebuild it:
   ```bash
  cd ~/ros2_ws
  colcon build --symlink-install
  ```

2. **OPTION A: Packaged Simulator** (recommended):
   > *TODO: Provide binary download link and installation steps.*

3. **OPTION B: Simulator Source Code**:
     ```bash
     cd <directory-outside-of-your-ros2_ws>
     git clone https://github.com/ub-cavas/UB-MR.git
     ```
     This script will build Ros2ForUnity binaries and import them into the Unity project
      ```bash
      cd UB-MR
      ./setup.sh
      ```


---

## Sourcing Your Environment

In each new terminal session (WSL2 Ubuntu):

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

## Running the Mixed Reality Package

Launch all MR nodes in one terminal:

```bash
ros2 run mr_pkg world_transform
```

### Option 1: Live Stream Mixed Reality

> *TODO: Describe how to configure and launch live streaming of sensor data.*

### Option 2: Replay a ROS Bag

1. Open a new WSL2 Ubuntu terminal and source ROS 2:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. Play your bag with an offset (e.g., start at 200s):
   ```bash
   ros2 bag play <path-to-rosbag> --start-offset 200
   ```

---

## Running the Unity Engine

On Windows 11, either:

- Double-click `UB-MR.exe` in the build folder, or
- Open the `service_center_loop` scene in Unity Editor and press ▶️ Play.

---

## Troubleshooting & Tips

- **Build Errors:** Ensure all dependencies are installed and your environment is sourced.
- **ROS 2 Topics:** Use `ros2 topic list` and `ros2 topic echo <topic>` to verify data flow.
- **Unity Logs:** Check the Console window for errors when launching the scene.

*For further assistance, please open an issue in the respective GitHub repository.*

