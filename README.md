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
- **Ubuntu 2022.04.5** 
- **Unity Editor:** 6000.0.36f1
- **ROS 2 Distribution:** Humble

## ROS2 Installation
Follow the official guide to install ROS 2 Humble (packaged version):
```bash
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
```

---
## Install Mixed Reality Packages

1. **ROS 2 MR Package** (~/ros2_ws/src):
   ```bash
   git clone https://github.com/ub-cavas/mr_pkg.git
   ```
   Whenever you update the ROS 2 package, rebuild it and source the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

2. **Vulkan Graphics API's** (skip if already installed)
   ```bash
     sudo apt update
     sudo apt install libvulkan1
     ```

2. **OPTION A: Packaged Simulator** (recommended):
   > *TODO: Provide binary download link and installation steps.*

3. **OPTION B: Simulator Source Code**:
     ```bash
     cd <directory-outside-of-your-ros2_ws>
     git clone https://github.com/ub-cavas/UB-MR.git
     ```
     *Add the project to UnityHub*

     1.) Open UnityHub

     2.) Click the "Add" Button from the "Project" tab

     3.) Locate the UB-MR directory that you cloned and add it to the UnityHub

     4.) Open the project with Unity Editor 6000.0.36f1

     5.) There is a bug that requires reimporting some files... Unity will crash the first time the project is loaded

     6.) Force Quit the editor and relaunch

     7.) In the editor: navigate to /Assets/UB_MR_Assets/RoadRunner/UB-Service-Center-Loop and reimport ServiceCenterLoopMR.fbx 
     (Right-Click -> "Reimport" )
     ![File Location](Docs/RR_Reimport_FBX.png)

     8.) Give it some time to import

     9.) The project should be ready now
---

## Running the Project

1. **Run the Mixed Reality ROS2 Package**

   Launch localization nodes in one terminal:

   ```bash
   ros2 launch mr_pkg localization.launch.py
   ```

2. **Run the Mixed Reality Simulator**

   OPTION A: Run the executable

   OPTION B: Press Play in the Unity Editor

## Playback a Ros Bag

1. Open a new terminal, source ROS2 Humble and your Workspace:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. Play your bag with an offset (e.g., start at 200s):
   ```bash
   ros2 bag play <path-to-rosbag> --clock --start-offset 200
   ```

---

## Troubleshooting & Tips

- **Build Errors:** Ensure all dependencies are installed and your environment is sourced.
- **ROS 2 Topics:** Use `ros2 topic list` and `ros2 topic echo <topic>` to verify data flow.
- **Unity Logs:** Check the Console window for errors when launching the scene.

*For further assistance, please open an issue in the respective GitHub repository.*

