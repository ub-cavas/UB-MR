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

---

## Quick-Start Guide
1. Install the Prerequisites
   0) Latest NVIDIA Graphics Drivers - tested on RTX 4070-580.95.05 but other versions might work depending on your hardware
   1) Install Docker [[Link]](https://docs.docker.com/engine/install/ubuntu/)
   2) Setup docker user to not require sudo when running docker [[Link]](https://docs.docker.com/engine/install/linux-postinstall/)
   3) Install NVIDIA container toolkit [[Link]](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#with-apt-ubuntu-debian) and register with docker [[Link]](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#configuration)

2. Pull the Docker Image from DockerHub:
   ```bash
   TODO: Link Upcoming!
   ```
   or build it locally:
   ```bash
   cd Docker
   docker build -f Docker/Dockerfile -t ub-mr .
   ```
   The docker image hosts the Unity Player and required ROS nodes for Mixed Reality
   

---

## Developer Guide
### Suggested Configuration
- **Ubuntu 2022.04.5** 
- **Unity Editor - 6000.0.36f1**
- **ROS 2 Distribution:** Humble

Install Vulkan Graphics APIs (skip if already installed)
   ```bash
   sudo apt update
   sudo apt install libvulkan1
   ```

### Unity Player Installation (the packaged version)
1. Download the latest release [[Link]] (https://github.com/ub-cavas/UB-MR)

### Unity Project Installation (with Unity Editor)

**IMPORTANT: There is a RoadRunner bug that requires reimporting some files... Unity will crash the first time the project is loaded, if this occurs force quit the editor and relaunch**
1. Install the UnityHub [[Link]](https://docs.unity3d.com/hub/manual/InstallHub.html#install-hub-linux/)
2. Install Unity Editor - 6000.0.36f1
3. Add the project (/UB-MR) to UnityHub
4. Open the project.. you may need to force quit and relaunch on first load

#### Fix import errors - Only needed if the RoadRunner environments do not load correctly
1. Navigate to Assets/UB_MR_Assets/RoadRunner/
2. In each subfolder, find the .fbx -> Right-Click -> "Reimport"
3. This takes some time...

![File Location](Docs/RR_Reimport_FBX.png)


### ROS2 Interface
1. Install ROS2 Humble [[Link]](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
2. Clone the mr_pkg ROS2 workspace:
   ```bash
   git clone https://github.com/ub-cavas/mr_pkg.git
   ```
3. Remember: when you update the ROS 2 package, rebuild it and source the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```





## Running the Project

### Docker 
```bash
xhost +local:root # x11 graphics forwarding - only needed ONCE per shell session

docker run --rm -it --gpus all   --net=host   -e NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility   -e DISPLAY=$DISPLAY   -v /tmp/.X11-unix:/tmp/.X11-unix:ro   -v "$(pwd)/Docker/Logs:/app/Logs"   ub-mr
```

### Unity Player (Native)
```bash
chmod +x UB-MR.x86_64 # give execution permissions

./UB-MR.x86_64
```
   
### Unity Editor (Native)
1. Open Assets/Modules/MainMenu.unity 
2. Press Play in editor

## Playback a Ros Bag

1. Open a new terminal, source ROS2 Humble
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

