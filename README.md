# Mixed Reality Autonomous Vehicle Digital Twin

## Quick-Start User Guide
If you only intend to run Mixed Reality scenarios, use this method... 
### Prerequisites
0) NVIDIA Graphics Drivers 
1) Docker [[Link]](https://docs.docker.com/engine/install/ubuntu/)
2) Set up user to not require sudo when running Docker [[Link]](https://docs.docker.com/engine/install/linux-postinstall/)
3) Install NVIDIA container toolkit [[Link]](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#with-apt-ubuntu-debian) and register with Docker [[Link]](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#configuration)

### Download a Release
```bash
# Download latest release
./download_unity_player.sh

# Download a specific version
./download_unity_player.sh <NAME-OF-BUILD-FOLDER>
```

### Setup the Runtime Environment
In order to maximize compatibility and minimize setup time, we use Docker with GPU passthrough. The image hosts the Unity player and the required ROS nodes for Mixed Reality

**Option A).** Pull from Dockerhub (Recommended)
   ```bash
   TODO: Link Upcoming!
   ```
**Option B).** Build the image locally:
   ```bash
   docker build -f Docker/Dockerfile -t ub-mr .
   ```
---

## Developer Guide
If you intend to develop the Mixed Reality Engine, follow these steps...
### Configuration
- **Ubuntu 2022.04.5** (Required)
- **Unity Editor - 6000.0.36f1**
- **ROS 2 Distribution:** Humble

Install Vulkan Graphics APIs (skip if already installed)
   ```bash
   sudo apt update
   sudo apt install libvulkan1
   ```


### Unity Project Installation (with Unity Editor)

**IMPORTANT: There is a RoadRunner bug that requires reimporting some files... Unity will crash the first time the project is loaded, if this occurs force quit the editor and relaunch**
1. Install the UnityHub [[Link]](https://docs.unity3d.com/hub/manual/InstallHub.html#install-hub-linux/)
2. Install Unity Editor - 6000.0.36f1
3. Add the project (/UB-MR) to UnityHub
4. Download Ros2ForUnity
   ```bash
   ./Util/Ros2ForUnity/DownloadRos2ForUnity.sh             # latest release
   ```
5. Open the project.. you may need to force quit and relaunch on first load

#### Fix import errors - Only needed if the RoadRunner environments do not load correctly
1. Navigate to Assets/UB_MR_Assets/RoadRunner/
2. In each subfolder, find the .fbx -> Right-Click -> "Reimport"
3. This takes some time...

![File Location](Docs/RR_Reimport_FBX.png)

## Running the Project

### Docker 
```bash
./run_ub_mr.sh 0.0.1
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

