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
   docker pull oakleyth/ub-mr:latest
   ```
**Option B).** Build the image locally:
   ```bash
   git submodule update --init --recursive
   docker build -f Docker/Dockerfile -t ub-mr .
   ```

### Run a Mixed Reality session
```bash
# Start the ub-mr-container and drop into a shell
./run_ub_mr.sh <NAME-OF-BUILD-FOLDER>
# Then launch the Unity player inside the container
./ub-mr.sh
# Optional: override the initial player window size
UB_MR_SCREEN_WIDTH=1600 UB_MR_SCREEN_HEIGHT=900 ./run_ub_mr.sh <NAME-OF-BUILD-FOLDER>
# Start the container using the repo's local submodule copy of mr_pkg
./run_ub_mr.sh use-local-mr-pkg <NAME-OF-BUILD-FOLDER>
# (Optional - in another terminal) Start a localization stack
docker exec -it ub-mr-container /bin/bash
ros2 launch mr_pkg <dual_ekf_localization.launch.py> <autoware_localization.launch.py> <carla_localization.py>
```
---

## Developer Guide
If you intend to develop the Mixed Reality Engine, follow these steps...

### Configuration
- **Ubuntu 2022.04.5** (Required)
- **Unity Editor - 6000.0.36f1** [[Link]](https://unity.com/releases/editor/archive#:~:text=See%20all-,6000.0.36f1,-Security%20Alert)
- **ROS 2 Distribution:** Humble [[Link]](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- **RMW**: Eclipse Cyclone DDS [[Link]](https://docs.ros.org/en/humble/Installation/RMW-Implementations/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html)

1. Install Vulkan Graphics APIs (skip if already installed)
   ```bash
   sudo apt update
   sudo apt install libvulkan1
   ```
2. Clone this repo and its submodules, then initialize git lfs for the assets

   ```bash
   git clone --recurse-submodules git@github.com:ub-cavas/mr_pkg.git
   cd UB-MR
   git lfs install
   ```

3. Unity Project Installation (with Unity Editor)

   **IMPORTANT: There is a RoadRunner bug that requires reimporting some files... Unity may crash the first time the project is loaded, if this occurs force quit the editor and relaunch**

   A. Install the UnityHub [[Link]](https://docs.unity3d.com/hub/manual/InstallHub.html#install-hub-linux/)

   B. Install Unity Editor - 6000.0.36f1

   C. Add the project (/UB-MR) to UnityHub

   D. Download Ros2ForUnity
   ```bash
   ./Util/Ros2ForUnity/DownloadRos2ForUnity.sh ros2forunity-v1.0.2          # latest release
   ```
   E. Open the project.. you may need to force quit and relaunch on first load

4. Copy agents to Unity Editor's expected path (OPTIONAL)


   ```bash
   ./Util/Development/copy_agents.sh
   ```

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
If you want the checked-in agent JSON files copied into Unity's persistent-data directory before launch:

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
- **Unity player window:** New player builds start windowed and resizable. The Docker launcher defaults to `1920x1080`; set `UB_MR_SCREEN_WIDTH`, `UB_MR_SCREEN_HEIGHT`, or `UB_MR_SCREEN_FULLSCREEN=1` before `./run_ub_mr.sh` to override the initial mode.
- **Interactive container shell:** Launch the player manually with `./ub-mr.sh`. If the host copy ever loses its executable bit, `bash /app/ub-mr.sh` is a safe fallback.
- **Docker GPU startup fails:** If `./run_ub_mr.sh` fails with a Docker/NVIDIA GPU error such as `no known GPU vendor found` or `exec: "nvidia-container-runtime": executable file not found`, verify the host first:
  ```bash
  nvidia-smi
  docker run --rm --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=all nvidia/cuda:12.5.0-base-ubuntu22.04 nvidia-smi
  ```
  If either command fails, install/configure the NVIDIA Container Toolkit and register it with Docker:
  ```bash
  sudo nvidia-ctk runtime configure --runtime=docker
  sudo systemctl restart docker
  ```

*For further assistance, please open an issue in the respective GitHub repository.*
