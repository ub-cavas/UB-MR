# Ros2ForUnity Builder Image (Ubuntu 22.04 + ROS 2 Humble)

This image is intended for building Ros2ForUnity (including `--standalone`) from this repository by running `Util/UpdateRos2ForUnity.sh`.

It includes the `ros2cs` Ubuntu prerequisites from:
- `submodules/ros2-for-unity/src/ros2cs/README-UBUNTU.md`

Installed requirements:
- ROS 2 Humble (`ros-humble-ros-base`)
- `test-msgs`, `fastrtps`, `rmw-fastrtps-cpp`, `cyclonedds`, `rmw-cyclonedds-cpp`
- `python3-vcstool`
- `.NET SDK 6.0`
- `patchelf` (required for `--standalone`)

## Build the image

```bash
docker build -f Docker/Ros2ForUnity/Dockerfile.ros2forunity-builder -t ubmr-ros2forunity:humble .
```

## Run the container

```bash
docker run --rm -it \
  -v "$PWD":/workspace/UB-MR \
  -w /workspace/UB-MR/Util \
  ubmr-ros2forunity:humble bash
```

Inside the container:

```bash
./UpdateRos2ForUnity.sh
```

The entrypoint automatically sources `/opt/ros/humble/setup.bash` and sets `ROS_DISTRO=humble`.
