# Ros2ForUnity Builder Image (Ubuntu 22.04 + ROS 2 Humble)

This image is intended for building Ros2ForUnity (including `--standalone`) from this repository by running `Util/Ros2ForUnity/UpdateRos2ForUnity.sh`.

It includes the `ros2cs` Ubuntu prerequisites from:
- `submodules/ros2-for-unity/src/ros2cs/README-UBUNTU.md`

Installed requirements:
- ROS 2 Humble (`ros-humble-ros-base`)
- `angles`, `test-msgs`, `fastrtps`, `rmw-fastrtps-cpp`, `cyclonedds`, `rmw-cyclonedds-cpp`
- `python3-vcstool`
- `.NET SDK 6.0`
- `patchelf` (required for `--standalone`)

## Build the image

```bash
docker build -f Util/Ros2ForUnity/Dockerfile.ros2forunity-builder -t ubmr-ros2forunity:humble .
```

## Getting Ros2ForUnity assets

`Assets/Ros2ForUnity/` is not tracked in git. Use one of the following options.

### Option A — Download a prebuilt release (recommended)

Requires the [GitHub CLI](https://cli.github.com/) (`gh`). Run from the repository root:

```bash
./Util/Ros2ForUnity/DownloadRos2ForUnity.sh             # latest release
./Util/Ros2ForUnity/DownloadRos2ForUnity.sh ros2forunity-v1.0.0  # specific tag
```

### Option B — Build locally

Run from the repository root:

```bash
./Util/Ros2ForUnity/BuildRos2ForUnity.sh
```

This starts the container (which runs `UpdateRos2ForUnity.sh` to pull and build), then copies the output to `Assets/Ros2ForUnity` as the host user so Unity has correct file ownership.

## Publishing a new release

Trigger the **Build Ros2ForUnity** GitHub Actions workflow (`workflow_dispatch`) with a tag name. The workflow builds the Docker image, runs the build container, and publishes `Ros2ForUnity.zip` as a GitHub Release asset.
