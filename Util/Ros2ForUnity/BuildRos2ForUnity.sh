#!/usr/bin/env bash
# Host-side script: runs the build container then copies the output as the host user.
# Run from the repository root: ./Util/Ros2ForUnity/BuildRos2ForUnity.sh
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

read -r -p "Remove existing Ros2ForUnity install (Assets/Ros2ForUnity) and Ros2ForUnity.meta? [y/N] " confirm
case "$confirm" in
  [yY]|[yY][eE][sS])
    rm -rf "${REPO_ROOT}/Assets/Ros2ForUnity"
    rm -f "${REPO_ROOT}/Assets/Ros2ForUnity.meta"
    ;;
  *)
    echo "Aborted."
    exit 1
    ;;
esac

docker run --rm -it \
  -v "${REPO_ROOT}/Util/Ros2ForUnity":/workspace/UB-MR/Util/Ros2ForUnity \
  -v "${REPO_ROOT}/submodules/ros2-for-unity":/workspace/UB-MR/submodules/ros2-for-unity \
  -v "${REPO_ROOT}/.git/modules/submodules/ros2-for-unity":/workspace/UB-MR/.git/modules/submodules/ros2-for-unity \
  ubmr-ros2forunity:humble

echo "Copying Ros2ForUnity assets to Unity project..."
cp -r "${REPO_ROOT}/submodules/ros2-for-unity/install/asset/Ros2ForUnity" \
      "${REPO_ROOT}/Assets/Ros2ForUnity"
echo "Done."
