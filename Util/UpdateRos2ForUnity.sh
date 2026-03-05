git config --global --add safe.directory '*'

read -r -p "Remove existing Ros2ForUnity install (../Assets/Ros2ForUnity) and Ros2ForUnity.meta? [y/N] " confirm
case "$confirm" in
  [yY]|[yY][eE][sS])
    rm -rf ../Assets/Ros2ForUnity
    rm -f ../Assets/Ros2ForUnity.meta
    ;;
  *)
    echo "Aborted."
    exit 1
    ;;
esac

# Update to get most recent versions of custom message types
cd ../submodules/ros2-for-unity
./pull_repositories.sh
echo "Cleaning build directory to remove stale CMake caches..."
rm -rf ./build

read -r -p "Build standalone Ros2ForUnity [y/N] " standalone_confirm
case "$standalone_confirm" in
  [yY]|[yY][eE][sS])
    ./build.sh --clean-install --standalone
    ;;
  *)
    ./build.sh --clean-install
    ;;
esac
