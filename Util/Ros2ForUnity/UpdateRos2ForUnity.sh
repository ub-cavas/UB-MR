git config --global --add safe.directory '*'

# Update to get most recent versions of custom message types
cd ../../submodules/ros2-for-unity
./pull_repositories.sh

# Patch robot_localization to only generate its service interfaces.
# The full package builds C++ nodes (EKF, UKF, navsat) requiring geographiclib,
# eigen, tf2_eigen, yaml_cpp_vendor, etc. — none of which are needed for C# bindings.
cat > src/ros2cs/custom_messages/robot_localization/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.5)
project(robot_localization)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FromLL.srv"
  "srv/GetState.srv"
  "srv/SetDatum.srv"
  "srv/SetPose.srv"
  "srv/ToggleFilterProcessing.srv"
  "srv/ToLL.srv"
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    geographic_msgs
    diagnostic_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
EOF

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

echo "Build complete. Assets will be copied by the host."
