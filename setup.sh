# Clone ros2-for-unity repo to build custom Message Types
git clone git@github.com:RobotecAI/ros2-for-unity.git ./Externals/ros2-for-unity

source /opt/ros/humble/setup.bash
cd Externals/ros2-for-unity

# Pull new 
./pull_repositories.sh