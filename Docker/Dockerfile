FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# ROS2 Humble
RUN apt-get update && apt-get install -y locales curl gnupg2 lsb-release \
 && locale-gen en_US en_US.UTF-8 \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list \
 && apt-get update && apt-get install -y \
   ros-humble-ros-base \
   ros-humble-robot-localization \
   ros-humble-rmw-cyclonedds-cpp \
   git \
   python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash","-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /ub_mr_workspace
RUN mkdir -p src \
 && cd src \
 && git clone https://github.com/ub-cavas/mr_pkg.git \
 && cd /ub_mr_workspace \
 && source /opt/ros/humble/setup.bash \
 && colcon build --symlink-install

# Unity Linux player runtime dependencies for X11 + Vulkan
RUN apt-get update && apt-get install -y --no-install-recommends \
    libvulkan1 \
    libgl1 \
    libglu1-mesa \
    libxcursor1 \
    libxrandr2 \
    libxinerama1 \
    libxi6 \
    libxss1 \
    libx11-6 \
    libxext6 \
    libxcomposite1 \
    libxkbcommon0 \
    libnss3 \
    libasound2 \
    ca-certificates \
 && rm -rf /var/lib/apt/lists/*

ENV UB_MR_PLAYER_DIR=/app/UB-MR-Player

WORKDIR /app

# Prepare mount points for the Unity player and vehicle config files.
# The actual content is expected to be bind-mounted from the host at runtime.
RUN mkdir -p "$UB_MR_PLAYER_DIR" \
 && mkdir -p /root/.config/unity3d/UB-CAVAS/UB-MR

# TODO: Use Vulkan API
CMD ["bash", "-lc", "mkdir -p ./Logs && chmod +x \"${UB_MR_PLAYER_DIR}/UB-MR.x86_64\" && (source /opt/ros/humble/setup.bash && source /ub_mr_workspace/install/setup.bash && ros2 launch mr_pkg localization.launch.py) & source /opt/ros/humble/setup.bash && \"${UB_MR_PLAYER_DIR}/UB-MR.x86_64\" -screen-fullscreen 0 -screen-width 1920 -screen-height 1080 -logFile ./Logs/ub-mr-player.log"]
