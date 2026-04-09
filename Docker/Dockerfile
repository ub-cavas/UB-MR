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
   ros-humble-tf2-ros \
   ros-humble-vision-msgs \
   python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash","-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
 && echo "source /ub_mr_workspace/install/setup.bash" >> /root/.bashrc \
 && echo "[ -f /app/cyclone_dds_config.bash ] && source /app/cyclone_dds_config.bash" >> /root/.bashrc

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /ub_mr_workspace
COPY submodules/mr_pkg /ub_mr_workspace/src/mr_pkg
RUN source /opt/ros/humble/setup.bash \
 && colcon build --symlink-install

# Unity Linux player runtime dependencies for X11 + Vulkan
RUN apt-get update && apt-get install -y --no-install-recommends \
    libvulkan1 vulkan-tools \
    libgl1 libglx0 libegl1 \
    libglu1-mesa \
    libxss1 \
    libx11-6 libxext6 libxrandr2 libxi6 libxcursor1 libxinerama1 \
    libxcomposite1 \
    libxkbcommon0 \
    libnss3 \
    libasound2 \
    ca-certificates \
    mesa-utils \
 && rm -rf /var/lib/apt/lists/*

ENV __GLX_VENDOR_LIBRARY_NAME=nvidia \
   __NV_PRIME_RENDER_OFFLOAD=1 \
   LIBGL_ALWAYS_INDIRECT=0

ENV UB_MR_PLAYER_DIR=/app/UB-MR-Player

WORKDIR /app
COPY Docker/Scripts/ub-mr.sh /app/ub-mr.sh
COPY Docker/Scripts/cyclone_dds_config.bash /app/cyclone_dds_config.bash
RUN chmod +x /app/ub-mr.sh /app/cyclone_dds_config.bash

# Prepare mount points for the Unity player and vehicle config files.
# The actual content is expected to be bind-mounted from the host at runtime.
RUN mkdir -p "$UB_MR_PLAYER_DIR" \
 && mkdir -p /root/.config/unity3d/UB-CAVAS/UB-MR

# TODO: Use Vulkan API
CMD ["bash", "/app/ub-mr.sh"]
