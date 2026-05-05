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
   python3-pyproj \
 && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash","-c"]
RUN echo "source /app/ub-mr-env.sh" >> /root/.bashrc \
 && echo "[[ -f \"${UB_MR_ROS_ENV_SCRIPT:-/app/Scripts/host_ros2_env.bash}\" ]] && source \"${UB_MR_ROS_ENV_SCRIPT:-/app/Scripts/host_ros2_env.bash}\"" >> /root/.bashrc \
 && echo "alias ub-mr-player='/app/ub-mr.sh'" >> /root/.bashrc

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV UB_MR_ROS_ENV_SCRIPT=/app/Scripts/host_ros2_env.bash

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
ENV UB_MR_LOCAL_MR_PKG_WS=/ub_mr_local_mr_pkg_ws
ENV UB_MR_USE_LOCAL_MR_PKG=0

WORKDIR /app
COPY Scripts/host_ros2_env.bash /app/Scripts/host_ros2_env.bash
COPY Docker/Scripts/ub-mr-entrypoint.sh /app/ub-mr-entrypoint.sh
COPY Docker/Scripts/ub-mr.sh /app/ub-mr.sh
COPY Docker/Scripts/ub-mr-env.sh /app/ub-mr-env.sh
COPY Docker/Scripts/ub-mr-shell.sh /app/ub-mr-shell.sh
COPY Docker/Scripts/cyclone_dds_config.bash /app/cyclone_dds_config.bash
RUN chmod +x /app/Scripts/host_ros2_env.bash /app/ub-mr-entrypoint.sh /app/ub-mr.sh /app/ub-mr-env.sh /app/ub-mr-shell.sh /app/cyclone_dds_config.bash

# Prepare mount points for the Unity player and vehicle config files.
# The actual content is expected to be bind-mounted from the host at runtime.
RUN mkdir -p "$UB_MR_PLAYER_DIR" \
 && mkdir -p "$UB_MR_LOCAL_MR_PKG_WS/src" \
 && mkdir -p /root/.config/unity3d/UB-CAVAS/UB-MR

# TODO: Use Vulkan API
ENTRYPOINT ["/app/ub-mr-entrypoint.sh"]
CMD ["/app/ub-mr-shell.sh"]
