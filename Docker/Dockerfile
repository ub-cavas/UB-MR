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
 && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash","-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

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

WORKDIR /app

# Copy the pre-built Unity player into the image.
COPY Builds/Linux ./Linux

# As configured in ProjectSettings/ProjectSettings.asset:
#   companyName: DefaultCompany
#   productName: UB-MR
RUN mkdir -p /root/.config/unity3d/DefaultCompany/UB-MR
COPY Agents/*.json /root/.config/unity3d/DefaultCompany/UB-MR/

RUN chmod +x ./Linux/UB-MR.x86_64

# TODO: Use Vulkan API
CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && ./Linux/UB-MR.x86_64 -screen-fullscreen 0 -screen-width 1920 -screen-height 1080"]
