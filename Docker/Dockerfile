FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

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

# Copy the pre-built Unity player (built on the host) into the image.
# Assumes you run `docker build` from the project root:
#   docker build -f Docker/Dockerfile -t ub-mr .
COPY Builds/Linux ./Linux

RUN chmod +x ./Linux/UB-MR.x86_64

# TODO: Use Vulkan API
#CMD ["./Linux/UB-MR.x86_64", "-screen-fullscreen", "0", "-screen-width", "1280", "-screen-height", "720", "-force-vulkan", "-logFile", "/dev/stdout"]
CMD ["./Linux/UB-MR.x86_64", "-screen-fullscreen", "0", "-screen-width", "1920", "-screen-height", "1080"]
