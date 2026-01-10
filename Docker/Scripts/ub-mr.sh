#!/usr/bin/env bash
set -eo pipefail

# Pull and rebuild mr_pkg from origin/main
cd /ub_mr_workspace/src/mr_pkg
git fetch
git pull
cd /ub_mr_workspace
colcon build

# Run from /app (matches your WORKDIR)
cd /app

# Ensure the Unity player is executable
chmod +x "${UB_MR_PLAYER_DIR}/UB-MR.x86_64"

# Start Unity in the foreground (container lifetime == Unity lifetime)

exec "${UB_MR_PLAYER_DIR}/UB-MR.x86_64" \
  -screen-fullscreen 0 \
  -screen-width 1920 \
  -screen-height 1080 \
  -logFile ./Logs/ub-mr-player.log \
  -force-glcore
