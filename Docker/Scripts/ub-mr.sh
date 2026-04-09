#!/usr/bin/env bash
set -eo pipefail

# Run from /app (matches your WORKDIR)
cd /app

# Ensure the Unity player is executable
chmod +x "${UB_MR_PLAYER_DIR}/UB-MR.x86_64"

# Start Unity in the foreground (container lifetime == Unity lifetime)
source /app/cyclone_dds_config.bash
source /ub_mr_workspace/install/setup.bash
exec "${UB_MR_PLAYER_DIR}/UB-MR.x86_64" \
  -screen-fullscreen 0 \
  -screen-width 1920 \
  -screen-height 1080 \
  -logFile ./Logs/ub-mr-player.log \
  -force-glcore
