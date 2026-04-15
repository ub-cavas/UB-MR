#!/usr/bin/env bash
set -eo pipefail

# Run from /app (matches your WORKDIR)
cd /app

PLAYER_EXECUTABLE="${UB_MR_PLAYER_DIR}/UB-MR.x86_64"

if [[ ! -f "${PLAYER_EXECUTABLE}" ]]; then
  echo "Unity player executable not found at ${PLAYER_EXECUTABLE}" >&2
  exit 1
fi

# Ensure the Unity player is executable
chmod +x "${PLAYER_EXECUTABLE}"
mkdir -p /app/Logs

# Source the runtime environment before launching the player from the shell alias.
source /app/ub-mr-env.sh
source /app/cyclone_dds_config.bash
exec "${PLAYER_EXECUTABLE}" \
  -screen-fullscreen 0 \
  -screen-width 1920 \
  -screen-height 1080 \
  -logFile ./Logs/ub-mr-player.log \
  -force-glcore
