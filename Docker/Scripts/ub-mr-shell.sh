#!/usr/bin/env bash
set -eo pipefail

if [[ "${UB_MR_USE_LOCAL_MR_PKG:-0}" == "1" ]]; then
  LOCAL_WS="${UB_MR_LOCAL_MR_PKG_WS}"
  LOCAL_SRC_DIR="${LOCAL_WS}/src/mr_pkg"

  if [[ ! -d "${LOCAL_SRC_DIR}" ]]; then
    echo "Local mr_pkg source not found at ${LOCAL_SRC_DIR}" >&2
    exit 1
  fi

  source /app/ub-mr-env.sh
  cd "${LOCAL_WS}"
  colcon build --symlink-install --packages-select mr_pkg
  source /app/ub-mr-env.sh
fi

if [[ $# -gt 0 ]]; then
  exec "$@"
fi

exec /bin/bash -i
