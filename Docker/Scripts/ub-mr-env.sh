#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source /ub_mr_workspace/install/setup.bash

if [[ "${UB_MR_USE_LOCAL_MR_PKG:-0}" == "1" ]]; then
  LOCAL_SETUP="${UB_MR_LOCAL_MR_PKG_WS}/install/setup.bash"
  if [[ -f "${LOCAL_SETUP}" ]]; then
    source "${LOCAL_SETUP}"
  fi
fi
