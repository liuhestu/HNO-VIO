#!/usr/bin/env bash
#
# Usage:
#   scripts/02_build_ros2_ws.sh
#
# Inputs:
#   Environment prepared by scripts/run_rtabmap.sh and common_env.sh:
#   ROS2_WS, RTABMAP_WS, LOG_DIR.
#
# Outputs:
#   Builds the local ROS 2 replay workspace under ${ROS2_WS} and writes:
#   ${LOG_DIR}/02_build_ros2_ws.log
#
# Notes:
#   This script is skipped when scripts/run_rtabmap.sh is called with
#   --skip-build.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_env.sh"

source_ros2_base
cd "${ROS2_WS}"
colcon build --symlink-install 2>&1 | tee "${LOG_DIR}/02_build_ros2_ws.log"
set +u
# shellcheck disable=SC1091
source "${ROS2_WS}/install/setup.bash"
set -u
ros2 pkg prefix hno_rtabmap_replay | tee -a "${LOG_DIR}/02_build_ros2_ws.log"
