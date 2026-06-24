#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_env.sh"

source_ros2_base
cd "${ROS2_WS}"
colcon build --symlink-install 2>&1 | tee "${OUT_DIR}/logs/02_build_ros2_ws.log"
set +u
# shellcheck disable=SC1091
source "${ROS2_WS}/install/setup.bash"
set -u
ros2 pkg prefix hno_rtabmap_replay | tee -a "${OUT_DIR}/logs/02_build_ros2_ws.log"
