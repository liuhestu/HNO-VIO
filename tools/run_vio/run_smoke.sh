#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WS_ROOT="$(cd "${PKG_ROOT}/../.." && pwd)"

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "ROS2 setup not found: ${ROS_SETUP}" >&2
    exit 1
fi

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u

cd "${WS_ROOT}"
PATH=/usr/bin:${PATH} PYTHON_EXECUTABLE=/usr/bin/python3 \
    colcon build --packages-select hno_vio --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
