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
if [[ ! -f "${WS_ROOT}/install/setup.bash" ]]; then
    echo "Workspace is not built: ${WS_ROOT}/install/setup.bash" >&2
    exit 1
fi

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${WS_ROOT}/install/setup.bash"
set -u

exec python3 "${SCRIPT_DIR}/run_matrix.py" \
    --package-root "${PKG_ROOT}" \
    --workspace-root "${WS_ROOT}" \
    "$@"
