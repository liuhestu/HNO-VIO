#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WS_ROOT="$(cd "${PKG_ROOT}/../.." && pwd)"
EVAL_ROOT="${PKG_ROOT}/eval"
REPORT_ROOT="${EVAL_ROOT}/reports"

ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
if [[ -f "${ROS_SETUP}" ]]; then
    # shellcheck disable=SC1090
    source "${ROS_SETUP}"
fi

WORKSPACE_HOME="$(dirname "${WS_ROOT}")"
for conda_bin in "${HOME}/miniconda3/envs/hno_vio/bin" "${WORKSPACE_HOME}/miniconda3/envs/hno_vio/bin"; do
    if [[ -d "${conda_bin}" ]]; then
        PATH="${PATH}:${conda_bin}"
    fi
done
for local_bin in "${HOME}/.local/bin" "${WORKSPACE_HOME}/.local/bin"; do
    if [[ -d "${local_bin}" ]]; then
        PATH="${PATH}:${local_bin}"
    fi
done
export PATH

if [[ -f "${WS_ROOT}/devel/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${WS_ROOT}/devel/setup.bash"
fi

mkdir -p "${EVAL_ROOT}" "${REPORT_ROOT}"

dataset_bag_path() {
    local dataset="$1"
    local bag_root="${HNO_VIO_BAG_ROOT:-/home/sharpa/datasets/euroc/ROSbag}"
    printf '%s/%s.bag\n' "${bag_root}" "${dataset}"
}

dataset_gt_path() {
    local dataset="$1"
    printf '%s/ground_truth/euroc_mav/%s.txt\n' "${PKG_ROOT}" "${dataset}"
}

timestamp_utc() {
    date -u +"%Y%m%dT%H%M%SZ"
}

require_command() {
    local cmd="$1"
    if ! command -v "${cmd}" >/dev/null 2>&1; then
        echo "Missing required command: ${cmd}" >&2
        return 1
    fi
}

build_workspace() {
    (cd "${WS_ROOT}" && catkin_make)
}
