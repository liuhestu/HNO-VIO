#!/usr/bin/env bash
#
# Usage:
#   source scripts/common_env.sh
#
# Inputs:
#   Required environment variables, usually exported by scripts/run_rtabmap.sh:
#   RUN_DIR, VIO_RESULTS_DIR, RUN_CONTEXT_JSON, INPUT_ODOM_CSV,
#   INPUT_ODOM_TXT, EUROC_MAV0, GROUND_TRUTH_TUM, ODOM_FRAME,
#   BASE_FRAME, OUT_DIR, INPUT_BAG, OUTPUT_BAG, RTABMAP_DB.
#
# Outputs:
#   Exports shared paths and ROS 2 environment defaults:
#   HNO_PKG, RUN_RTABMAP_DIR, ROS2_WS, RTABMAP_WS, ROS_LOG_DIR,
#   RMW_IMPLEMENTATION, LOG_DIR.
#   Also defines helper functions:
#   print_paths, source_ros2_base, source_ros2_local.
#
# Notes:
#   This file is shared configuration and should be sourced by other scripts.
#   It exits with status 2 if required variables are missing.
set -euo pipefail

export HNO_PKG="${HNO_PKG:-/home/sharpa/hno_vio_clean/src/hno_vio}"
export RUN_RTABMAP_DIR="${RUN_RTABMAP_DIR:-${HNO_PKG}/tools/run_rtabmap}"
export ROS2_WS="${ROS2_WS:-${RUN_RTABMAP_DIR}/ros2_ws}"
export RTABMAP_WS="${RTABMAP_WS:-/home/sharpa/ros2_ws/install}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros2_logs}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

require_env() {
  local name="$1"
  if [[ -z "${!name:-}" ]]; then
    echo "missing required environment variable ${name}; run scripts/run_rtabmap.sh first" >&2
    exit 2
  fi
}

for key in \
  RUN_DIR \
  VIO_RESULTS_DIR \
  RUN_CONTEXT_JSON \
  INPUT_ODOM_CSV \
  INPUT_ODOM_TXT \
  EUROC_MAV0 \
  GROUND_TRUTH_TUM \
  ODOM_FRAME \
  BASE_FRAME \
  OUT_DIR \
  INPUT_BAG \
  OUTPUT_BAG \
  RTABMAP_DB; do
  require_env "${key}"
done

export LOG_DIR="${LOG_DIR:-${OUT_DIR}/logs}"
mkdir -p "${OUT_DIR}" "${LOG_DIR}" "${ROS_LOG_DIR}"

print_paths() {
  cat <<EOF
HNO_PKG=${HNO_PKG}
RUN_RTABMAP_DIR=${RUN_RTABMAP_DIR}
ROS2_WS=${ROS2_WS}
RTABMAP_WS=${RTABMAP_WS}
RUN_DIR=${RUN_DIR}
VIO_RESULTS_DIR=${VIO_RESULTS_DIR}
RUN_CONTEXT_JSON=${RUN_CONTEXT_JSON}
EUROC_MAV0=${EUROC_MAV0}
GROUND_TRUTH_TUM=${GROUND_TRUTH_TUM}
ODOM_FRAME=${ODOM_FRAME}
BASE_FRAME=${BASE_FRAME}
OUT_DIR=${OUT_DIR}
LOG_DIR=${LOG_DIR}
INPUT_ODOM_CSV=${INPUT_ODOM_CSV}
INPUT_ODOM_TXT=${INPUT_ODOM_TXT}
INPUT_BAG=${INPUT_BAG}
OUTPUT_BAG=${OUTPUT_BAG}
RTABMAP_DB=${RTABMAP_DB}
EOF
}

source_ros2_base() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "${RTABMAP_WS}/setup.bash"
  set -u
}

source_ros2_local() {
  source_ros2_base
  set +u
  # shellcheck disable=SC1091
  source "${ROS2_WS}/install/setup.bash"
  set -u
}
