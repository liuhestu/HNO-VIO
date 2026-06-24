#!/usr/bin/env bash
set -euo pipefail

export HNO_PKG="${HNO_PKG:-/home/sharpa/hno_vio_clean/src/hno_vio}"
export RUN_RTABMAP_DIR="${RUN_RTABMAP_DIR:-${HNO_PKG}/tools/run_rtabmap}"
export ROS2_WS="${ROS2_WS:-${RUN_RTABMAP_DIR}/ros2_ws}"
export RTABMAP_WS="${RTABMAP_WS:-/home/sharpa/ros2_ws/install}"
export EUROC_MAV0="${EUROC_MAV0:-/home/sharpa/datasets/euroc/ASL/V1_01_easy/mav0}"
export CASE_DIR="${CASE_DIR:-${HNO_PKG}/success_odom/case009_guarded_020}"
export OUT_DIR="${OUT_DIR:-${CASE_DIR}/offline_results}"
export INPUT_ODOM_CSV="${INPUT_ODOM_CSV:-${CASE_DIR}/odom.csv}"
export INPUT_ODOM_TUM="${INPUT_ODOM_TUM:-${CASE_DIR}/odom.tum}"
export HNO_VIO_ODOM_CSV="${HNO_VIO_ODOM_CSV:-${OUT_DIR}/hno_vio_odom.csv}"
export INPUT_BAG="${INPUT_BAG:-/tmp/hno_case009_rtabmap_input_bag}"
export OUTPUT_BAG="${OUTPUT_BAG:-/tmp/hno_case009_rtabmap_output_bag}"
export RTABMAP_DB="${RTABMAP_DB:-/tmp/hno_case009_rtabmap.db}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros2_logs}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

mkdir -p "${OUT_DIR}/logs" "${ROS_LOG_DIR}"

print_paths() {
  cat <<EOF
HNO_PKG=${HNO_PKG}
RUN_RTABMAP_DIR=${RUN_RTABMAP_DIR}
ROS2_WS=${ROS2_WS}
RTABMAP_WS=${RTABMAP_WS}
EUROC_MAV0=${EUROC_MAV0}
CASE_DIR=${CASE_DIR}
OUT_DIR=${OUT_DIR}
INPUT_ODOM_CSV=${INPUT_ODOM_CSV}
HNO_VIO_ODOM_CSV=${HNO_VIO_ODOM_CSV}
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
