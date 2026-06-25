#!/usr/bin/env bash
#
# Usage:
#   scripts/03_record_input_bag.sh
#
# Inputs:
#   Environment prepared by scripts/run_rtabmap.sh and common_env.sh:
#   INPUT_ODOM_CSV, INPUT_ODOM_TXT, EUROC_MAV0, INPUT_BAG, LOG_DIR.
#   Optional: MAX_DURATION_SEC.
#
# Outputs:
#   Replays EuRoC images plus HNO-VIO odometry into ROS 2 and records:
#   ${INPUT_BAG}
#   ${LOG_DIR}/03_record_input_bag_record.log
#   ${LOG_DIR}/03_record_input_bag_replay.log
#   ${LOG_DIR}/input_bag_info.txt
#
# Notes:
#   This script is normally run by scripts/run_rtabmap.sh after the ROS 2
#   workspace is built or sourced.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_env.sh"

source_ros2_local
rm -rf "${INPUT_BAG}"

REC_PID=""
cleanup() {
  if [[ -n "${REC_PID}" ]] && kill -0 "${REC_PID}" 2>/dev/null; then
    kill -INT "${REC_PID}" 2>/dev/null || true
    wait "${REC_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

ros2 bag record \
  -o "${INPUT_BAG}" \
  /cam0/image_rect \
  /cam1/image_rect \
  /cam0/camera_info \
  /cam1/camera_info \
  /hno_vio/odom \
  /tf \
  /tf_static \
  /clock \
  > "${LOG_DIR}/03_record_input_bag_record.log" 2>&1 &
REC_PID=$!

sleep 2
ros2 launch hno_rtabmap_replay replay_case009.launch.py \
  max_duration_sec:="${MAX_DURATION_SEC:-0.0}" \
  2>&1 | tee "${LOG_DIR}/03_record_input_bag_replay.log"

sleep 2
cleanup
trap - EXIT
ros2 bag info "${INPUT_BAG}" | tee "${LOG_DIR}/input_bag_info.txt"
