#!/usr/bin/env bash
#
# Usage:
#   scripts/00_check_inputs.sh
#
# Inputs:
#   Environment prepared by scripts/run_rtabmap.sh and common_env.sh:
#   RUN_CONTEXT_JSON, INPUT_ODOM_CSV, INPUT_ODOM_TXT, EUROC_MAV0,
#   GROUND_TRUTH_TUM, RTABMAP_WS, ROS2_WS, OUT_DIR, LOG_DIR.
#
# Outputs:
#   Writes dependency and dataset checks to:
#   ${LOG_DIR}/input_check.txt
#
# Notes:
#   This script is normally run by scripts/run_rtabmap.sh, not directly.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_env.sh"

export PATH="${HOME}/.local/bin:${PATH}"
REPORT="${LOG_DIR}/input_check.txt"

{
  print_paths
  echo
  echo "checks:"
  test -s "${RUN_CONTEXT_JSON}"
  echo "run_context.json: ok"
  test -s "${INPUT_ODOM_CSV}"
  test -s "${INPUT_ODOM_TXT}"
  echo "odom_raw.csv/txt: ok"
  test -d "${EUROC_MAV0}"
  test -s "${EUROC_MAV0}/cam0/data.csv"
  test -s "${EUROC_MAV0}/cam1/data.csv"
  test -s "${EUROC_MAV0}/cam0/sensor.yaml"
  test -s "${EUROC_MAV0}/cam1/sensor.yaml"
  echo "EuRoC ASL images/calibration: ok"
  test -s "${GROUND_TRUTH_TUM}"
  echo "ground truth TUM: ok"
  test -s /opt/ros/humble/setup.bash
  test -d "${RTABMAP_WS}"
  echo "ROS2/RTAB-Map paths: ok"
  command -v evo_ape
  command -v evo_rpe
  command -v evo_traj
  command -v colcon
  echo "host commands: ok"
  source_ros2_base
  ros2 pkg prefix rtabmap_slam
  ros2 pkg prefix rtabmap_sync
  /usr/bin/python3 - <<'PY'
import cv2
import matplotlib
import numpy
import rclpy
import rosbag2_py
import scipy
import yaml
print("python imports: ok")
PY
} | tee "${REPORT}"
