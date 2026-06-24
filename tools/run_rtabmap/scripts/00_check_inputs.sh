#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_env.sh"

export PATH="${HOME}/.local/bin:${PATH}"
REPORT="${OUT_DIR}/input_check.txt"

{
  print_paths
  echo
  echo "checks:"
  test -s "${INPUT_ODOM_CSV}" || test -s "${INPUT_ODOM_TUM}"
  echo "odom source: ok"
  test -d "${EUROC_MAV0}"
  test -s "${EUROC_MAV0}/cam0/data.csv"
  test -s "${EUROC_MAV0}/cam1/data.csv"
  test -s "${EUROC_MAV0}/cam0/sensor.yaml"
  test -s "${EUROC_MAV0}/cam1/sensor.yaml"
  test -s "${EUROC_MAV0}/state_groundtruth_estimate0/data.csv"
  echo "EuRoC ASL V1_01_easy: ok"
  test -s /opt/ros/humble/setup.bash
  test -d "${RTABMAP_WS}"
  echo "ROS2/RTAB-Map paths: ok"
  command -v evo_ape
  command -v evo_rpe
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
