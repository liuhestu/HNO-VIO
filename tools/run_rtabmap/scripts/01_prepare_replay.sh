#!/usr/bin/env bash
#
# Usage:
#   scripts/01_prepare_replay.sh
#
# Inputs:
#   Environment prepared by scripts/run_rtabmap.sh and common_env.sh:
#   INPUT_ODOM_CSV, INPUT_ODOM_TXT, EUROC_MAV0, ROS2_WS, LOG_DIR.
#
# Outputs:
#   Validates HNO-VIO odometry against EuRoC camera timestamps and writes:
#   ${LOG_DIR}/odom_check.txt
#   ${LOG_DIR}/01_prepare_replay.log
#
# Notes:
#   This script is normally run by scripts/run_rtabmap.sh after
#   scripts/00_check_inputs.sh.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_env.sh"

PYTHONPATH="${ROS2_WS}/src/hno_rtabmap_replay:${PYTHONPATH:-}" /usr/bin/python3 - "${INPUT_ODOM_CSV}" "${INPUT_ODOM_TXT}" "${EUROC_MAV0}" "${LOG_DIR}/odom_check.txt" <<'PY' \
  2>&1 | tee "${LOG_DIR}/01_prepare_replay.log"
import sys
from pathlib import Path

from hno_rtabmap_replay.euroc_utils import read_euroc_image_csv
from hno_rtabmap_replay.odom_utils import read_hno_odom_csv, read_tum, validate_poses

csv_path = Path(sys.argv[1])
txt_path = Path(sys.argv[2])
euroc_mav0 = Path(sys.argv[3])
report_path = Path(sys.argv[4])

csv_poses = read_hno_odom_csv(csv_path)
txt_poses = read_tum(txt_path)
if len(csv_poses) != len(txt_poses):
    raise RuntimeError(f"csv/txt pose count mismatch: {len(csv_poses)} vs {len(txt_poses)}")
if csv_poses and txt_poses:
    if csv_poses[0].stamp_ns != txt_poses[0].stamp_ns or csv_poses[-1].stamp_ns != txt_poses[-1].stamp_ns:
        raise RuntimeError("csv/txt timestamp range mismatch")

cam_rows = read_euroc_image_csv(euroc_mav0 / "cam0" / "data.csv")
if not cam_rows:
    raise RuntimeError("cam0/data.csv is empty")
stats = validate_poses(
    csv_poses,
    cam_start_ns=cam_rows[0][0],
    cam_end_ns=cam_rows[-1][0],
    max_step_translation=0.50,
    max_step_rotation_deg=30.0,
)
stats["first_cam0_timestamp_ns"] = cam_rows[0][0]
stats["last_cam0_timestamp_ns"] = cam_rows[-1][0]
stats["odom_vs_cam0_start_diff_sec"] = (csv_poses[0].stamp_ns - cam_rows[0][0]) * 1e-9
stats["odom_vs_cam0_end_diff_sec"] = (csv_poses[-1].stamp_ns - cam_rows[-1][0]) * 1e-9

lines = [
    f"input_csv: {csv_path}",
    f"input_txt: {txt_path}",
    f"euroc_mav0: {euroc_mav0}",
]
for key in [
    "row_count",
    "start_timestamp_ns",
    "end_timestamp_ns",
    "duration_sec",
    "mean_hz",
    "first_cam0_timestamp_ns",
    "last_cam0_timestamp_ns",
    "odom_vs_cam0_start_diff_sec",
    "odom_vs_cam0_end_diff_sec",
    "quaternion_norm_bad_count",
    "nan_count",
    "max_step_translation_m",
    "max_step_rotation_deg",
    "max_step_time_sec",
    "large_step_count",
]:
    lines.append(f"{key}: {stats[key]}")
report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
print(report_path)
PY
