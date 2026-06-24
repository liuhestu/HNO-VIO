#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_env.sh"

PYTHONPATH="${ROS2_WS}/src/hno_rtabmap_replay:${PYTHONPATH:-}" /usr/bin/python3 -m hno_rtabmap_replay.prepare_odom \
  --input-csv "${INPUT_ODOM_CSV}" \
  --input-tum "${INPUT_ODOM_TUM}" \
  --euroc-mav0 "${EUROC_MAV0}" \
  --output-csv "${HNO_VIO_ODOM_CSV}" \
  --output-tum "${OUT_DIR}/hno_vio_odom.tum" \
  --report "${OUT_DIR}/odom_check.txt" \
  --max-step-translation 0.50 \
  --max-step-rotation-deg 30.0 \
  2>&1 | tee "${OUT_DIR}/logs/01_prepare_odom.log"
