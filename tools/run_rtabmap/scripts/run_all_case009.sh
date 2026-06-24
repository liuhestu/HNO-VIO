#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_env.sh"

RUN_LOG="${OUT_DIR}/run.log"
{
  date -Is
  print_paths
  echo
} | tee "${RUN_LOG}"

"${SCRIPT_DIR}/00_check_inputs.sh" 2>&1 | tee -a "${RUN_LOG}"
"${SCRIPT_DIR}/01_prepare_odom.sh" 2>&1 | tee -a "${RUN_LOG}"
if [[ "${SKIP_BUILD:-0}" != "1" ]]; then
  "${SCRIPT_DIR}/02_build_ros2_ws.sh" 2>&1 | tee -a "${RUN_LOG}"
fi
if [[ "${SKIP_RECORD:-0}" != "1" ]]; then
  "${SCRIPT_DIR}/03_record_input_bag.sh" 2>&1 | tee -a "${RUN_LOG}"
fi
if [[ "${SKIP_RTABMAP:-0}" != "1" ]]; then
  "${SCRIPT_DIR}/04_run_rtabmap.sh" 2>&1 | tee -a "${RUN_LOG}"
fi
if [[ "${SKIP_EVAL:-0}" != "1" ]]; then
  "${SCRIPT_DIR}/05_export_and_eval.sh" 2>&1 | tee -a "${RUN_LOG}"
fi

{
  echo
  echo "completed:"
  echo "standard odom: ${HNO_VIO_ODOM_CSV}"
  echo "input bag: ${INPUT_BAG}"
  echo "RTAB-Map output bag: ${OUTPUT_BAG}"
  echo "RTAB-Map database: ${RTABMAP_DB}"
  echo "raw TUM: ${OUT_DIR}/hno_vio_raw.tum"
  echo "optimized TUM: ${OUT_DIR}/rtabmap_optimized.tum"
  echo "GT TUM: ${OUT_DIR}/gt.tum"
  echo "summary: ${OUT_DIR}/summary.md"
  if [[ -s "${OUT_DIR}/summary.md" ]]; then
    sed -n '1,24p' "${OUT_DIR}/summary.md"
  fi
} | tee -a "${RUN_LOG}"
