#!/usr/bin/env bash
#
# Usage:
#   scripts/05_export_and_eval.sh
#
# Inputs:
#   Environment prepared by scripts/run_rtabmap.sh and common_env.sh:
#   INPUT_BAG, OUTPUT_BAG, OUT_DIR, LOG_DIR, INPUT_ODOM_TXT,
#   GROUND_TRUTH_TUM.
#   Optional: TF_MATCH_TIME.
#
# Outputs:
#   Exports optimized RTAB-Map graph odometry and evaluation artifacts:
#   ${OUT_DIR}/odom_optimized.txt
#   ${OUT_DIR}/summary.md
#   ${OUT_DIR}/evo_*.txt
#   ${OUT_DIR}/evo_*.pdf
#   ${LOG_DIR}/05_export.log
#   ${LOG_DIR}/05_diagnostics.log
#   ${LOG_DIR}/05_eval.log
#   ${LOG_DIR}/rpy_raw_vs_optimized.{csv,pdf}
#
# Notes:
#   This script is normally run by scripts/run_rtabmap.sh after
#   scripts/04_run_rtabmap.sh.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_env.sh"

export PATH="${HOME}/.local/bin:${PATH}"
source_ros2_local

hno_export_optimized_tf \
  --bag "${OUTPUT_BAG}" \
  --out-dir "${OUT_DIR}" \
  --tf-match-time "${TF_MATCH_TIME:-bag}" \
  --min-graph-poses 20 \
  --warn-graph-poses 50 \
  2>&1 | tee "${LOG_DIR}/05_export.log"

ros2 run hno_rtabmap_replay hno_analyze_rtabmap_bag \
  --input-bag "${INPUT_BAG}" \
  --output-bag "${OUTPUT_BAG}" \
  --out-dir "${LOG_DIR}" \
  2>&1 | tee "${LOG_DIR}/05_diagnostics.log"

hno_eval_raw_vs_optimized \
  --out-dir "${OUT_DIR}" \
  --raw-tum "${INPUT_ODOM_TXT}" \
  --optimized-tum "${OUT_DIR}/odom_optimized.txt" \
  --gt-tum "${GROUND_TRUTH_TUM}" \
  --evo-ape "$(command -v evo_ape)" \
  --evo-rpe "$(command -v evo_rpe)" \
  --evo-traj "$(command -v evo_traj)" \
  2>&1 | tee "${LOG_DIR}/05_eval.log"
