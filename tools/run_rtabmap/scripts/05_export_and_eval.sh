#!/usr/bin/env bash
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
  2>&1 | tee "${OUT_DIR}/logs/05_export.log"

ros2 run hno_rtabmap_replay hno_analyze_rtabmap_bag \
  --input-bag "${INPUT_BAG}" \
  --output-bag "${OUTPUT_BAG}" \
  --out-dir "${OUT_DIR}" \
  2>&1 | tee "${OUT_DIR}/logs/05_diagnostics.log"

hno_eval_raw_vs_optimized \
  --out-dir "${OUT_DIR}" \
  --euroc-mav0 "${EUROC_MAV0}" \
  --evo-ape "$(command -v evo_ape)" \
  --evo-rpe "$(command -v evo_rpe)" \
  2>&1 | tee "${OUT_DIR}/logs/05_eval.log"
