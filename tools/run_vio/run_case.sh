#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

usage() {
    echo "Usage: $0 DATASET CASE_DIR CANDIDATE_JSON [--rviz true|false] [--timeout SEC]" >&2
}

if [[ $# -lt 3 ]]; then
    usage
    exit 2
fi

DATASET="$1"
CASE_DIR="$2"
CANDIDATE_JSON="$3"
shift 3

RVIZ="false"
CASE_TIMEOUT="${HNO_CASE_TIMEOUT_SEC:-190}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --rviz)
            RVIZ="$2"
            shift 2
            ;;
        --timeout)
            CASE_TIMEOUT="$2"
            shift 2
            ;;
        *)
            usage
            exit 2
            ;;
    esac
done

mkdir -p "${CASE_DIR}"
CASE_DIR="$(cd "${CASE_DIR}" && pwd)"
mkdir -p "${CASE_DIR}/logs" "${CASE_DIR}/evo"

BASE_CONFIG="${PKG_ROOT}/config/euroc_mav/estimator_config.yaml"
CASE_CONFIG="${CASE_DIR}/estimator_config.yaml"
BAG_PATH="$(dataset_bag_path "${DATASET}")"
GT_PATH="$(dataset_gt_path "${DATASET}")"

python3 "${SCRIPT_DIR}/materialize_candidate.py" \
    --base-config "${BASE_CONFIG}" \
    --candidate-json "${CANDIDATE_JSON}" \
    --output-config "${CASE_CONFIG}" \
    --output-params "${CASE_DIR}/params.json"

cat >"${CASE_DIR}/run_context.json" <<EOF
{
  "dataset": "${DATASET}",
  "bag_path": "${BAG_PATH}",
  "gt_path": "${GT_PATH}",
  "rviz": ${RVIZ},
  "timeout_sec": ${CASE_TIMEOUT}
}
EOF

if [[ ! -f "${BAG_PATH}" ]]; then
    echo "Bag file not found: ${BAG_PATH}" | tee "${CASE_DIR}/logs/run.log" >&2
    python3 "${SCRIPT_DIR}/analyze_case.py" --case-dir "${CASE_DIR}" --gt "${GT_PATH}" --launch-exit 2
    exit 2
fi

set +e
timeout --signal=INT --kill-after=20s "${CASE_TIMEOUT}s" \
    roslaunch hno_vio euroc_hno.launch \
        dataset:="${DATASET}" \
        config_path:="${CASE_CONFIG}" \
        bag_path:="${BAG_PATH}" \
        rviz:="${RVIZ}" \
        use_gt_init:=false \
        use_gt_mapping:=false \
        >"${CASE_DIR}/logs/run.log" 2>&1 &
LAUNCH_PID=$!

sleep 5
COLLECTOR_PID=""
if kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
    python3 "${SCRIPT_DIR}/collect_run.py" --output-dir "${CASE_DIR}" >"${CASE_DIR}/logs/collector.log" 2>&1 &
    COLLECTOR_PID=$!
fi

wait "${LAUNCH_PID}"
LAUNCH_EXIT=$?

if [[ -n "${COLLECTOR_PID}" ]]; then
    kill -INT "${COLLECTOR_PID}" >/dev/null 2>&1
    wait "${COLLECTOR_PID}" >/dev/null 2>&1
fi
set -e

TIMED_OUT_FLAG=()
if [[ "${LAUNCH_EXIT}" -eq 124 ]]; then
    TIMED_OUT_FLAG=(--timed-out)
fi

python3 "${SCRIPT_DIR}/analyze_case.py" \
    --case-dir "${CASE_DIR}" \
    --gt "${GT_PATH}" \
    --launch-exit "${LAUNCH_EXIT}" \
    "${TIMED_OUT_FLAG[@]}"

exit 0
