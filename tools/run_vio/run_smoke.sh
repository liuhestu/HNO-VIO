#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 DATASET [--timeout SEC]" >&2
    exit 2
fi

DATASET="$1"
shift
CASE_TIMEOUT="${HNO_CASE_TIMEOUT_SEC:-190}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --timeout)
            CASE_TIMEOUT="$2"
            shift 2
            ;;
        *)
            echo "Usage: $0 DATASET [--timeout SEC]" >&2
            exit 2
            ;;
    esac
done

require_command python3
require_command catkin_make
require_command roslaunch

STAMP="$(timestamp_utc)"
EXP_DIR="${EVAL_ROOT}/${DATASET}_${STAMP}_smoke"
MANIFEST="${EXP_DIR}/manifest.jsonl"
mkdir -p "${EXP_DIR}"

echo "Smoke experiment: ${EXP_DIR}"
echo "Building workspace..."
build_workspace

python3 "${SCRIPT_DIR}/generate_manifest.py" --dataset "${DATASET}" --count 1 --output "${MANIFEST}"
CANDIDATE_JSON="$(head -n 1 "${MANIFEST}")"

"${SCRIPT_DIR}/run_case.sh" "${DATASET}" "${EXP_DIR}/case_001" "${CANDIDATE_JSON}" --rviz true --timeout "${CASE_TIMEOUT}"

REPORT="${REPORT_ROOT}/${DATASET}_${STAMP}_smoke.md"
python3 "${SCRIPT_DIR}/make_report.py" --exp-dir "${EXP_DIR}" --dataset "${DATASET}" --output "${REPORT}" --final
echo "Smoke report: ${REPORT}"
