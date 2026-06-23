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

BEST_JSON="${EVAL_ROOT}/best_${DATASET}.json"
if [[ ! -f "${BEST_JSON}" ]]; then
    echo "No promoted best candidate found: ${BEST_JSON}" >&2
    echo "Run run_auto_converge.sh until full validation reaches 3/3 usable." >&2
    exit 2
fi

require_command python3
require_command roslaunch

STAMP="$(timestamp_utc)"
REPLAY_DIR="${EVAL_ROOT}/${DATASET}_${STAMP}_replay_best"
mkdir -p "${REPLAY_DIR}"

CANDIDATE_JSON="$(python3 -c 'import json,sys; data=json.load(open(sys.argv[1])); cand=data["candidate"]; cand["case_id"]="replay_best"; print(json.dumps(cand, sort_keys=True))' "${BEST_JSON}")"

"${SCRIPT_DIR}/run_case.sh" "${DATASET}" "${REPLAY_DIR}/replay_best" "${CANDIDATE_JSON}" --rviz true --timeout "${CASE_TIMEOUT}"
echo "Replay artifacts: ${REPLAY_DIR}/replay_best"
