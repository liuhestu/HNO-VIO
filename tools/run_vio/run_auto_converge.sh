#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

usage() {
    echo "Usage: $0 DATASET [--min-runs N] [--max-runs N] [--seed N] [--timeout SEC]" >&2
}

if [[ $# -lt 1 ]]; then
    usage
    exit 2
fi

DATASET="$1"
shift

MIN_RUNS=50
MAX_RUNS=150
SEED=20260624
CASE_TIMEOUT="${HNO_CASE_TIMEOUT_SEC:-190}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --min-runs)
            MIN_RUNS="$2"
            shift 2
            ;;
        --max-runs)
            MAX_RUNS="$2"
            shift 2
            ;;
        --seed)
            SEED="$2"
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

if (( MIN_RUNS < 50 )); then
    MIN_RUNS=50
fi
if (( MAX_RUNS > 150 )); then
    MAX_RUNS=150
fi
if (( MAX_RUNS < MIN_RUNS )); then
    echo "max-runs must be >= min-runs after clamping min-runs to 50" >&2
    exit 2
fi

require_command python3
require_command catkin_make
require_command roslaunch
require_command evo_ape
require_command evo_rpe

STAMP="$(timestamp_utc)"
EXP_DIR="${EVAL_ROOT}/${DATASET}_${STAMP}"
MANIFEST="${EXP_DIR}/manifest.jsonl"
mkdir -p "${EXP_DIR}"

echo "Experiment: ${EXP_DIR}"
echo "Building workspace..."
build_workspace

python3 "${SCRIPT_DIR}/generate_manifest.py" \
    --dataset "${DATASET}" \
    --count "${MAX_RUNS}" \
    --seed "${SEED}" \
    --output "${MANIFEST}"

CASE_INDEX=0
USABLE_COUNT=0

while IFS= read -r CANDIDATE_JSON; do
    CASE_INDEX=$((CASE_INDEX + 1))
    CASE_ID="$(python3 -c 'import json,sys; print(json.loads(sys.argv[1])["case_id"])' "${CANDIDATE_JSON}")"
    CASE_DIR="${EXP_DIR}/${CASE_ID}"
    echo "Running ${CASE_ID} (${CASE_INDEX}/${MAX_RUNS})..."

    "${SCRIPT_DIR}/run_case.sh" "${DATASET}" "${CASE_DIR}" "${CANDIDATE_JSON}" --rviz false --timeout "${CASE_TIMEOUT}"

    CASE_USABLE="$(python3 -c 'import json,sys; print(1 if json.load(open(sys.argv[1]))["usable"] else 0)' "${CASE_DIR}/summary.json")"
    if [[ "${CASE_USABLE}" == "1" ]]; then
        USABLE_COUNT=$((USABLE_COUNT + 1))
    fi

    if (( CASE_INDEX % 10 == 0 )); then
        REPORT="${REPORT_ROOT}/${DATASET}_${STAMP}_round_$(printf '%03d' "${CASE_INDEX}").md"
        python3 "${SCRIPT_DIR}/make_report.py" --exp-dir "${EXP_DIR}" --dataset "${DATASET}" --output "${REPORT}"
        echo "Wrote report: ${REPORT}"
    fi

    if (( CASE_INDEX >= MIN_RUNS && USABLE_COUNT > 0 )); then
        echo "Usable candidate found after ${CASE_INDEX} runs; stopping search and starting validation."
        break
    fi
done < "${MANIFEST}"

FINAL_REPORT="${REPORT_ROOT}/${DATASET}_${STAMP}_final.md"
python3 "${SCRIPT_DIR}/make_report.py" --exp-dir "${EXP_DIR}" --dataset "${DATASET}" --output "${FINAL_REPORT}" --final

if (( USABLE_COUNT == 0 )); then
    {
        echo ""
        echo "## Full Validation"
        echo ""
        echo "No usable candidate found; validation was skipped."
    } >> "${FINAL_REPORT}"
    echo "Final report: ${FINAL_REPORT}"
    exit 0
fi

BEST_JSON="${EXP_DIR}/best_candidate.json"
BEST_CASE="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["summary"]["case_id"])' "${BEST_JSON}")"
VALIDATION_DIR="${EXP_DIR}/validation_${BEST_CASE}"
mkdir -p "${VALIDATION_DIR}"

VALIDATION_USABLE=0
for REPEAT in 1 2 3; do
    CANDIDATE_JSON="$(python3 -c 'import json,sys; data=json.load(open(sys.argv[1])); cand=data["candidate"]; cand["case_id"]=data["summary"]["case_id"]+"_repeat_"+sys.argv[2]; print(json.dumps(cand, sort_keys=True))' "${BEST_JSON}" "${REPEAT}")"
    REPEAT_DIR="${VALIDATION_DIR}/${BEST_CASE}_repeat_${REPEAT}"
    echo "Validation repeat ${REPEAT}/3 for ${BEST_CASE}..."
    "${SCRIPT_DIR}/run_case.sh" "${DATASET}" "${REPEAT_DIR}" "${CANDIDATE_JSON}" --rviz false --timeout "${CASE_TIMEOUT}"
    REPEAT_USABLE="$(python3 -c 'import json,sys; print(1 if json.load(open(sys.argv[1]))["usable"] else 0)' "${REPEAT_DIR}/summary.json")"
    if [[ "${REPEAT_USABLE}" == "1" ]]; then
        VALIDATION_USABLE=$((VALIDATION_USABLE + 1))
    fi
done

PROMOTION="rejected"
if (( VALIDATION_USABLE == 3 )); then
    PROMOTION="default_candidate"
    python3 -c 'import json,sys,pathlib; data=json.load(open(sys.argv[1])); data["validation_usable"]="3/3"; pathlib.Path(sys.argv[2]).write_text(json.dumps(data, indent=2, sort_keys=True)+"\n")' \
        "${BEST_JSON}" "${EVAL_ROOT}/best_${DATASET}.json"
elif (( VALIDATION_USABLE == 2 )); then
    PROMOTION="unstable_promising"
fi

{
    echo ""
    echo "## Full Validation"
    echo ""
    echo "- best_case: \`${BEST_CASE}\`"
    echo "- usable_repeats: ${VALIDATION_USABLE}/3"
    echo "- promotion: ${PROMOTION}"
    echo "- validation_dir: \`${VALIDATION_DIR}\`"
} >> "${FINAL_REPORT}"

echo "Final report: ${FINAL_REPORT}"
