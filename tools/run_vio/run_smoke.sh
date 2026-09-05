#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WS_ROOT="$(cd "${PKG_ROOT}/../.." && pwd)"
SMOKE_ROOT="${PKG_ROOT}/smoke_results"
LOG_DIR="${SMOKE_ROOT}/logs"
SUMMARY="${SMOKE_ROOT}/summary.txt"
GT_TRAJECTORY="${PKG_ROOT}/ground_truth/euroc_mav/V1_01_easy.txt"
LAUNCH_TIMEOUT="${HNO_VIO_SMOKE_TIMEOUT:-300}"
HNO_VIO_VENV="${HNO_VIO_VENV:-/home/he/.venvs/hnovio}"

if [[ -d "${HNO_VIO_VENV}/bin" ]]; then
    export PATH="${HNO_VIO_VENV}/bin:${PATH}"
fi

GT_STRICT_MEAN=0.021
GT_STRICT_RMSE=0.040
GT_TOLERANCE_MEAN=0.025
GT_TOLERANCE_RMSE=0.050
GT_MIN_ROWS=1931
GT_MIN_DURATION=131.34

VIO_MAX_MEAN=0.8
VIO_MAX_RMSE=0.8
VIO_MIN_ROWS=2599
VIO_MIN_DURATION=137.09

GT_MEAN="N/A"
GT_RMSE="N/A"
GT_COMPLETENESS="N/A"
GT_RESULT="FAIL"
GT_WARNING="NO"
VIO_RESULT="FAIL"
declare -a RUN_MEAN=("" "N/A" "N/A" "N/A")
declare -a RUN_RMSE=("" "N/A" "N/A" "N/A")
declare -a RUN_COMPLETENESS=("" "N/A" "N/A" "N/A")
declare -a RUN_RESULT=("" "SKIPPED" "SKIPPED" "SKIPPED")

write_summary() {
    {
        echo "========================================"
        echo "HNO-VIO Smoke Test"
        echo "========================================"
        echo "Build:              ${BUILD_RESULT:-FAIL}"
        echo
        echo "GT Mapping:"
        echo "  mean:              ${GT_MEAN}"
        echo "  rmse:              ${GT_RMSE}"
        echo "  completeness:      ${GT_COMPLETENESS}"
        echo "  precision warning: ${GT_WARNING}"
        echo "  result:            ${GT_RESULT}"
        echo
        echo "Normal VIO:"
        for run in 1 2 3; do
            echo "  run ${run}:             mean=${RUN_MEAN[run]}, rmse=${RUN_RMSE[run]}, completeness=${RUN_COMPLETENESS[run]}, ${RUN_RESULT[run]}"
        done
        echo "  result:            ${VIO_RESULT}"
        echo
        echo "Overall:             ${OVERALL_RESULT:-FAIL}"
        echo "========================================"
    } | tee "${SUMMARY}"
}

float_le() {
    awk -v lhs="$1" -v rhs="$2" 'BEGIN { exit !(lhs <= rhs) }'
}

trajectory_stats() {
    /usr/bin/python3 - "$1" <<'PY'
import math
import sys

path = sys.argv[1]
timestamps = []
with open(path, "r", encoding="utf-8") as stream:
    for line in stream:
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        fields = stripped.split()
        if len(fields) != 8:
            raise ValueError(f"invalid TUM row: {stripped}")
        values = [float(field) for field in fields]
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"non-finite TUM row: {stripped}")
        timestamps.append(values[0])

if not timestamps:
    raise ValueError("trajectory has no valid rows")

print(len(timestamps), f"{timestamps[-1] - timestamps[0]:.6f}")
PY
}

run_launch() {
    local use_gt_mapping="$1"
    local output_dir="$2"
    local output_csv="$3"
    local log_file="$4"

    mkdir -p "${output_dir}"
    set +e
    timeout "${LAUNCH_TIMEOUT}s" ros2 launch hno_vio hno_vio.launch.py \
        dataset:=V1_01_easy \
        use_gt_mapping:="${use_gt_mapping}" \
        run_preprocess:=false \
        export_odom:=true \
        rviz:=false \
        play_bag:=true \
        results_root:="${output_dir}" \
        odom_output_path:="${output_csv}" \
        >"${log_file}" 2>&1
    local status=$?
    set -e

    if [[ ${status} -eq 124 ]]; then
        echo "launch timed out after ${LAUNCH_TIMEOUT}s: ${log_file}" >&2
        return 1
    fi
    if [[ ${status} -ne 0 ]]; then
        echo "launch failed with exit code ${status}: ${log_file}" >&2
        return 1
    fi
}

evaluate_trajectory() {
    local estimate="$1"
    local min_rows="$2"
    local min_duration="$3"
    local evo_log="$4"
    local stats rows duration mean rmse

    if [[ ! -s "${estimate}" ]]; then
        echo "missing or empty trajectory: ${estimate}" >&2
        return 1
    fi

    if ! stats="$(trajectory_stats "${estimate}")"; then
        echo "invalid trajectory: ${estimate}" >&2
        return 1
    fi
    read -r rows duration <<<"${stats}"
    LAST_COMPLETENESS="rows=${rows}, duration=${duration}s"
    if (( rows < min_rows )) || ! float_le "${min_duration}" "${duration}"; then
        echo "incomplete trajectory: ${LAST_COMPLETENESS}" >&2
        return 1
    fi

    set +e
    evo_ape tum "${GT_TRAJECTORY}" "${estimate}" -a -r trans_part >"${evo_log}" 2>&1
    local evo_status=$?
    set -e
    if [[ ${evo_status} -ne 0 ]]; then
        echo "evo evaluation failed: ${evo_log}" >&2
        return 1
    fi

    mean="$(awk '$1 == "mean" { print $2; exit }' "${evo_log}")"
    rmse="$(awk '$1 == "rmse" { print $2; exit }' "${evo_log}")"
    if [[ -z "${mean}" || -z "${rmse}" ]]; then
        echo "failed to parse evo statistics: ${evo_log}" >&2
        return 1
    fi

    LAST_MEAN="${mean}"
    LAST_RMSE="${rmse}"
}

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "ROS2 setup not found: ${ROS_SETUP}" >&2
    exit 1
fi

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u

if ! command -v evo_ape >/dev/null 2>&1; then
    echo "required EVO command not found: evo_ape" >&2
    echo "Expected it under ${HNO_VIO_VENV}/bin; set HNO_VIO_VENV to override." >&2
    exit 1
fi

rm -rf "${SMOKE_ROOT}"
mkdir -p "${LOG_DIR}"

cd "${WS_ROOT}"
set +e
PATH=/usr/bin:${PATH} PYTHON_EXECUTABLE=/usr/bin/python3 \
    colcon build --packages-select hno_vio \
        --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
        >"${LOG_DIR}/build.log" 2>&1
build_status=$?
set -e
if [[ ${build_status} -ne 0 ]]; then
    BUILD_RESULT="FAIL"
    OVERALL_RESULT="FAIL"
    write_summary
    echo "build failed: ${LOG_DIR}/build.log" >&2
    exit 1
fi
BUILD_RESULT="PASS"

set +u
# shellcheck disable=SC1090
source "${WS_ROOT}/install/setup.bash"
set -u

GT_DIR="${SMOKE_ROOT}/gt_mapping"
GT_CSV="${GT_DIR}/odom_gt.csv"
GT_TXT="${GT_DIR}/odom_gt.txt"
if ! run_launch true "${GT_DIR}" "${GT_CSV}" "${LOG_DIR}/gt_mapping_launch.log"; then
    OVERALL_RESULT="FAIL"
    write_summary
    exit 1
fi

LAST_MEAN="N/A"
LAST_RMSE="N/A"
LAST_COMPLETENESS="N/A"
if ! evaluate_trajectory "${GT_TXT}" "${GT_MIN_ROWS}" "${GT_MIN_DURATION}" "${LOG_DIR}/gt_mapping_evo.log"; then
    GT_MEAN="${LAST_MEAN}"
    GT_RMSE="${LAST_RMSE}"
    GT_COMPLETENESS="${LAST_COMPLETENESS}, FAIL"
    OVERALL_RESULT="FAIL"
    write_summary
    exit 1
fi
GT_MEAN="${LAST_MEAN}"
GT_RMSE="${LAST_RMSE}"
GT_COMPLETENESS="${LAST_COMPLETENESS}, PASS"

if float_le "${GT_MEAN}" "${GT_STRICT_MEAN}" && float_le "${GT_RMSE}" "${GT_STRICT_RMSE}"; then
    GT_RESULT="PASS"
elif float_le "${GT_MEAN}" "${GT_TOLERANCE_MEAN}" && float_le "${GT_RMSE}" "${GT_TOLERANCE_RMSE}"; then
    GT_RESULT="PASS"
    GT_WARNING="YES - precision regression within tolerance"
    echo "WARNING: GT mapping precision regressed: mean=${GT_MEAN}, rmse=${GT_RMSE}" >&2
else
    GT_RESULT="FAIL"
    OVERALL_RESULT="FAIL"
    write_summary
    exit 1
fi

for run in 1 2 3; do
    RUN_RESULT[run]="FAIL"
    run_dir="${SMOKE_ROOT}/vio_run_${run}"
    run_csv="${run_dir}/odom_raw.csv"
    run_txt="${run_dir}/odom_raw.txt"
    if ! run_launch false "${run_dir}" "${run_csv}" "${LOG_DIR}/vio_run_${run}_launch.log"; then
        RUN_COMPLETENESS[run]="N/A, FAIL"
        continue
    fi

    LAST_MEAN="N/A"
    LAST_RMSE="N/A"
    LAST_COMPLETENESS="N/A"
    if ! evaluate_trajectory "${run_txt}" "${VIO_MIN_ROWS}" "${VIO_MIN_DURATION}" "${LOG_DIR}/vio_run_${run}_evo.log"; then
        RUN_MEAN[run]="${LAST_MEAN}"
        RUN_RMSE[run]="${LAST_RMSE}"
        RUN_COMPLETENESS[run]="${LAST_COMPLETENESS}, FAIL"
        continue
    fi

    RUN_MEAN[run]="${LAST_MEAN}"
    RUN_RMSE[run]="${LAST_RMSE}"
    RUN_COMPLETENESS[run]="${LAST_COMPLETENESS}, PASS"
    if float_le "${LAST_MEAN}" "${VIO_MAX_MEAN}" && float_le "${LAST_RMSE}" "${VIO_MAX_RMSE}"; then
        RUN_RESULT[run]="PASS"
        VIO_RESULT="PASS"
        break
    fi
done

if [[ "${VIO_RESULT}" == "PASS" ]]; then
    OVERALL_RESULT="PASS"
    write_summary
    exit 0
fi

OVERALL_RESULT="FAIL"
write_summary
exit 1
