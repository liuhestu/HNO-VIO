#!/usr/bin/env bash
#
# Usage:
#   scripts/run_rtabmap.sh [--skip-build] /path/to/run_YYYYmmddTHHMMSS/vio_results
#   scripts/run_rtabmap.sh [--skip-build] /path/to/run_YYYYmmddTHHMMSS/vio_results/odom_raw.csv
#
# Inputs:
#   A run directory created by HNO-VIO odom export:
#   run_YYYYmmddTHHMMSS/run_context.json
#   run_YYYYmmddTHHMMSS/vio_results/odom_raw.csv
#   run_YYYYmmddTHHMMSS/vio_results/odom_raw.txt
#   The context JSON must provide dataset, euroc_mav0, ground_truth_tum,
#   odom_csv, odom_tum, odom_frame, and base_frame.
#
# Outputs:
#   Recreates run_YYYYmmddTHHMMSS/offline_results/ with:
#   rtabmap_input.bag, rtabmap_output.bag, rtabmap.db,
#   odom_optimized.txt, summary.md, evo plots/metrics, logs, and RPY plots.
#
# Notes:
#   This is the main entry point for the offline RTAB-Map pipeline.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HNO_PKG_DEFAULT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
export HNO_PKG="${HNO_PKG:-${HNO_PKG_DEFAULT}}"

usage() {
  cat <<EOF
Usage:
  $0 [--skip-build] /path/to/run_YYYYmmddTHHMMSS/vio_results
  $0 [--skip-build] /path/to/run_YYYYmmddTHHMMSS/vio_results/odom_raw.csv
EOF
}

SKIP_BUILD=0
INPUT_PATH=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-build)
      SKIP_BUILD=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --*)
      echo "unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      if [[ -n "${INPUT_PATH}" ]]; then
        echo "multiple input paths provided" >&2
        usage >&2
        exit 2
      fi
      INPUT_PATH="$1"
      shift
      ;;
  esac
done

if [[ -z "${INPUT_PATH}" ]]; then
  echo "missing odom input path" >&2
  usage >&2
  exit 2
fi
if [[ ! -e "${INPUT_PATH}" ]]; then
  echo "input path does not exist: ${INPUT_PATH}" >&2
  exit 2
fi

INPUT_PATH="$(realpath "${INPUT_PATH}")"
if [[ -d "${INPUT_PATH}" ]]; then
  export VIO_RESULTS_DIR="${INPUT_PATH}"
  export INPUT_ODOM_CSV="${VIO_RESULTS_DIR}/odom_raw.csv"
else
  if [[ "$(basename "${INPUT_PATH}")" != "odom_raw.csv" ]]; then
    echo "input file must be named odom_raw.csv: ${INPUT_PATH}" >&2
    exit 2
  fi
  export INPUT_ODOM_CSV="${INPUT_PATH}"
  export VIO_RESULTS_DIR="$(dirname "${INPUT_ODOM_CSV}")"
fi

if [[ "$(basename "${VIO_RESULTS_DIR}")" != "vio_results" ]]; then
  echo "input must be inside a vio_results directory: ${VIO_RESULTS_DIR}" >&2
  exit 2
fi

export RUN_DIR="$(dirname "${VIO_RESULTS_DIR}")"
case "$(basename "${RUN_DIR}")" in
  run_*) ;;
  *)
    echo "input must be under a run_* directory: ${RUN_DIR}" >&2
    exit 2
    ;;
esac

export RUN_CONTEXT_JSON="${RUN_DIR}/run_context.json"
if [[ ! -s "${RUN_CONTEXT_JSON}" ]]; then
  echo "missing run_context.json: ${RUN_CONTEXT_JSON}" >&2
  exit 2
fi

eval "$(/usr/bin/python3 - "${RUN_DIR}" "${RUN_CONTEXT_JSON}" "${INPUT_ODOM_CSV}" <<'PY'
import json
import shlex
import sys
from pathlib import Path

run_dir = Path(sys.argv[1]).resolve()
context_path = Path(sys.argv[2]).resolve()
input_odom_csv = Path(sys.argv[3]).resolve()
data = json.loads(context_path.read_text(encoding="utf-8"))
required = ["dataset", "euroc_mav0", "ground_truth_tum", "odom_csv", "odom_tum", "odom_frame", "base_frame"]
missing = [key for key in required if not data.get(key)]
if missing:
    raise SystemExit(f"run_context.json missing required fields: {', '.join(missing)}")

def resolve_run_path(value):
    path = Path(value)
    if not path.is_absolute():
        path = run_dir / path
    return path.resolve()

context_odom_csv = resolve_run_path(data["odom_csv"])
context_odom_tum = resolve_run_path(data["odom_tum"])
if context_odom_csv != input_odom_csv:
    raise SystemExit(f"input odom does not match run_context.json odom_csv: {input_odom_csv} != {context_odom_csv}")
if context_odom_tum.name != "odom_raw.txt":
    raise SystemExit(f"run_context.json odom_tum must resolve to odom_raw.txt: {context_odom_tum}")

out_dir = run_dir / "offline_results"
values = {
    "DATASET": str(data["dataset"]),
    "EUROC_MAV0": str(Path(data["euroc_mav0"]).resolve()),
    "GROUND_TRUTH_TUM": str(Path(data["ground_truth_tum"]).resolve()),
    "INPUT_ODOM_TXT": str(context_odom_tum),
    "ODOM_FRAME": str(data["odom_frame"]),
    "BASE_FRAME": str(data["base_frame"]),
    "OUT_DIR": str(out_dir),
    "INPUT_BAG": str(out_dir / "rtabmap_input.bag"),
    "OUTPUT_BAG": str(out_dir / "rtabmap_output.bag"),
    "RTABMAP_DB": str(out_dir / "rtabmap.db"),
    "LOG_DIR": str(out_dir / "logs"),
}
for key, value in values.items():
    print(f"export {key}={shlex.quote(value)}")
PY
)"

if [[ ! -s "${INPUT_ODOM_CSV}" ]]; then
  echo "missing or empty odom csv: ${INPUT_ODOM_CSV}" >&2
  exit 2
fi
if [[ ! -s "${INPUT_ODOM_TXT}" ]]; then
  echo "missing or empty odom txt: ${INPUT_ODOM_TXT}" >&2
  exit 2
fi

rm -rf "${OUT_DIR}"
mkdir -p "${LOG_DIR}"
exec > >(tee "${LOG_DIR}/run.log") 2>&1

echo "run_rtabmap start: $(date -Is)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_env.sh"
print_paths
echo

"${SCRIPT_DIR}/00_check_inputs.sh"
"${SCRIPT_DIR}/01_prepare_replay.sh"
if [[ "${SKIP_BUILD}" != "1" ]]; then
  "${SCRIPT_DIR}/02_build_ros2_ws.sh"
fi
"${SCRIPT_DIR}/03_record_input_bag.sh"
"${SCRIPT_DIR}/04_run_rtabmap.sh"
"${SCRIPT_DIR}/05_export_and_eval.sh"

echo
echo "completed:"
echo "offline results: ${OUT_DIR}"
echo "input bag: ${INPUT_BAG}"
echo "output bag: ${OUTPUT_BAG}"
echo "rtabmap db: ${RTABMAP_DB}"
echo "optimized odom: ${OUT_DIR}/odom_optimized.txt"
echo "summary: ${OUT_DIR}/summary.md"
if [[ -s "${OUT_DIR}/summary.md" ]]; then
  sed -n '1,80p' "${OUT_DIR}/summary.md"
fi
