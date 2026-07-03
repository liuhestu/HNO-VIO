#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<EOF
Usage:
  $0 /path/to/run_YYYYmmddTHHMMSS/vio_results/rtabmap_input_db3
EOF
}

if [[ $# -ne 1 ]]; then
  usage >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
HNO_PKG="$(cd "${SCRIPT_DIR}/../.." && pwd)"
INPUT_BAG="$(realpath "$1")"

if [[ ! -d "${INPUT_BAG}" || ! -s "${INPUT_BAG}/metadata.yaml" ]]; then
  echo "missing rtabmap_input_db3: ${INPUT_BAG}" >&2
  echo "Generate it first with: ros2 launch hno_vio hno_vio.launch.py run_preprocess:=true" >&2
  exit 2
fi

VIO_RESULTS_DIR="$(dirname "${INPUT_BAG}")"
RUN_DIR="$(dirname "${VIO_RESULTS_DIR}")"
if [[ "$(basename "${VIO_RESULTS_DIR}")" != "vio_results" ]]; then
  echo "input bag must be under a vio_results directory: ${INPUT_BAG}" >&2
  exit 2
fi

RUN_CONTEXT_JSON="${RUN_DIR}/run_context.json"
if [[ ! -s "${RUN_CONTEXT_JSON}" ]]; then
  echo "missing run_context.json: ${RUN_CONTEXT_JSON}" >&2
  exit 2
fi

OFFLINE_DIR="${RUN_DIR}/offline_results"
LOG_DIR="${OFFLINE_DIR}/logs"
OUTPUT_BAG="${OFFLINE_DIR}/rtabmap_output.bag"
RTABMAP_DB="${OFFLINE_DIR}/rtabmap.db"

rm -rf "${OFFLINE_DIR}"
mkdir -p "${LOG_DIR}"

source_if_exists() {
  local file="$1"
  if [[ -s "${file}" ]]; then
    # shellcheck disable=SC1090
    source "${file}"
  fi
}

set +u
source_if_exists /opt/ros/humble/setup.bash
source_if_exists /home/sharpa/ros2_ws/install/setup.bash
source_if_exists "${WORKSPACE_ROOT}/install/setup.bash"
set -u

export ROS_LOG_DIR="${LOG_DIR}/ros"
mkdir -p "${ROS_LOG_DIR}"

PIDS=()
cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -INT "${pid}" 2>/dev/null || true
    fi
  done
  for pid in "${PIDS[@]:-}"; do
    wait "${pid}" 2>/dev/null || true
  done
}
trap cleanup EXIT

echo "hno_rtabmap start: $(date -Is)" | tee "${LOG_DIR}/run.log"
echo "input_bag=${INPUT_BAG}" | tee -a "${LOG_DIR}/run.log"
echo "offline_dir=${OFFLINE_DIR}" | tee -a "${LOG_DIR}/run.log"

ros2 run rtabmap_sync stereo_sync \
  --ros-args \
  -p use_sim_time:=true \
  -p approx_sync:=true \
  -p approx_sync_max_interval:=0.0 \
  -p topic_queue_size:=100 \
  -p sync_queue_size:=100 \
  -p qos:=1 \
  -p qos_camera_info:=1 \
  -r left/image_rect:=/cam0/image_rect \
  -r right/image_rect:=/cam1/image_rect \
  -r left/camera_info:=/cam0/camera_info \
  -r right/camera_info:=/cam1/camera_info \
  -r rgbd_image:=/rtabmap/rgbd_image \
  > "${LOG_DIR}/stereo_sync.log" 2>&1 &
PIDS+=("$!")

ros2 run rtabmap_slam rtabmap --delete_db_on_start \
  --ros-args \
  -p use_sim_time:=true \
  -p frame_id:=base_link \
  -p odom_frame_id:=odom \
  -p map_frame_id:=map \
  -p subscribe_rgbd:=true \
  -p subscribe_stereo:=false \
  -p subscribe_rgb:=false \
  -p subscribe_depth:=false \
  -p subscribe_odom_info:=false \
  -p approx_sync:=true \
  -p publish_tf:=true \
  -p publish_tf_map:=true \
  -p database_path:="${RTABMAP_DB}" \
  -p wait_for_transform:=1.0 \
  -p Mem/IncrementalMemory:=true \
  -p Mem/InitWMWithAllNodes:=false \
  -p Reg/Force3DoF:=false \
  -p Rtabmap/CreateIntermediateNodes:=true \
  -p Rtabmap/DetectionRate:=5 \
  -p RGBD/LinearUpdate:=0.03 \
  -p RGBD/AngularUpdate:=0.03 \
  -p RGBD/OptimizeFromGraphEnd:=true \
  -p RGBD/NeighborLinkRefining:=true \
  -p Vis/MinInliers:=12 \
  -p Kp/MaxFeatures:=800 \
  -r rgbd_image:=/rtabmap/rgbd_image \
  > "${LOG_DIR}/rtabmap.log" 2>&1 &
PIDS+=("$!")

sleep 2

ros2 bag record \
  -o "${OUTPUT_BAG}" \
  /hno_vio/odom \
  /rtabmap/rgbd_image \
  /rtabmap/mapData \
  /rtabmap/info \
  /rtabmap/global_path \
  /rtabmap/local_path \
  /tf \
  /tf_static \
  /clock \
  > "${LOG_DIR}/record_output.log" 2>&1 &
PIDS+=("$!")

sleep 2
ros2 bag play "${INPUT_BAG}" --clock 2>&1 | tee "${LOG_DIR}/play_input.log"
sleep 3
cleanup
trap - EXIT

ros2 bag info "${OUTPUT_BAG}" > "${LOG_DIR}/rtabmap_output_bag_info.txt" 2>&1 || true

"${SCRIPT_DIR}/export_optimized_odom.py" \
  --bag "${OUTPUT_BAG}" \
  --out "${OFFLINE_DIR}/odom_optimized.txt" \
  2>&1 | tee "${LOG_DIR}/export_optimized_odom.log"

"${SCRIPT_DIR}/eval_and_summary.py" \
  --run-dir "${RUN_DIR}" \
  --input-bag "${INPUT_BAG}" \
  --output-bag "${OUTPUT_BAG}" \
  --offline-dir "${OFFLINE_DIR}" \
  2>&1 | tee "${LOG_DIR}/eval_and_summary.log"

echo "completed:"
echo "offline results: ${OFFLINE_DIR}"
echo "output bag: ${OUTPUT_BAG}"
echo "rtabmap db: ${RTABMAP_DB}"
echo "optimized odom: ${OFFLINE_DIR}/odom_optimized.txt"
echo "summary: ${OFFLINE_DIR}/summary.md"
