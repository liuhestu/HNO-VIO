#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_env.sh"

source_ros2_local
rm -rf "${OUTPUT_BAG}" "${RTABMAP_DB}"

LAUNCH_PID=""
REC_PID=""
cleanup() {
  if [[ -n "${REC_PID}" ]] && kill -0 "${REC_PID}" 2>/dev/null; then
    kill -INT "${REC_PID}" 2>/dev/null || true
    wait "${REC_PID}" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

ros2 launch hno_rtabmap_replay rtabmap_case009.launch.py \
  > "${OUT_DIR}/logs/04_run_rtabmap_launch.log" 2>&1 &
LAUNCH_PID=$!
sleep 6

{
  echo "node list:"
  ros2 node list || true
  echo
  echo "rtabmap relevant params:"
  for key in \
    use_sim_time \
    frame_id \
    odom_frame_id \
    map_frame_id \
    subscribe_rgbd \
    subscribe_odom \
    approx_sync \
    publish_tf \
    publish_tf_map \
    database_path \
    "Mem/IncrementalMemory" \
    "RGBD/ProximityBySpace" \
    "RGBD/OptimizeFromGraphEnd" \
    "RGBD/NeighborLinkRefining" \
    "RGBD/LoopClosureReextractFeatures" \
    "Vis/MinInliers" \
    "Kp/MaxFeatures" \
    "Rtabmap/DetectionRate"; do
    printf "%s: " "${key}"
    ros2 param get /rtabmap/rtabmap "${key}" 2>/dev/null || true
  done
  echo
  echo "stereo_sync relevant params:"
  for key in use_sim_time approx_sync approx_sync_max_interval; do
    printf "%s: " "${key}"
    ros2 param get /rtabmap/stereo_sync "${key}" 2>/dev/null || true
  done
} > "${OUT_DIR}/rtabmap_relevant_params.txt"

ros2 param dump /rtabmap/rtabmap > "${OUT_DIR}/rtabmap_params_dump.yaml" 2>"${OUT_DIR}/logs/04_rtabmap_param_dump.err" || true
ros2 param dump /rtabmap/stereo_sync > "${OUT_DIR}/stereo_sync_params_dump.yaml" 2>"${OUT_DIR}/logs/04_stereo_sync_param_dump.err" || true

ros2 bag record \
  -o "${OUTPUT_BAG}" \
  /tf \
  /tf_static \
  /hno_vio/odom \
  /rtabmap/rgbd_image \
  /rtabmap/mapData \
  /rtabmap/info \
  /rtabmap/global_path \
  /rtabmap/local_path \
  > "${OUT_DIR}/logs/04_run_rtabmap_record.log" 2>&1 &
REC_PID=$!
sleep 2

set +e
timeout "${BAG_PLAY_TIMEOUT_SEC:-240}" ros2 bag play "${INPUT_BAG}" --read-ahead-queue-size 1000 \
  2>&1 | tee "${OUT_DIR}/logs/04_run_rtabmap_play.log"
PLAY_STATUS="${PIPESTATUS[0]}"
set -e
if [[ "${PLAY_STATUS}" != "0" && "${PLAY_STATUS}" != "124" ]]; then
  echo "ros2 bag play failed with status ${PLAY_STATUS}" >&2
  exit "${PLAY_STATUS}"
fi
if [[ "${PLAY_STATUS}" == "124" ]]; then
  echo "ros2 bag play reached BAG_PLAY_TIMEOUT_SEC=${BAG_PLAY_TIMEOUT_SEC:-240}; continuing cleanup" | tee -a "${OUT_DIR}/logs/04_run_rtabmap_play.log"
fi

sleep 8
cleanup
trap - EXIT

ros2 bag info "${OUTPUT_BAG}" | tee "${OUT_DIR}/output_bag_info.txt"
echo "${RTABMAP_DB}" > "${OUT_DIR}/rtabmap_db_path.txt"

if [[ ! -d "${OUTPUT_BAG}" ]]; then
  echo "missing output bag: ${OUTPUT_BAG}" >&2
  exit 1
fi
if [[ ! -s "${RTABMAP_DB}" ]]; then
  echo "missing or empty RTAB-Map DB: ${RTABMAP_DB}" >&2
  exit 1
fi
if ! grep -q "/rtabmap/info" "${OUT_DIR}/output_bag_info.txt"; then
  {
    echo "no /rtabmap/info in output bag"
    echo
    ros2 bag info "${INPUT_BAG}" || true
    echo
    tail -200 "${OUT_DIR}/logs/04_run_rtabmap_launch.log" || true
  } > "${OUT_DIR}/logs/rtabmap_failure_debug.txt"
  exit 1
fi
