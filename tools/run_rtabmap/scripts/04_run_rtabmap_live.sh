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
  > "${OUT_DIR}/logs/04_live_rtabmap_launch.log" 2>&1 &
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
    wait_for_transform \
    "Mem/IncrementalMemory" \
    "Mem/InitWMWithAllNodes" \
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
  for key in use_sim_time approx_sync approx_sync_max_interval topic_queue_size sync_queue_size qos qos_camera_info; do
    printf "%s: " "${key}"
    ros2 param get /rtabmap/stereo_sync "${key}" 2>/dev/null || true
  done
} > "${OUT_DIR}/rtabmap_relevant_params.txt"

ros2 param dump /rtabmap/rtabmap > "${OUT_DIR}/rtabmap_params_dump.yaml" 2>"${OUT_DIR}/logs/04_live_rtabmap_param_dump.err" || true
ros2 param dump /rtabmap/stereo_sync > "${OUT_DIR}/stereo_sync_params_dump.yaml" 2>"${OUT_DIR}/logs/04_live_stereo_sync_param_dump.err" || true

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
  > "${OUT_DIR}/logs/04_live_rtabmap_record.log" 2>&1 &
REC_PID=$!
sleep 2

ros2 launch hno_rtabmap_replay replay_case009.launch.py \
  max_duration_sec:="${MAX_DURATION_SEC:-0.0}" \
  2>&1 | tee "${OUT_DIR}/logs/04_live_replay.log"

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
  echo "no /rtabmap/info in output bag" >&2
  exit 1
fi
