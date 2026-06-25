#!/usr/bin/env bash
#
# Usage:
#   scripts/04_run_rtabmap.sh
#
# Inputs:
#   Environment prepared by scripts/run_rtabmap.sh and common_env.sh:
#   INPUT_BAG, OUTPUT_BAG, RTABMAP_DB, LOG_DIR.
#   Optional: BAG_PLAY_TIMEOUT_SEC.
#
# Outputs:
#   Runs RTAB-Map on ${INPUT_BAG} and records:
#   ${OUTPUT_BAG}
#   ${RTABMAP_DB}
#   ${LOG_DIR}/04_run_rtabmap_launch.log
#   ${LOG_DIR}/04_run_rtabmap_record.log
#   ${LOG_DIR}/04_run_rtabmap_play.log
#   ${LOG_DIR}/output_bag_info.txt
#   ${LOG_DIR}/rtabmap_db_path.txt
#
# Notes:
#   This script is normally run by scripts/run_rtabmap.sh after
#   scripts/03_record_input_bag.sh.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_env.sh"

source_ros2_local
rm -rf "${OUTPUT_BAG}" "${RTABMAP_DB}"

LAUNCH_PID=""
REC_PID=""
stop_pid() {
  local pid="$1"
  local label="$2"
  if [[ -z "${pid}" ]] || ! kill -0 "${pid}" 2>/dev/null; then
    return
  fi

  kill -INT "${pid}" 2>/dev/null || true
  for _ in {1..20}; do
    kill -0 "${pid}" 2>/dev/null || break
    sleep 0.25
  done

  if kill -0 "${pid}" 2>/dev/null; then
    echo "cleanup: ${label} did not exit after SIGINT; sending SIGTERM" >&2
    pkill -TERM -P "${pid}" 2>/dev/null || true
    kill -TERM "${pid}" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 0.25
    done
  fi

  if kill -0 "${pid}" 2>/dev/null; then
    echo "cleanup: ${label} did not exit after SIGTERM; sending SIGKILL" >&2
    pkill -KILL -P "${pid}" 2>/dev/null || true
    kill -KILL "${pid}" 2>/dev/null || true
  fi

  wait "${pid}" 2>/dev/null || true
}

cleanup() {
  stop_pid "${REC_PID}" "ros2 bag record"
  stop_pid "${LAUNCH_PID}" "rtabmap launch"
}
trap cleanup EXIT

ros2 launch hno_rtabmap_replay rtabmap_case009.launch.py \
  > "${LOG_DIR}/04_run_rtabmap_launch.log" 2>&1 &
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
    "Rtabmap/CreateIntermediateNodes" \
    "Rtabmap/DetectionRate" \
    "RGBD/LinearUpdate" \
    "RGBD/AngularUpdate" \
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
} > "${LOG_DIR}/rtabmap_relevant_params.txt"

ros2 param dump /rtabmap/rtabmap > "${LOG_DIR}/rtabmap_params_dump.yaml" 2>"${LOG_DIR}/04_rtabmap_param_dump.err" || true
ros2 param dump /rtabmap/stereo_sync > "${LOG_DIR}/stereo_sync_params_dump.yaml" 2>"${LOG_DIR}/04_stereo_sync_param_dump.err" || true

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
  > "${LOG_DIR}/04_run_rtabmap_record.log" 2>&1 &
REC_PID=$!
sleep 2

set +e
timeout --foreground "${BAG_PLAY_TIMEOUT_SEC:-240}" ros2 bag play "${INPUT_BAG}" --read-ahead-queue-size 1000 < /dev/null \
  2>&1 | tee "${LOG_DIR}/04_run_rtabmap_play.log"
PLAY_STATUS="${PIPESTATUS[0]}"
set -e
if [[ "${PLAY_STATUS}" != "0" && "${PLAY_STATUS}" != "124" ]]; then
  echo "ros2 bag play failed with status ${PLAY_STATUS}" >&2
  exit "${PLAY_STATUS}"
fi
if [[ "${PLAY_STATUS}" == "124" ]]; then
  echo "ros2 bag play reached BAG_PLAY_TIMEOUT_SEC=${BAG_PLAY_TIMEOUT_SEC:-240}; continuing cleanup" | tee -a "${LOG_DIR}/04_run_rtabmap_play.log"
fi

sleep 8
cleanup
trap - EXIT

ros2 bag info "${OUTPUT_BAG}" | tee "${LOG_DIR}/output_bag_info.txt"
echo "${RTABMAP_DB}" > "${LOG_DIR}/rtabmap_db_path.txt"

if [[ ! -d "${OUTPUT_BAG}" ]]; then
  echo "missing output bag: ${OUTPUT_BAG}" >&2
  exit 1
fi
if [[ ! -s "${RTABMAP_DB}" ]]; then
  echo "missing or empty RTAB-Map DB: ${RTABMAP_DB}" >&2
  exit 1
fi
if ! grep -q "/rtabmap/info" "${LOG_DIR}/output_bag_info.txt"; then
  {
    echo "no /rtabmap/info in output bag"
    echo
    ros2 bag info "${INPUT_BAG}" || true
    echo
    tail -200 "${LOG_DIR}/04_run_rtabmap_launch.log" || true
  } > "${LOG_DIR}/rtabmap_failure_debug.txt"
  exit 1
fi
