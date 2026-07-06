#!/usr/bin/env bash
#
# 用法（在 hno_vio 包目录执行）：
#   bash tools/run_rtabmap/hno_rtabmap.sh \
#     results/run_YYYYmmddTHHMMSS/vio_results/rtabmap_input_db3
#
# 脚本会自动启动双目同步和 RTAB-Map、完整回放输入 bag、记录输出、
# 导出优化轨迹并运行 evo 评估。执行期间无需按键，看到 completed: 即完成。
set -euo pipefail

# 检查命令行输入，并定位工作空间、运行目录和输出目录。
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

# 加载 ROS2、rtabmap_ros 和当前 HNO-VIO 工作空间环境。
source_if_exists() {
  local file="$1"
  if [[ -s "${file}" ]]; then
    # shellcheck disable=SC1090
    source "${file}"
  fi
}

set +u
source_if_exists /opt/ros/humble/setup.bash
source_if_exists /home/sharpa/ros2_ws/install/local_setup.bash
source_if_exists "${WORKSPACE_ROOT}/install/local_setup.bash"
set -u

# The locally built rtabmap_ros links against the RTAB-Map core build tree.
RTABMAP_CORE_LIB_DIR="${RTABMAP_CORE_LIB_DIR:-/home/sharpa/rtabmap/build/bin}"
if [[ -d "${RTABMAP_CORE_LIB_DIR}" ]]; then
  export LD_LIBRARY_PATH="${RTABMAP_CORE_LIB_DIR}:${LD_LIBRARY_PATH:-}"
fi

export ROS_LOG_DIR="${LOG_DIR}/ros"
mkdir -p "${ROS_LOG_DIR}"

PIDS=()
# 打印全流程阶段进度。
stage() {
  echo "[$1/6] $2" | tee -a "${LOG_DIR}/run.log"
}

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "required command not found: $1" >&2
    exit 1
  fi
}

require_package() {
  if ! ros2 pkg prefix "$1" >/dev/null 2>&1; then
    echo "required ROS2 package not found: $1" >&2
    exit 1
  fi
}

# 确认后台节点没有在启动后立即退出。
assert_alive() {
  local label="$1"
  local pid="$2"
  local log_file="$3"
  if ! kill -0 "${pid}" 2>/dev/null; then
    echo "${label} failed to start. Last log lines:" >&2
    tail -40 "${log_file}" >&2 || true
    exit 1
  fi
}

# 有界关闭所有后台进程，避免录包或 ROS 节点阻塞脚本退出。
cleanup() {
  local deadline
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -INT "${pid}" 2>/dev/null || true
    fi
  done
  deadline=$((SECONDS + 10))
  while (( SECONDS < deadline )); do
    local running=false
    for pid in "${PIDS[@]:-}"; do
      if kill -0 "${pid}" 2>/dev/null; then
        running=true
        break
      fi
    done
    if [[ "${running}" == false ]]; then
      break
    fi
    sleep 0.2
  done
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
  sleep 1
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -KILL "${pid}" 2>/dev/null || true
    fi
    wait "${pid}" 2>/dev/null || true
  done
}
trap cleanup EXIT

echo "hno_rtabmap start: $(date -Is)" | tee "${LOG_DIR}/run.log"
echo "input_bag=${INPUT_BAG}" | tee -a "${LOG_DIR}/run.log"
echo "offline_dir=${OFFLINE_DIR}" | tee -a "${LOG_DIR}/run.log"

require_command ros2
require_command evo_ape
require_command evo_res
require_command evo_traj
require_package rtabmap_sync
require_package rtabmap_slam
require_package rtabmap_msgs

# 将左右相机消息同步并合成为 RTAB-Map 的 RGBDImage 输入。
stage 1 "starting stereo synchronization"
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
STEREO_SYNC_PID="$!"
sleep 1
assert_alive "stereo_sync" "${STEREO_SYNC_PID}" "${LOG_DIR}/stereo_sync.log"

# 启动 RTAB-Map 后端，并显式统一输出 topic 的命名空间。
stage 2 "starting RTAB-Map"
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
  -p 'Mem/IncrementalMemory:="true"' \
  -p 'Mem/InitWMWithAllNodes:="false"' \
  -p 'Reg/Force3DoF:="false"' \
  -p 'Rtabmap/CreateIntermediateNodes:="true"' \
  -p 'Rtabmap/DetectionRate:="5"' \
  -p 'RGBD/LinearUpdate:="0.03"' \
  -p 'RGBD/AngularUpdate:="0.03"' \
  -p 'RGBD/OptimizeFromGraphEnd:="true"' \
  -p 'RGBD/NeighborLinkRefining:="true"' \
  -p 'Vis/MinInliers:="12"' \
  -p 'Kp/MaxFeatures:="800"' \
  -r rgbd_image:=/rtabmap/rgbd_image \
  -r mapData:=/rtabmap/mapData \
  -r info:=/rtabmap/info \
  -r global_path:=/rtabmap/global_path \
  -r local_path:=/rtabmap/local_path \
  > "${LOG_DIR}/rtabmap.log" 2>&1 &
PIDS+=("$!")
RTABMAP_PID="$!"
sleep 2
assert_alive "rtabmap" "${RTABMAP_PID}" "${LOG_DIR}/rtabmap.log"

# 记录原始里程计、同步图像、优化图和 TF，供后续离线导出。
stage 3 "recording RTAB-Map outputs"
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
RECORDER_PID="$!"
sleep 2
assert_alive "ros2 bag record" "${RECORDER_PID}" "${LOG_DIR}/record_output.log"

# 输入 bag 已含 /clock；完整回放一次，不额外生成第二路模拟时钟。
stage 4 "playing the complete input bag; no keyboard operation is required"
ros2 bag play "${INPUT_BAG}" </dev/null 2>&1 | tee "${LOG_DIR}/play_input.log"
assert_alive "rtabmap after playback" "${RTABMAP_PID}" "${LOG_DIR}/rtabmap.log"

# 请求发布最终优化图，确保输出 bag 中包含最后一帧 MapData。
if ! timeout 20s ros2 service call \
  /rtabmap/publish_map \
  rtabmap_msgs/srv/PublishMap \
  "{global_map: true, optimized: true, graph_only: false}" \
  > "${LOG_DIR}/publish_final_map.log" 2>&1; then
  echo "failed to request the final optimized map." >&2
  echo "See ${LOG_DIR}/publish_final_map.log" >&2
  exit 1
fi
sleep 3
cleanup
trap - EXIT

# 确认最终优化图已录制，失败时停止后续导出。
ros2 bag info "${OUTPUT_BAG}" > "${LOG_DIR}/rtabmap_output_bag_info.txt" 2>&1 || true
if ! grep -q "/rtabmap/mapData" "${LOG_DIR}/rtabmap_output_bag_info.txt"; then
  echo "RTAB-Map produced no /rtabmap/mapData messages." >&2
  echo "See ${LOG_DIR}/rtabmap.log" >&2
  exit 1
fi

# 从最终 MapData 图节点导出 TUM 格式优化轨迹。
stage 5 "exporting optimized graph poses"
"${SCRIPT_DIR}/export_optimized_odom.py" \
  --bag "${OUTPUT_BAG}" \
  --out "${OFFLINE_DIR}/odom_optimized.txt" \
  2>&1 | tee "${LOG_DIR}/export_optimized_odom.log"

# 运行 APE 对比和三轨迹绘图，结果统一写入 run_dir/evo_results。
stage 6 "running evo evaluation and analysis"
"${SCRIPT_DIR}/eval_and_analysis.py" "${RUN_DIR}"

echo "completed:"
echo "offline results: ${OFFLINE_DIR}"
echo "output bag: ${OUTPUT_BAG}"
echo "rtabmap db: ${RTABMAP_DB}"
echo "optimized odom: ${OFFLINE_DIR}/odom_optimized.txt"
echo "evo results: ${RUN_DIR}/evo_results"
