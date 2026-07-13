# HNO-VIO Saved Odom + RTAB-Map Offline Plan

## 1. Current Goal

This document records the current offline RTAB-Map pipeline for HNO-VIO runs saved under `src/hno_vio/results/run_*/`.

The goal is:

```text
HNO-VIO saved odom_raw.csv / odom_raw.txt
+ run_context.json
+ EuRoC stereo images and calibration
+ RTAB-Map ROS 2 graph backend
-> replay stereo + HNO odometry into ROS 2
-> build RTAB-Map visual graph constraints
-> export graph-final optimized trajectory
-> evaluate raw vs optimized against EuRoC ground truth with evo
```

This is an offline integration and evaluation pipeline. It is not a repeatability study of HNO-VIO itself.

## 2. HNO-VIO Run Inputs

The current `roslaunch hno_vio euroc_hno.launch export_odom:=true` output layout is:

```text
src/hno_vio/results/run_YYYYmmddTHHMMSS/
  run_context.json
  vio_results/
    odom_raw.csv
    odom_raw.txt
```

Run IDs use Beijing time / UTC+8:

```text
run_YYYYmmddTHHMMSS
```

This avoids Docker UTC time causing an 8-hour offset in directory names.

### `run_context.json`

`run_context.json` is the source of dataset and frame context:

```json
{
  "dataset": "V1_01_easy",
  "euroc_mav0": "/home/sharpa/datasets/euroc/ASL/V1_01_easy/mav0",
  "ground_truth_tum": "/home/sharpa/hno_vio_clean/src/hno_vio/ground_truth/euroc_mav/V1_01_easy.txt",
  "odom_csv": "vio_results/odom_raw.csv",
  "odom_tum": "vio_results/odom_raw.txt",
  "odom_frame": "odom",
  "base_frame": "base_link",
  "camera_left_frame": "cam0_rect",
  "camera_right_frame": "cam1_rect",
  "odom_semantic": "T_odom_base"
}
```

`run_rtabmap.sh` requires these fields:

```text
dataset
euroc_mav0
ground_truth_tum
odom_csv
odom_tum
odom_frame
base_frame
```

### Odometry Files

`odom_raw.csv` format:

```csv
timestamp_ns,tx,ty,tz,qx,qy,qz,qw
```

Semantic:

```text
timestamp_ns: EuRoC absolute time in nanoseconds
tx,ty,tz: base_link position in odom
qx,qy,qz,qw: ROS quaternion order
pose: T_odom_base
```

`odom_raw.txt` format:

```text
timestamp tx ty tz qx qy qz qw
```

This is evo/TUM-compatible and is also used by the RTAB-Map replay/evaluation scripts.

## 3. Main Command

From workspace root:

```bash
cd /home/sharpa/hno_vio_clean
src/hno_vio/tools/run_rtabmap/scripts/run_rtabmap.sh \
  src/hno_vio/results/run_YYYYmmddTHHMMSS/vio_results/odom_raw.csv
```

The script also accepts the `vio_results/` directory:

```bash
src/hno_vio/tools/run_rtabmap/scripts/run_rtabmap.sh \
  src/hno_vio/results/run_YYYYmmddTHHMMSS/vio_results
```

Use `--skip-build` only when the ROS 2 workspace has already been built:

```bash
src/hno_vio/tools/run_rtabmap/scripts/run_rtabmap.sh --skip-build \
  src/hno_vio/results/run_YYYYmmddTHHMMSS/vio_results/odom_raw.csv
```

Important: the script recreates the target `offline_results/` directory:

```bash
rm -rf "${OUT_DIR}"
mkdir -p "${LOG_DIR}"
```

Manual deletion is not required before rerunning.

## 4. Runtime Pipeline

`run_rtabmap.sh` executes:

```text
00_check_inputs.sh
01_prepare_replay.sh
02_build_ros2_ws.sh       skipped with --skip-build
03_record_input_bag.sh
04_run_rtabmap.sh
05_export_and_eval.sh
```

### Step 00: Input Checks

Validates:

```text
run_context.json
odom_raw.csv / odom_raw.txt
EuRoC ASL mav0 images and calibration
ground truth TUM
ROS 2 / RTAB-Map environment
evo_ape / evo_rpe / evo_traj
Python dependencies
```

Output:

```text
offline_results/logs/input_check.txt
```

### Step 01: Replay Preparation

`01_prepare_replay.sh` validates odometry timing and motion continuity:

```text
row count
timestamp range
mean Hz
overlap with cam0/data.csv
quaternion norm failures
NaN/Inf count
max translation/rotation step
```

Output:

```text
offline_results/logs/odom_check.txt
offline_results/logs/01_prepare_replay.log
```

### Step 02: ROS 2 Workspace Build

Builds:

```text
src/hno_vio/tools/run_rtabmap/ros2_ws
```

Output:

```text
offline_results/logs/02_build_ros2_ws.log
```

### Step 03: Input Bag Recording

The replay node publishes:

```text
/cam0/image_rect
/cam1/image_rect
/cam0/camera_info
/cam1/camera_info
/hno_vio/odom
/tf
/tf_static
/clock
```

Frame convention:

```text
odom -> base_link       replayed HNO odometry
base_link -> cam0_rect  replay static TF
base_link -> cam1_rect  replay static TF
```

The input bag records:

```text
/cam0/image_rect
/cam1/image_rect
/cam0/camera_info
/cam1/camera_info
/hno_vio/odom
/tf
/tf_static
/clock
```

Output:

```text
offline_results/rtabmap_input.bag/
offline_results/logs/03_record_input_bag_record.log
offline_results/logs/03_record_input_bag_replay.log
offline_results/logs/input_bag_info.txt
```

### Step 04: RTAB-Map Run

Launches:

```text
rtabmap_sync/stereo_sync
rtabmap_slam/rtabmap
```

RTAB-Map consumes stereo images and TF odometry:

```text
/cam0/image_rect + /cam1/image_rect + CameraInfo
  -> /rtabmap/rgbd_image
  -> rtabmap_slam/rtabmap

TF odom -> base_link
  -> external odometry source for RTAB-Map
```

`/hno_vio/odom` is recorded for export/debugging. The primary RTAB-Map odometry source is TF `odom -> base_link`.

Output bag records:

```text
/tf
/tf_static
/hno_vio/odom
/rtabmap/rgbd_image
/rtabmap/mapData
/rtabmap/info
/rtabmap/global_path
/rtabmap/local_path
```

Output:

```text
offline_results/rtabmap_output.bag/
offline_results/rtabmap.db
offline_results/logs/output_bag_info.txt
offline_results/logs/rtabmap_db_path.txt
offline_results/logs/rtabmap_relevant_params.txt
offline_results/logs/rtabmap_params_dump.yaml
offline_results/logs/stereo_sync_params_dump.yaml
```

### Step 05: Export and Evaluation

Exports optimized odometry from:

```text
/rtabmap/mapData.graph.poses
```

Then evaluates raw and optimized trajectories against `ground_truth_tum`.

Output:

```text
offline_results/odom_optimized.txt
offline_results/summary.md
offline_results/evo_ape_raw.txt
offline_results/evo_rpe_raw.txt
offline_results/evo_ape_optimized.txt
offline_results/evo_rpe_optimized.txt
offline_results/evo_ape_optimized.pdf
offline_results/evo_rpe_trans_optimized.pdf
offline_results/evo_rpe_rot_optimized.pdf
offline_results/evo_traj_gt_raw_optimized.pdf
offline_results/logs/rpy_raw_vs_optimized.csv
offline_results/logs/rpy_raw_vs_optimized.pdf
offline_results/logs/traj_raw_vs_optimized_matplotlib.pdf
```

## 5. Output Directory Contract

Final layout:

```text
results/run_YYYYmmddTHHMMSS/offline_results/
  rtabmap_input.bag/
  rtabmap_output.bag/
  rtabmap.db

  odom_optimized.txt
  summary.md

  evo_ape_raw.txt
  evo_rpe_raw.txt
  evo_ape_optimized.txt
  evo_rpe_optimized.txt

  evo_ape_optimized.pdf
  evo_rpe_trans_optimized.pdf
  evo_rpe_rot_optimized.pdf
  evo_traj_gt_raw_optimized.pdf

  logs/
    run.log
    input_check.txt
    odom_check.txt
    export_report.txt
    map_odom_stats.txt
    rtabmap_graph_stats.txt
    topic_hz_stats.txt
    rpy_raw_vs_optimized.csv
    rpy_raw_vs_optimized.pdf
    traj_raw_vs_optimized_matplotlib.pdf
```

`rtabmap_input.bag/` and `rtabmap_output.bag/` are ignored by Git.

## 6. Evaluation Semantics

The evo calls use:

```text
--align --correct_scale
```

This is a Sim(3)-aligned shape comparison. Do not describe it as metric-scale drift validation.

RPE uses:

```text
evo_rpe ... -d 20 -u f
```

At approximately 20 Hz this is about 1 second and is reported as `@20 frames (~1s)`.

Reported metrics:

```text
ATE RMSE / mean / median
RPE trans RMSE @20 frames
RPE rot RMSE @20 frames
path length
pose count
duration
raw-vs-optimized position delta
optimized graph pose count
```

## 7. Visualization Rules

### `evo_traj_gt_raw_optimized.pdf`

Generated by:

```text
evo_traj --plot_mode xyz
```

It includes evo's xyz component pages and evo's own RPY page. Evo wraps RPY at `+/-180 deg`; this page is not branch-safe.

Use it when xyz component plots are needed.

### `logs/rpy_raw_vs_optimized.pdf`

This is the branch-safe RPY plot generated by local code.

The local RPY process:

```text
1. convert quaternions to roll/pitch/yaw
2. unwrap each curve over time
3. place optimized and GT on the nearest 360-degree branch of raw at matched timestamps
4. write both CSV and PDF
```

Use this file when discussing roll/pitch/yaw behavior.

### `logs/traj_raw_vs_optimized_matplotlib.pdf`

This is a local XY trajectory plot using the same Sim(3)-style alignment idea for visual comparison.

## 8. RTAB-Map Parameters

Current important RTAB-Map settings:

```text
use_sim_time=false
frame_id=base_link
odom_frame_id=odom
map_frame_id=map
subscribe_rgbd=true
subscribe_stereo=false
subscribe_odom=true
wait_for_transform=1.0
Mem/IncrementalMemory=true
Mem/InitWMWithAllNodes=false
Reg/Force3DoF=false
RGBD/OptimizeFromGraphEnd=true
RGBD/NeighborLinkRefining=true
Vis/MinInliers=12
Kp/MaxFeatures=800
```

Current density tuning defaults:

```text
RTABMAP_DETECTION_RATE=5
RTABMAP_LINEAR_UPDATE=0.03
RTABMAP_ANGULAR_UPDATE=0.03
RTABMAP_CREATE_INTERMEDIATE_NODES=true
```

These are mapped to:

```text
Rtabmap/DetectionRate
RGBD/LinearUpdate
RGBD/AngularUpdate
Rtabmap/CreateIntermediateNodes
```

Reason: `odom_optimized.txt` is exported from RTAB-Map graph poses. If the graph is too sparse, the optimized curve appears piecewise-linear, especially where corrections are large.

Override per run:

```bash
RTABMAP_DETECTION_RATE=10 RTABMAP_LINEAR_UPDATE=0.02 \
RTABMAP_ANGULAR_UPDATE=0.02 \
src/hno_vio/tools/run_rtabmap/scripts/run_rtabmap.sh \
  src/hno_vio/results/run_YYYYmmddTHHMMSS/vio_results/odom_raw.csv
```

Check density in:

```text
offline_results/logs/export_report.txt
```

Important fields:

```text
graph_final_pose_count
graph_final_duration_sec
graph_final_mean_hz
graph_final_median_dt_sec
graph_final_max_dt_sec
graph_final_median_step_m
graph_final_max_step_m
```

If `graph_final_mean_hz` is still low or `graph_final_max_step_m` is large, raise `RTABMAP_DETECTION_RATE` or lower `RTABMAP_LINEAR_UPDATE` / `RTABMAP_ANGULAR_UPDATE`.

## 9. Current Example Run

Recent run:

```text
src/hno_vio/results/run_20260625T211722
```

Context:

```text
dataset: V1_01_easy
euroc_mav0: /home/sharpa/datasets/euroc/ASL/V1_01_easy/mav0
ground_truth_tum: /home/sharpa/hno_vio_clean/src/hno_vio/ground_truth/euroc_mav/V1_01_easy.txt
odom_frame: odom
base_frame: base_link
odom_semantic: T_odom_base
```

Before density tuning, the optimized graph was sparse:

```text
raw poses:       2362 over 142.8 s, about 16.5 Hz
optimized poses: 115 over 142.0 s, about 0.8 Hz
optimized median step: about 0.44 m
optimized max step:    about 1.27 m
```

Interpretation:

```text
The piecewise-linear optimized curve was mainly caused by sparse RTAB-Map graph poses, not by the plotting layer.
```

Rerun the same run after density tuning:

```bash
cd /home/sharpa/hno_vio_clean
src/hno_vio/tools/run_rtabmap/scripts/run_rtabmap.sh \
  src/hno_vio/results/run_20260625T211722/vio_results/odom_raw.csv
```

This automatically deletes and recreates:

```text
src/hno_vio/results/run_20260625T211722/offline_results/
```

## 10. Reporting Rules

Use this wording:

```text
The RTAB-Map optimized trajectory is exported from the final optimized graph in /rtabmap/mapData.graph.poses. The result is evaluated against EuRoC ground truth after Sim(3) alignment with evo_ape/evo_rpe.
```

When discussing RPY:

```text
Use logs/rpy_raw_vs_optimized.pdf, not the RPY page inside evo_traj_gt_raw_optimized.pdf, because evo_traj wraps angles at +/-180 degrees.
```

When discussing curve smoothness:

```text
The optimized trajectory density is controlled by RTAB-Map graph creation frequency. Inspect graph_final_mean_hz and graph_final_max_step_m in export_report.txt before interpreting sharp segments as physical motion.
```
