# HNO-VIO ROS2 Humble Adaptation Plan

## Summary

- Refactor `/home/sharpa/hno_vio_clean` into a single ROS2 Humble workspace built with one `colcon build`.
- `hno_vio.launch.py` is the VIO front-end entry point. Launching it always runs HNO-VIO; there is no `run_hno` switch.
- When `run_preprocess:=true`, also start RTAB preprocess and `ros2 bag record` to generate `vio_results/rtabmap_input_db3`.
- Use `src/hno_vio/tools/run_rtabmap/hno_rtabmap.sh` as the only RTAB-Map backend entry point. Do not add `hno_rtabmap.launch.py`.
- Preserve the tuned RViz layout at `src/hno_vio/launch/hno.rviz`.

## Target Layout

```text
/home/sharpa/hno_vio_clean/
├── src/
│   └── hno_vio/
│       ├── src/
│       ├── include/hno_vio/
│       ├── launch/
│       │   ├── hno_vio.launch.py
│       │   └── hno.rviz
│       ├── config/
│       ├── ground_truth/
│       ├── tools/
│       │   ├── run_vio/
│       │   └── run_rtabmap/
│       │       ├── hno_rtabmap.sh
│       │       ├── rtabmap_preprocess.py
│       │       ├── export_optimized_odom.py
│       │       └── eval_and_summary.py
│       └── results/
│           └── run_YYYYmmddTHHMMSS/
│               ├── run_context.json
│               ├── vio_results/
│               │   ├── odom_raw.csv
│               │   ├── odom_raw.txt
│               │   └── rtabmap_input_db3/
│               └── offline_results/
│                   ├── rtabmap_output.bag/
│                   ├── rtabmap.db
│                   ├── odom_optimized.txt
│                   ├── summary.md
│                   └── logs/
├── build/
├── install/
└── log/
```

## ROS2 Migration

- Convert `package.xml` and `CMakeLists.txt` from catkin/ROS1 to `ament_cmake`/ROS2.
- Use dependencies: `rclcpp`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `tf2_ros`, `cv_bridge`, `message_filters`, OpenCV, Eigen, Boost.
- Compile with `ROS_AVAILABLE=2`.
- Replace ROS1 APIs in `HNOManager`:
  - `ros::NodeHandle` -> `rclcpp::Node`
  - `ros::Publisher` / `ros::Subscriber` -> ROS2 publishers/subscriptions
  - `tf::TransformBroadcaster` -> `tf2_ros::TransformBroadcaster`
  - ROS params/logging -> ROS2 params/logging
- Keep algorithm core behavior stable: `HNOPropagator`, `HNOUpdater`, `HNOFeature`, `HNOInitializer`.
- Remove `use_gt_init`.
- Keep `use_gt_mapping` for Wang 2022 GT mapping comparison.
- Keep `num_cams` / `max_cameras` support for mono/stereo dynamic switching.
- Remove ASL runtime dependency: no `euroc_mav0`, no ASL `sensor.yaml`.
- HNO initialization must use config/ROS parameters:
  - `init_imu_thresh`, default from `estimator_config.yaml` is `1.5`
  - `init_gyro_thresh`, default `0.01`
  - `init_window_size`, default `250`
  - Do not keep the old hard-coded accelerometer threshold `0.05`, because it can block initialization on EuRoC.

## `hno_vio.launch.py`

- Launching this file always runs HNO-VIO.
- Parameters:

```text
dataset:=V1_01_easy
bag_path:=/home/sharpa/datasets/euroc/ros2db/V1_01_easy_db
play_bag:=true
bag_rate:=1.0
bag_start:=0.0
play_topics:=/imu0 /cam0/image_raw /cam1/image_raw

config:=euroc_mav
config_path:=<hno_vio share>/config/euroc_mav/estimator_config.yaml
camera_config:=<hno_vio share>/config/euroc_mav/kalibr_imucam_chain.yaml
path_gt:=<hno_vio share>/ground_truth/euroc_mav/V1_01_easy.txt

max_cameras:=2
num_cams:=2
use_gt_mapping:=false
export_odom:=true
odom_output_path:=<hno_vio source>/results/{run_id}/vio_results/odom_raw.csv

run_preprocess:=false
rviz:=true
use_sim_time:=true
```

- If `play_bag=true`, start `ros2 bag play bag_path --clock --rate bag_rate --start-offset bag_start`.
- `play_topics` defaults to the three HNO input topics to avoid unrelated converted topics or missing custom message types affecting playback.
- Always start `run_hno_vio`.
- Always export `odom_raw.csv` and `odom_raw.txt` when `export_odom=true`.
- If `rviz=true`, load `src/hno_vio/launch/hno.rviz`.
- `hno.rviz` must use RViz2 plugin class names and ROS2 topics such as `/hno_vio/path`, `/hno_vio/path_gt`, `/hno_vio/features_3d`, and `/hno_vio/image_track`.
- If `run_preprocess=true`, also start `rtabmap_preprocess` and `ros2 bag record`.
- `run_preprocess=true` requires stereo input; if `num_cams != 2`, fail clearly because RTAB stereo input cannot be generated.

## Preprocess Timing

- `rtabmap_preprocess` does not depend on odometry; it only converts raw stereo images into rectified stereo + camera_info + static TF.
- It can run concurrently with HNO-VIO without requiring artificial delay.
- To avoid startup races, launch order should be:
  - start HNO-VIO
  - start preprocess
  - start recorder
  - start bag playback after a short startup delay if `play_bag=true`
- `ros2 bag record` records topics by timestamp; RTAB-Map later consumes `rtabmap_input_db3`, so exact wall-clock arrival order during recording is less important than correct message stamps.
- If machine load causes dropped frames, lower `bag_rate`; do not make preprocess intentionally slower than odom.

## `rtabmap_preprocess.py`

- Source path:

```text
src/hno_vio/tools/run_rtabmap/rtabmap_preprocess.py
```

- Install as ROS2 executable:

```bash
ros2 run hno_vio rtabmap_preprocess
```

- It should read calibration from:

```text
src/hno_vio/config/euroc_mav/kalibr_imucam_chain.yaml
```

or from the launch parameter:

```text
camera_config:=<hno_vio share>/config/$(config)/kalibr_imucam_chain.yaml
```

- It subscribes:

```text
/cam0/image_raw
/cam1/image_raw
```

- It publishes:

```text
/cam0/image_rect
/cam1/image_rect
/cam0/camera_info
/cam1/camera_info
/tf_static
```

- It parses:

```text
cam0.intrinsics
cam0.distortion_coeffs
cam0.resolution
cam0.T_imu_cam
cam1.intrinsics
cam1.distortion_coeffs
cam1.resolution
cam1.T_imu_cam
```

- It reuses the current `replay_node.py` rectification logic conceptually:
  - `cv2.stereoRectify`
  - `cv2.initUndistortRectifyMap`
  - `cv2.remap`
  - rectified `P1/P2`
  - baseline sanity check
- It must not implement replay:
  - no ASL image reading
  - no odom CSV reading
  - no `/hno_vio/odom` publishing
  - no `/clock` publishing
  - no playback rate control

## Raw Bag Topics

- Expected converted ROS2 bag topics include:

```text
/cam0/image_raw
/cam1/image_raw
/events/read_split
/fcu/imu
/imu0
/parameter_events
/rosout
/vicon/firefly_sbx/firefly_sbx
```

- HNO-VIO uses:

```text
/imu0
/cam0/image_raw
/cam1/image_raw
```

- Other topics are ignored unless explicitly used later.
- If `ros2 bag info` fails with `yaml-cpp: bad conversion` at `offered_qos_profiles: []`, patch the converted bag metadata so empty QoS profiles are written as an empty string:

```text
offered_qos_profiles: ''
```

  Keep a backup of the original `metadata.yaml`.
- `hno_vio.launch.py` performs this metadata normalization automatically before `ros2 bag play`, so manually converted rosbags do not need to be edited by hand each time.
- RTAB input recording should include only:

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

## `run_context.json`

- Required fields:

```json
{
  "dataset": "V1_01_easy",
  "raw_bag": "/home/sharpa/datasets/euroc/ros2db/V1_01_easy_db",
  "config": "euroc_mav",
  "config_path": "config/euroc_mav/estimator_config.yaml",
  "camera_config": "config/euroc_mav/kalibr_imucam_chain.yaml",
  "ground_truth_tum": "ground_truth/euroc_mav/V1_01_easy.txt",
  "odom_csv": "vio_results/odom_raw.csv",
  "odom_tum": "vio_results/odom_raw.txt",
  "rtabmap_input_bag": "vio_results/rtabmap_input_db3",
  "odom_frame": "odom",
  "base_frame": "base_link",
  "camera_left_frame": "cam0_rect",
  "camera_right_frame": "cam1_rect",
  "odom_semantic": "T_odom_base",
  "num_cams": 2,
  "use_gt_mapping": false
}
```

## RTAB-Map Backend

- Remove nested workspace:

```text
src/hno_vio/tools/run_rtabmap/ros2_ws
```

- Use:

```text
src/hno_vio/tools/run_rtabmap/hno_rtabmap.sh
```

- Input:

```bash
src/hno_vio/tools/run_rtabmap/hno_rtabmap.sh \
  src/hno_vio/results/run_YYYYmmddTHHMMSS/vio_results/rtabmap_input_db3
```

- If `rtabmap_input_db3` is missing, fail clearly and tell the user to rerun:

```bash
ros2 launch hno_vio hno_vio.launch.py run_preprocess:=true
```

- Derive output paths automatically:

```text
run_dir = input_bag/../..
offline_results = run_dir/offline_results
rtabmap_db = offline_results/rtabmap.db
rtabmap_output_bag = offline_results/rtabmap_output.bag
```

- Runtime steps:
  - source `/home/sharpa/hno_vio_clean/install/setup.bash`
  - clean/create `offline_results`
  - start `rtabmap_sync/stereo_sync`
  - start `rtabmap_slam/rtabmap` with `database_path:=offline_results/rtabmap.db`
  - start `ros2 bag record`
  - play `rtabmap_input_db3`
  - stop recorder and RTAB nodes after playback
  - run export/eval scripts
- Keep `rtabmap.db` as output for `rtabmap-databaseViewer` / graph debugging.

## Optimized Odometry Export

- `export_optimized_odom.py` must export optimized trajectory from:

```text
/rtabmap/mapData.graph.poses
```

- Do not export from a less-correct pose source if `graph.poses` is available.
- Output:

```text
offline_results/odom_optimized.txt
```

- Format should be TUM-compatible:

```text
timestamp tx ty tz qx qy qz qw
```

## Evaluation

- `eval_and_summary.py` should compare:

```text
vio_results/odom_raw.txt
offline_results/odom_optimized.txt
ground_truth_tum from run_context.json
```

- Output:

```text
offline_results/summary.md
offline_results/logs/
```

- Keep evaluation focused on evo and existing RTAB diagnostics.
- Do not add extra monitoring/analysis frameworks beyond the required eval/summary.

## `tools/run_vio`

- If existing smoke scripts cannot be used directly under ROS2, adapt them only enough for ROS2 build/run path compatibility.
- Copy existing `/home/sharpa/hno_vio_clean/src/hno_vio/tools/run_vio` behavior as-is.
- Current target is compile success and manual runtime testing by the user.
- Do not add additional analysis or monitoring scripts.

## Test Plan

- Build:

```bash
cd /home/sharpa/hno_vio_clean
colcon build --packages-select hno_vio
source install/setup.bash
```

- Raw bag check:

```bash
ros2 bag info /home/sharpa/datasets/euroc/ros2db/V1_01_easy_db
```

- VIO smoke:

```bash
ros2 launch hno_vio hno_vio.launch.py \
  dataset:=V1_01_easy \
  bag_path:=/home/sharpa/datasets/euroc/ros2db/V1_01_easy_db \
  play_bag:=true \
  run_preprocess:=false
```

Verify:

```text
/hno_vio/odom publishes
results/run_*/vio_results/odom_raw.csv exists
results/run_*/vio_results/odom_raw.txt exists
```

- VIO + RTAB input generation:

```bash
ros2 launch hno_vio hno_vio.launch.py \
  dataset:=V1_01_easy \
  bag_path:=/home/sharpa/datasets/euroc/ros2db/V1_01_easy_db \
  play_bag:=true \
  run_preprocess:=true
```

Verify:

```text
results/run_*/vio_results/rtabmap_input_db3/metadata.yaml
/cam0/image_rect
/cam1/image_rect
/cam0/camera_info
/cam1/camera_info
/hno_vio/odom
/tf
/tf_static
/clock
```

- RTAB backend:

```bash
src/hno_vio/tools/run_rtabmap/hno_rtabmap.sh \
  src/hno_vio/results/run_*/vio_results/rtabmap_input_db3
```

Verify:

```text
offline_results/rtabmap_output.bag
offline_results/rtabmap.db
offline_results/odom_optimized.txt
offline_results/summary.md
```

## Assumptions

- Converted ROS2 raw bags are manually maintained under `/home/sharpa/datasets/euroc/ros2db/`.
- `hno_vio.launch.py` always runs HNO-VIO.
- `run_preprocess:=true` means preprocess and RTAB input recording are both enabled.
- `hno_rtabmap.sh` does not regenerate missing `rtabmap_input_db3`.
- ASL `mav0` folders and `sensor.yaml` are no longer part of the ROS2 runtime path.
- `use_gt_mapping` remains for Wang 2022 comparison; `use_gt_init` is removed.
