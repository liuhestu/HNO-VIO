# HNO-VIO 当前结构说明

本文档记录当前仓库结构和核心代码边界，用作后续重构前的现状参考。内容以当前源码为准，只描述已有模块、接口、数据流和生成物，不提出重构方案。

## 1. 系统总结构

当前 `hno_vio` 是一个 ROS2 `ament_cmake` 包，包目录位于 `src/hno_vio/`。工作区根目录下的 `build/`、`install/`、`log/` 等是构建生成目录，不属于源码结构。

### 1.1 顶层文件

- `CMakeLists.txt`：ROS2 构建入口。设置 C++17 和 `ROS_AVAILABLE=2`，查找 `rclcpp`、消息包、`cv_bridge`、`message_filters`、`tf2_ros`、Eigen、OpenCV、Boost、CURL 等依赖。
- `package.xml`：ROS2 package manifest，包名为 `hno_vio`，构建类型为 `ament_cmake`，声明运行时依赖 `ros2bag`、`rosbag2`、`rviz2`、`python3-numpy`、`python3-yaml` 等。
- `README.md`：包级说明文件。

`CMakeLists.txt` 当前构建三个目标：

- `openvins_core_minimal`：由 `thirdparty/openvins_core/src/` 中的最小 OpenVINS 前端组件组成。
- `hno_vio_lib`：由 HNO-VIO 核心实现组成，包括传播、更新、特征、初始化和管理器。
- `run_hno_vio`：ROS2 可执行节点，源码为 `src/run_hno_vio.cpp`。

安装规则会安装 `include/`、`launch/`、`config/`、`ground_truth/`、`tools/`，并把 RTAB-Map 相关 Python 脚本安装为可执行程序。

### 1.2 配置目录

`config/` 分为两个数据源配置目录：

- `config/euroc_mav/`
  - `estimator_config.yaml`：EuRoC 参数文件。包含 OpenVINS 风格参数、IMU 噪声、前端跟踪参数、HNO 特征健康检查参数、HNO 更新保护参数等。当前代码实际读取其中一部分字段。
  - `kalibr_imu_chain.yaml`：IMU 噪声和频率配置。
  - `kalibr_imucam_chain.yaml`：相机内参、畸变、分辨率、`T_imu_cam` 外参。
- `config/realsense/`
  - `estimator_config.yaml`
  - `kalibr_imu_chain.yaml`
  - `kalibr_imucam_chain.yaml`

当前 `HNOManager::load_parameters()` 实际读取的配置主要包括：

- 相机数量：YAML 中的 `max_cameras` 作为配置上限；launch 只暴露 `num_cams` 表示当前输入相机数，`1` 为单目，`2` 为双目。
- 相机内外参：通过 `relative_config_imucam` 指向的 `kalibr_imucam_chain.yaml` 读取每个 `camN` 的 `intrinsics`、`distortion_coeffs`、`resolution`、`T_imu_cam`。
- IMU 噪声：`accelerometer_noise_density`、`gyroscope_noise_density`。
- 初始化参数：`init_imu_thresh`、`init_gyro_thresh`、`init_window_size`。
- 特征参数：`feature_tracker_num_pts`、`feature_tracker_fast_threshold`、`feature_tracker_grid_x`、`feature_tracker_grid_y`、`feature_tracker_min_px_dist`、`feature_min_stereo_depth`、`feature_max_stereo_depth`、`feature_stereo_reproj_thresh`、`feature_reproj_thresh`、`feature_reproj_thresh_low`、`feature_low_feature_pts`、`feature_low_feature_db`、`feature_mature_thresh`、`feature_mature_thresh_low`、`feature_fail_limit`、`feature_fail_limit_low`、`feature_map_jump_thresh`、`feature_active_mature_thresh`、`feature_health_min_stable`、`feature_health_min_db`、`feature_health_hold_frames`、`feature_health_start_frame`。
- 更新参数：`update_pixel_noise`、`update_focal_length`、`update_chi2_gate`、`update_max_delta_p`、`update_max_delta_r`、`update_min_observations`、`update_low_observation_hold_frames`、`update_warn_delta_ratio`、`update_enforce_structure`。

配置文件中存在一些 OpenVINS 或历史字段，例如 `use_gt_init`。当前节点代码没有读取 `use_gt_init` 作为运行参数；当前和 GT 相关、会影响代码路径的开关是 `use_gt_mapping`。

### 1.3 Launch 与 RViz

- `launch/hno_vio.launch.py`：当前主 launch 文件。
- `launch/hno_vio.rviz`：RViz2 显示配置。

`hno_vio.launch.py` 的主要行为：

- 解析 launch 参数并启动 `hno_vio/run_hno_vio`。
- 默认 `dataset=V1_01_easy`，`config=euroc_mav`。
- 当 `config_path` 为空时，默认使用 `share/hno_vio/config/<config>/estimator_config.yaml`。
- 当 `camera_config` 为空时，默认使用 `share/hno_vio/config/<config>/kalibr_imucam_chain.yaml`。
- 当 `path_gt` 为空时，默认使用 `share/hno_vio/ground_truth/euroc_mav/<dataset>.txt`。
- 生成北京时间命名的 `run_YYYYmmddTHHMMSS` 结果目录，并默认把 VIO odom 写到 `vio_results/odom_raw.csv`。
- 可选播放 ROS2 bag：`play_bag=true` 时执行 `ros2 bag play`，默认播放 `/imu0 /cam0/image_raw /cam1/image_raw`。
- 可选启动 RTAB-Map 预处理：`run_preprocess=true` 时启动 `rtabmap_preprocess` 并录制 `vio_results/rtabmap_input_db3`。
- 可选启动 RViz2：`rviz=true` 时加载 `launch/hno_vio.rviz`。

主要 launch 参数包括：

- 数据和路径：`dataset`、`bag_path`、`config`、`config_path`、`camera_config`、`path_gt`、`results_root`。
- bag 播放：`play_bag`、`bag_rate`、`bag_start`、`play_topics`。
- 相机数量：`num_cams`，`1` 为单目，`2` 为双目。
- 功能开关：`use_gt_mapping`、`export_odom`、`run_preprocess`、`rviz`、`use_sim_time`。
- 输出和坐标系：`odom_output_path`、`odom_frame`、`base_frame`。
- 输入 topic：`topic_imu`、`topic_cam0`、`topic_cam1`。

### 1.4 源码目录

`src/` 中的实现文件：

- `src/run_hno_vio.cpp`：ROS2 节点入口。
- `src/HNOManager.cpp`：节点级管理器实现，负责参数、订阅发布、缓存、初始化、传播、视觉更新、可视化、导出和 GT 评估。
- `src/HNOState.cpp`：不存在；`HNOState` 当前是头文件内实现。
- `src/HNOInitializer.cpp`：静态 IMU 初始化。
- `src/HNOPropagator.cpp`：IMU 状态传播和协方差传播。
- `src/HNOFeature.cpp`：KLT 跟踪、双目三角化、局部 3D 点维护、观测生成和健康检查。
- `src/HNOUpdater.cpp`：基于观测的 HNO 序贯更新、卡方门限和更新保护。

`include/hno_vio/` 中的公开头文件：

- `HNOManager.h`
- `HNOState.h`
- `HNOInitializer.h`
- `HNOPropagator.h`
- `HNOFeature.h`
- `HNOUpdater.h`

### 1.5 Thirdparty

`thirdparty/openvins_core/src/` 保存当前依赖的 OpenVINS 最小组件：

- `cam/`：相机模型，例如 `CamBase.h`、`CamRadtan.h`。
- `feat/`：`Feature` 和 `FeatureDatabase`。
- `track/`：`TrackBase`、`TrackKLT`、网格和 FAST 特征工具。
- `utils/`：传感器数据结构、四元数工具、YAML 解析、打印和颜色工具。

当前 HNO-VIO 主要复用 OpenVINS 的相机模型、KLT tracker、YAML parser 和 `ov_core::ImuData`、`ov_core::CameraData` 数据结构。

### 1.6 Tools

`tools/run_vio/`：

- `README.md`：说明当前目录是最小 ROS2 smoke entry point，正常运行使用 `ros2 launch hno_vio hno_vio.launch.py`。
- `run_smoke.sh`：构建 smoke 脚本入口。

`tools/run_rtabmap/`：

- `README.md`：RTAB-Map 后端离线流程说明。
- `hno_rtabmap.sh`：RTAB-Map 后端主入口，输入为某次运行的 `vio_results/rtabmap_input_db3`。
- `rtabmap_preprocess.py`：ROS2 节点，把原始双目图像和 HNO odom 整理成 RTAB-Map 输入所需 topic。
- `export_optimized_odom.py`：从 RTAB-Map 输出 bag 的 `/rtabmap/mapData.graph.poses` 导出优化轨迹。
- `eval_and_analysis.py`：读取 `run_context.json`、raw/optimized odom 和 GT，生成 APE 结果与轨迹对比图。

典型 RTAB-Map 输入输出结构：

```text
results/run_YYYYmmddTHHMMSS/
  run_context.json
  vio_results/
    odom_raw.csv
    odom_raw.txt
    rtabmap_input_db3/
  offline_results/
    rtabmap_output.bag/
    rtabmap.db
    odom_optimized.txt
    logs/
  evo_results/
    ape_raw.zip
    ape_optimized.zip
    ate_plot.pdf
    ate_stats.txt
    traj_trajectories.png
    traj_xyz.png
    traj_rpy.png
    traj_speeds.png
```

### 1.7 Ground Truth、Results 与 Params Eval

- `ground_truth/euroc_mav/`：EuRoC 数据集参考轨迹，包含 CSV 和 TUM TXT 格式文件，例如 `V1_01_easy.txt`、`MH_01_easy.txt` 等。
- `results/`：运行输出目录。每次正常 launch 会生成 `run_YYYYmmddTHHMMSS/`，常见内容包括 `run_context.json`、`vio_results/odom_raw.csv`、`vio_results/odom_raw.txt`，以及可选的 RTAB-Map 输入和离线结果。
- `params_eval/`：历史参数搜索、验证和报告输出目录。它记录已有评估实验和候选参数，不是核心运行时代码路径。
- `docs/`：设计、历史、理论和 RTAB-Map 工作流文档。

## 2. HNO-VIO 核心代码结构

### 2.0 类与函数归属速查

这一节只列当前 HNO-VIO 自有类和入口函数，方便阅读 C++ 代码时先判断函数归属。`src/run_hno_vio.cpp` 中的 `main()` 不是类成员；其余核心逻辑基本都挂在 `hno_vio` namespace 下的类中。

| 文件 | 类 / 结构 | 函数或成员 | 归属说明 |
| --- | --- | --- | --- |
| `src/run_hno_vio.cpp` | 无 | `main()` | ROS2 进程入口，创建 `rclcpp::Node` 和 `HNOManager`。 |
| `include/hno_vio/HNOManager.h` / `src/HNOManager.cpp` | `hno_vio::HNOManager` | `HNOManager::HNOManager()` | 构造管理器，创建状态、传播器、初始化器、更新器、特征模块和发布器。 |
| `include/hno_vio/HNOManager.h` / `src/HNOManager.cpp` | `hno_vio::HNOManager` | `HNOManager::~HNOManager()` | 析构时 flush/close odom 输出文件。 |
| `include/hno_vio/HNOManager.h` / `src/HNOManager.cpp` | `hno_vio::HNOManager` | `HNOManager::launch_subscribers()` | 创建 IMU、单目或双目订阅。 |
| `include/hno_vio/HNOManager.h` / `src/HNOManager.cpp` | `hno_vio::HNOManager` | `HNOManager::feed_measurement(const ov_core::ImuData&)` | IMU 数据入口，负责初始化前缓存和初始化后传播/发布。 |
| `include/hno_vio/HNOManager.h` / `src/HNOManager.cpp` | `hno_vio::HNOManager` | `HNOManager::feed_measurement(const ov_core::CameraData&)` | 相机数据入口，初始化后转入 `process_camera_data()`。 |
| `include/hno_vio/HNOManager.h` / `src/HNOManager.cpp` | `hno_vio::HNOManager` | `HNOManager::load_parameters()` | 读取 YAML 和 ROS2 参数，配置相机、IMU 噪声、初始化、feature、updater、GT 和导出。 |
| `include/hno_vio/HNOManager.h` / `src/HNOManager.cpp` | `hno_vio::HNOManager` | `HNOManager::load_gt_data()` | 读取 TUM/CSV 风格 GT 轨迹到 `gt_states`。 |
| `include/hno_vio/HNOManager.h` / `src/HNOManager.cpp` | `hno_vio::HNOManager` | `HNOManager::process_camera_data()` | 相机帧主处理流程：IMU 补传播、特征观测、状态更新、误差打印、导出和可视化。 |
| `include/hno_vio/HNOManager.h` / `src/HNOManager.cpp` | `hno_vio::HNOManager` | `HNOManager::get_interpolated_gt()` | 按时间插值 GT 位置和姿态。 |
| `include/hno_vio/HNOManager.h` / `src/HNOManager.cpp` | `hno_vio::HNOManager` | `HNOManager::publish_state()` | 发布 pose、odom、path、GT path 和 TF。 |
| `include/hno_vio/HNOManager.h` / `src/HNOManager.cpp` | `hno_vio::HNOManager` | `HNOManager::export_odom_state()` | 输出 `odom_raw.csv` 和 `odom_raw.txt`。 |
| `include/hno_vio/HNOManager.h` / `src/HNOManager.cpp` | `hno_vio::HNOManager` | `HNOManager::make_run_id_beijing_time()` | 生成北京时间 `run_YYYYmmddTHHMMSS`。 |
| `include/hno_vio/HNOManager.h` / `src/HNOManager.cpp` | `hno_vio::HNOManager` | `HNOManager::infer_dataset_from_bag_path()` | 从 bag 路径推断 dataset 名称。 |
| `include/hno_vio/HNOManager.h` / `src/HNOManager.cpp` | `hno_vio::HNOManager` | `HNOManager::json_escape()` | 写 `run_context.json` 前做字符串转义。 |
| `include/hno_vio/HNOManager.h` / `src/HNOManager.cpp` | `hno_vio::HNOManager` | `HNOManager::write_run_context()` | 写 RTAB-Map 离线流程依赖的 `run_context.json`。 |
| `include/hno_vio/HNOManager.h` / `src/HNOManager.cpp` | `hno_vio::HNOManager` | `HNOManager::publish_visualization()` | 发布 3D 特征点和跟踪图像。 |
| `include/hno_vio/HNOManager.h` / `src/HNOManager.cpp` | `hno_vio::HNOManager` | `HNOManager::compute_and_print_error()` | 对齐 GT 后打印位置误差、速度和 `e_hat` 正交性。 |
| `include/hno_vio/HNOManager.h` / `src/HNOManager.cpp` | `hno_vio::HNOManager` | `HNOManager::imu_callback()` | ROS2 IMU 消息回调，转换成 `ov_core::ImuData`。 |
| `include/hno_vio/HNOManager.h` / `src/HNOManager.cpp` | `hno_vio::HNOManager` | `HNOManager::mono_callback()` | ROS2 单目图像回调，转换成 `ov_core::CameraData`。 |
| `include/hno_vio/HNOManager.h` / `src/HNOManager.cpp` | `hno_vio::HNOManager` | `HNOManager::stereo_callback()` | ROS2 双目同步图像回调，转换成 `ov_core::CameraData`。 |
| `include/hno_vio/HNOManager.h` / `src/HNOManager.cpp` | `hno_vio::HNOManager` | `HNOManager::time_from_sec()` | 把 double 秒转成 ROS time。 |
| `include/hno_vio/HNOState.h` | `hno_vio::HNOState` | `HNOState::HNOState()` | 初始化默认状态和协方差。 |
| `include/hno_vio/HNOState.h` | `hno_vio::HNOState` | `HNOState::initialize()` | 写入初始姿态、位置、速度、IMU bias。 |
| `include/hno_vio/HNOState.h` | `hno_vio::HNOState` | `HNOState::enforce_structure()` | 正交化 `R_hat_B2I` 和 `e_hat`。 |
| `include/hno_vio/HNOInitializer.h` / `src/HNOInitializer.cpp` | `hno_vio::HNOInitializer` | `HNOInitializer::HNOInitializer()` | 初始化器构造函数。 |
| `include/hno_vio/HNOInitializer.h` / `src/HNOInitializer.cpp` | `hno_vio::HNOInitializer` | `HNOInitializer::setOptions()` | 设置静态初始化窗口和静止阈值。 |
| `include/hno_vio/HNOInitializer.h` / `src/HNOInitializer.cpp` | `hno_vio::HNOInitializer` | `HNOInitializer::feedImuData()` | 接收并维护初始化 IMU 滑动窗口。 |
| `include/hno_vio/HNOInitializer.h` / `src/HNOInitializer.cpp` | `hno_vio::HNOInitializer` | `HNOInitializer::initialize()` | 尝试静态初始化 `HNOState`。 |
| `include/hno_vio/HNOPropagator.h` / `src/HNOPropagator.cpp` | `hno_vio::HNOPropagator` | `HNOPropagator::HNOPropagator()` | 设置默认 `rho` 和噪声协方差。 |
| `include/hno_vio/HNOPropagator.h` / `src/HNOPropagator.cpp` | `hno_vio::HNOPropagator` | `HNOPropagator::setNoiseParams()` | 设置 IMU 噪声协方差 `Cov_nx`。 |
| `include/hno_vio/HNOPropagator.h` / `src/HNOPropagator.cpp` | `hno_vio::HNOPropagator` | `HNOPropagator::propagate()` | 用 IMU 测量传播状态和协方差。 |
| `include/hno_vio/HNOPropagator.h` / `src/HNOPropagator.cpp` | `hno_vio::HNOPropagator` | `HNOPropagator::RK4()` | 四阶 Runge-Kutta 积分协方差。 |
| `include/hno_vio/HNOPropagator.h` / `src/HNOPropagator.cpp` | `hno_vio::HNOPropagator` | `HNOPropagator::skew()` | 私有工具函数，生成反对称矩阵。 |
| `include/hno_vio/HNOFeature.h` / `src/HNOFeature.cpp` | `hno_vio::FeatureInfo` | 数据结构，无成员函数 | 保存单个地图点的 `p_w`、`track_count`、`fail_count`。 |
| `include/hno_vio/HNOFeature.h` / `src/HNOFeature.cpp` | `hno_vio::HNOFeature` | `HNOFeature::HNOFeature()` | 创建 OpenVINS `TrackKLT` 并保存相机模型/外参。 |
| `include/hno_vio/HNOFeature.h` / `src/HNOFeature.cpp` | `hno_vio::HNOFeature` | `HNOFeature::feed_measurement()` | 视觉前端主入口，跟踪、剔除、三角化、维护地图并生成 `HNOObservation`。 |
| `include/hno_vio/HNOFeature.h` / `src/HNOFeature.cpp` | `hno_vio::HNOFeature` | `HNOFeature::get_active_map()` | 返回成熟地图点。 |
| `include/hno_vio/HNOFeature.h` / `src/HNOFeature.cpp` | `hno_vio::HNOFeature` | `HNOFeature::get_tracker()` | 返回内部 `TrackKLT` 指针。 |
| `include/hno_vio/HNOFeature.h` / `src/HNOFeature.cpp` | `hno_vio::HNOFeature` | `HNOFeature::triangulate_stereo()` | 私有函数，双目三角化并做深度/重投影检查。 |
| `include/hno_vio/HNOFeature.h` / `src/HNOFeature.cpp` | `hno_vio::HNOFeature` | `HNOFeature::check_reprojection()` | 私有函数，检查世界点在左目的重投影误差。 |
| `include/hno_vio/HNOUpdater.h` | `hno_vio::HNOObservation` | 数据结构，无成员函数 | 保存左右目归一化观测、右目有效标志和世界点。 |
| `include/hno_vio/HNOUpdater.h` / `src/HNOUpdater.cpp` | `hno_vio::HNOUpdater` | `HNOUpdater::HNOUpdater()` | 初始化默认相机外参。 |
| `include/hno_vio/HNOUpdater.h` / `src/HNOUpdater.cpp` | `hno_vio::HNOUpdater` | `HNOUpdater::setOptions()` | 设置更新门限和保护参数。 |
| `include/hno_vio/HNOUpdater.h` / `src/HNOUpdater.cpp` | `hno_vio::HNOUpdater` | `HNOUpdater::setExtrinsics()` | 设置左右相机 `Camera -> Body` 外参。 |
| `include/hno_vio/HNOUpdater.h` / `src/HNOUpdater.cpp` | `hno_vio::HNOUpdater` | `HNOUpdater::update()` | 执行基于观测的序贯 HNO 更新。 |
| `include/hno_vio/HNOUpdater.h` / `src/HNOUpdater.cpp` | `hno_vio::HNOUpdater` | `HNOUpdater::project_pi()` | 私有函数，计算投影算子 `I - x x^T`。 |

### 2.1 入口节点：`run_hno_vio.cpp`

`src/run_hno_vio.cpp` 是 ROS2 进程入口：

1. 调用 `rclcpp::init()`。
2. 创建节点 `run_hno_vio`。
3. 声明并读取 `config_path` 参数。
4. 如果 `config_path` 为空，打印错误并返回 `2`。
5. 创建 `hno_vio::HNOManager`。
6. 调用 `manager->launch_subscribers()`。
7. 进入 `rclcpp::spin(node)`。

该文件不直接处理 IMU、图像、状态或输出，所有业务逻辑下放到 `HNOManager`。

### 2.2 管理器：`HNOManager`

`HNOManager` 是当前系统的中心编排类。它持有并连接以下对象：

- `HNOState state`
- `HNOPropagator propagator`
- `HNOUpdater updater`
- `HNOFeature feature_handler`
- `HNOInitializer initializer`

#### 参数加载

`HNOManager::load_parameters()` 负责：

- 从 YAML 读取相机内参、畸变、分辨率和 `T_imu_cam` 外参，并构造 `ov_core::CamRadtan`。
- 将 `T_imu_cam` 作为当前代码使用的 `Camera -> Body` 外参保存到 `cams_T_C2B`。
- 读取 IMU 噪声并调用 `HNOPropagator::setNoiseParams()`。
- 读取初始化参数并调用 `HNOInitializer::setOptions()`。
- 读取 feature 和 updater 参数，并允许同名 ROS2 参数覆盖 YAML 默认值。
- 读取 GT 路径 `path_gt`，非空时调用 `load_gt_data()`。
- 读取运行开关和上下文：`use_gt_mapping`、`export_odom`、`odom_output_path`、`dataset`、`bag_path`、`raw_bag`、`config`、`camera_config`、`odom_frame`、`base_frame`、`topic_imu`、`topic_cam0`、`topic_cam1`。
- 当 `export_odom=true` 且 `odom_output_path` 有效时，打开 CSV 和 TUM TXT 输出文件，并写入 `run_context.json`。

#### 订阅

`launch_subscribers()` 使用 `rclcpp::SensorDataQoS()`：

- 总是订阅 IMU topic，默认 `/imu0`。
- `num_cams == 2` 时，使用 `message_filters::Synchronizer<ExactTime>` 同步订阅默认 `/cam0/image_raw` 和 `/cam1/image_raw`。
- `num_cams == 1` 时，仅订阅默认 `/cam0/image_raw`。
- 其他 `num_cams` 值会打印错误并 shutdown。

#### 回调转换

- `imu_callback()`：把 `sensor_msgs::msg::Imu` 转成 `ov_core::ImuData`，字段为 `timestamp`、`wm`、`am`。
- `mono_callback()`：把单目 `sensor_msgs::msg::Image` 转成 `ov_core::CameraData`，`sensor_ids={0}`，图像编码转为 `MONO8`。
- `stereo_callback()`：把同步双目图像转成 `ov_core::CameraData`，`sensor_ids={0,1}`，两路图像均转为 `MONO8`。

#### IMU 数据路径

`feed_measurement(const ov_core::ImuData&)`：

1. 把 IMU 放入 `imu_data_buffer`。
2. 未初始化时，将 IMU 送入 `HNOInitializer`。如果静态初始化成功，设置 `is_initialized=true`，并把 `current_time` 设为初始化窗口最后一帧时间。
3. 已初始化后，复制当前 `state` 得到 `state_viz`，用缓存 IMU 从 `current_time` 临时传播到最新 IMU 时间，用于更高频发布状态。
4. 调用 `publish_state()` 发布 pose、odom、path、TF 和 GT 可视化。

#### 相机数据路径

`feed_measurement(const ov_core::CameraData&)` 已初始化后调用 `process_camera_data()`。

`process_camera_data()` 的主要步骤：

1. 丢弃时间戳不大于 `current_time` 的旧图像。
2. 使用 `imu_data_buffer` 将真实 `state` 从 `current_time` 传播到图像时间戳。
3. 清理已经消费的 IMU 缓存。
4. 插值 GT，供评估、可视化和可选 GT mapping 使用。
5. 调用 `HNOFeature::feed_measurement()` 生成 `HNOObservation` 列表。
   - `use_gt_mapping=true` 且有 GT 时，用 GT 姿态/位置辅助建图。
   - 否则用当前估计姿态/位置建图。
6. 若观测非空，调用 `HNOUpdater::update()` 更新状态。
7. 打印和 GT 的位置误差信息。
8. 调用 `export_odom_state()` 写 `odom_raw.csv` 和 `odom_raw.txt`。
9. 调用 `publish_visualization()` 发布 3D 特征点和可选跟踪图像。

#### 发布和 TF

`publish_state()` 发布：

- `/hno_vio/pose`：`geometry_msgs::msg::PoseWithCovarianceStamped`。
- `/hno_vio/odom`：`nav_msgs::msg::Odometry`，`child_frame_id=base_frame`。
- `/hno_vio/path`：`nav_msgs::msg::Path`。
- `/hno_vio/path_gt`：有 GT 时发布对齐后的 GT path。
- TF：`odom_frame -> base_frame`。
- TF：有 GT 时发布 `odom_frame -> gt_base_link`。

`publish_visualization()` 发布：

- `/hno_vio/features_3d`：`sensor_msgs::msg::PointCloud`，frame 为 `odom_frame`。
- `/hno_vio/image_track`：`sensor_msgs::msg::Image`，仅当该 topic 有订阅者时生成并发布，frame 为 `base_frame`。

#### Odom 导出和运行上下文

`export_odom_state()` 输出两种格式：

- CSV：表头为 `timestamp_ns,tx,ty,tz,qx,qy,qz,qw`。
- TUM TXT：`timestamp tx ty tz qx qy qz qw`。

`write_run_context()` 写入 `run_context.json`，当前字段包括：

- `dataset`
- `raw_bag`
- `config`
- `config_path`
- `camera_config`
- `ground_truth_tum`
- `odom_csv`
- `odom_tum`
- `rtabmap_input_bag`
- `odom_frame`
- `base_frame`
- `camera_left_frame`
- `camera_right_frame`
- `odom_semantic`
- `num_cams`
- `use_gt_mapping`

### 2.3 状态：`HNOState`

`HNOState` 当前全部实现位于 `include/hno_vio/HNOState.h`。

状态变量包括：

- `R_hat_B2I`：Body 到 Inertial 的旋转。
- `p_hat`：惯性系位置。
- `v_hat`：惯性系速度。
- `e_hat[3]`：HNO 辅助向量，估计惯性系基向量。
- `bg`：陀螺仪零偏。
- `ba`：加速度计零偏。
- `P`：`15x15` 协方差。

协方差状态顺序为：

```text
[Pos(0-2), e1(3-5), e2(6-8), e3(9-11), Vel(12-14)]
```

主要函数：

- 构造函数：设置默认状态和初始协方差尺度。
- `initialize()`：填入初始姿态、位置、速度和 IMU bias，并重置 `e_hat` 为单位基。
- `enforce_structure()`：用 SVD 分别把 `R_hat_B2I` 和 `e_hat` 组成的矩阵投影回正交矩阵。

### 2.4 初始化：`HNOInitializer`

`HNOInitializer` 实现静态 IMU 初始化。

主要成员：

- `imu_buffer`：初始化窗口内的 IMU 数据。
- `window_size`：窗口大小，默认 `250`。
- `max_acc_variance`：静止检测加速度方差阈值。
- `max_gyro_variance`：静止检测陀螺仪方差阈值。

主要函数：

- `setOptions()`：设置窗口大小和静止检测阈值。
- `feedImuData()`：加入 IMU 样本，超出窗口后丢弃最旧样本。
- `initialize()`：
  1. 等待 IMU 数量达到窗口大小。
  2. 计算加速度和角速度均值。
  3. 计算方差并做静止检测。
  4. 陀螺仪零偏取角速度均值。
  5. 加速度计零偏当前设为零。
  6. 用平均加速度方向和世界 Z 轴做重力对齐，得到初始 `R0_B2I`。
  7. 调用 `HNOState::initialize()`，初始位置和速度为零。
  8. 返回初始化时间戳。

当前初始化不使用视觉特征，也不读取 `use_gt_init`。

### 2.5 传播：`HNOPropagator`

`HNOPropagator` 负责 IMU propagation 和误差协方差 propagation。

主要配置：

- `NoiseParams::noise_acc`
- `NoiseParams::noise_gyro`
- `k_R = 20.0`
- `rho = [0.5, 0.3, 0.2]`
- `gravity = [0, 0, -9.81]`
- `Cov_nx`：`6x6` IMU 噪声协方差。

主要函数：

- `setNoiseParams()`：由加速度计和陀螺仪噪声密度设置 `Cov_nx`。
- `propagate()`：
  1. 从测量值中扣除 `bg` 和 `ba`。
  2. 根据 `e_hat` 和标准基计算 HNO 姿态反馈项 `sigma_R`。
  3. 积分位置、速度和 `e_hat`。
  4. 用角速度和反馈项通过指数映射更新 `R_hat_B2I`。
  5. 构造连续误差动力学矩阵 `A`。
  6. 构造过程噪声映射 `Gt` 和 `Vt = Gt * Cov_nx * Gt^T`。
  7. 调用 `RK4()` 积分协方差，并强制协方差对称。
- `RK4()`：对 `P_dot = A P + P A^T + Vt` 做四阶 Runge-Kutta 积分。

当前 `state->enforce_structure()` 在传播末尾是注释状态；是否更新后强制结构由 `update_enforce_structure` 控制。

### 2.6 特征与观测：`HNOFeature`

`HNOFeature` 是当前视觉前端和简化地图维护模块。

主要数据结构：

- `FeatureInfo`
  - `p_w`：当前估计的世界系 3D 点。
  - `track_count`：成功跟踪帧计数。
  - `fail_count`：连续失败计数。
- `feature_db`：`feature id -> FeatureInfo`。
- `history_obs`：上一帧左目像素观测，用于 2D-2D RANSAC。
- `tracker`：`ov_core::TrackKLT`。
- `cameras`：OpenVINS 相机模型。
- `T_C_B`：相机到机体的外参。

`Options` 包含：

- KLT 配置：特征数量、FAST 阈值、网格、最小像素距离。
- 双目深度和重投影阈值。
- 少点模式阈值。
- 成熟点阈值和失败清理阈值。
- 地图跳变阈值。
- 地图健康检查阈值和开始帧。

`feed_measurement()` 的主要流程：

1. 调用 OpenVINS `TrackKLT::feed_new_camera()` 跟踪特征。
2. 获取当前帧观测和特征 ID。
3. 用上一帧和当前帧左目观测做 `cv::findFundamentalMat()` RANSAC，剔除 2D 外点。
4. 建立右目 ID 到索引的映射。
5. 根据传入的 `R_gt/p_gt` 或 `R_est/p_est` 确定建图用的 `R_wb/p_wb`。
6. 对每个左目特征：
   - 计算左目归一化坐标。
   - 若有右目匹配，计算右目归一化坐标。
   - 已存在的老点：优先用当前双目三角化刷新 3D 点；若无可靠双目，则做左目重投影检查。
   - 新点：只有有右目且三角化通过时才加入 `feature_db`，先不输出观测。
   - 成熟老点生成 `HNOObservation`。
7. 清理不再被 KLT 跟踪或失败过多的地图点。
8. 根据健康检查条件，在特征过少或地图不稳定时抑制输出观测。
9. 每 30 帧打印前端统计。

辅助函数：

- `triangulate_stereo()`：用左右归一化观测和左右相机外参做双目三角化，检查深度和右目重投影误差。
- `check_reprojection()`：把世界点转到 body 再转到左相机，检查归一化平面重投影误差。
- `get_active_map()`：只返回 `track_count >= active_mature_thresh` 的地图点，用于可视化和统计。

### 2.7 更新：`HNOUpdater`

`HNOUpdater` 接收 `HNOFeature` 生成的 `HNOObservation`，对 `HNOState` 做序贯更新。

`HNOObservation` 包含：

- `uv_left`：左目归一化观测向量。
- `uv_right`：右目归一化观测向量。
- `isValidRight`：右目观测是否有效。
- `xyz`：世界系 3D 点。

`Options` 包含：

- `pixel_noise`
- `focal_length`
- `chi2_gate`
- `max_delta_p`
- `max_delta_r`
- `min_observations`
- `low_observation_hold_frames`
- `warn_delta_ratio`
- `enforce_structure_after_update`

主要函数：

- `setOptions()`：设置更新参数。
- `setExtrinsics()`：设置左右相机 `Camera -> Body` 外参。
- `update()`：
  1. 若观测数量低于 `min_observations`，累计低观测帧数；连续过低时跳过更新。
  2. 对每个观测构造投影残差。
  3. 用 `state->e_hat` 和观测中的 `xyz` 重构路标估计。
  4. 计算左目和可选右目的投影算子。
  5. 构造观测噪声 `Q_i` 和雅可比 `C_i`。
  6. 计算 `S_i = C_i P C_i^T + Q_i`。
  7. 对每个观测做 NaN/奇异检查和卡方检验。
  8. 计算 Kalman 增益和状态修正量。
  9. 使用 `max_delta_p`、`max_delta_r` 限制单次更新幅度。
  10. 更新 `p_hat`、`e_hat[0..2]`、`v_hat`。
  11. 用 Joseph 形式更新协方差，并强制对称。
  12. 对协方差对角线设置下限。
  13. 若 `enforce_structure_after_update=true`，调用 `state->enforce_structure()`。

当前 updater 不更新 `R_hat_B2I` 本身，而是更新 `e_hat`、位置和速度；姿态主要由 propagator 的 HNO 反馈和 IMU 积分推进。

## 3. 当前数据流

主数据流如下：

```text
ROS2 bag / live topics
  -> HNOManager callbacks
  -> ov_core::ImuData / ov_core::CameraData

IMU
  -> HNOInitializer
  -> HNOState initial R, p, v, bg, ba

IMU after init
  -> HNOPropagator
  -> propagated HNOState and covariance
  -> /hno_vio/pose, /hno_vio/odom, /hno_vio/path, TF

Stereo image
  -> HNOFeature / TrackKLT
  -> RANSAC filtering
  -> stereo triangulation
  -> feature_db
  -> HNOObservation list

HNOObservation list
  -> HNOUpdater
  -> gated sequential update
  -> updated HNOState

Updated HNOState
  -> GT error print
  -> odom_raw.csv / odom_raw.txt
  -> /hno_vio/features_3d
  -> /hno_vio/image_track

Ground truth TUM
  -> HNOManager::load_gt_data()
  -> interpolation
  -> /hno_vio/path_gt
  -> odom_frame -> gt_base_link TF
  -> position error print

Optional run_preprocess
  -> rtabmap_preprocess
  -> ros2 bag record
  -> vio_results/rtabmap_input_db3

Offline RTAB-Map
  -> hno_rtabmap.sh
  -> offline_results/odom_optimized.txt
  -> evo_results/APE and trajectory plots
```

## 4. 当前 ROS 接口汇总

### 4.1 输入

默认输入 topic：

- `/imu0`：`sensor_msgs::msg::Imu`
- `/cam0/image_raw`：`sensor_msgs::msg::Image`
- `/cam1/image_raw`：`sensor_msgs::msg::Image`，双目模式使用

这些 topic 可通过 launch 参数 `topic_imu`、`topic_cam0`、`topic_cam1` 覆盖。

### 4.2 输出

固定输出 topic：

- `/hno_vio/pose`：`geometry_msgs::msg::PoseWithCovarianceStamped`
- `/hno_vio/odom`：`nav_msgs::msg::Odometry`
- `/hno_vio/path`：`nav_msgs::msg::Path`
- `/hno_vio/path_gt`：`nav_msgs::msg::Path`
- `/hno_vio/features_3d`：`sensor_msgs::msg::PointCloud`
- `/hno_vio/image_track`：`sensor_msgs::msg::Image`

TF：

- `<odom_frame> -> <base_frame>`，默认 `odom -> base_link`
- `<odom_frame> -> gt_base_link`，有 GT 时发布

### 4.3 文件输出

默认运行输出：

```text
results/run_YYYYmmddTHHMMSS/
  run_context.json
  vio_results/
    odom_raw.csv
    odom_raw.txt
```

`run_preprocess=true` 时还会录制：

```text
results/run_YYYYmmddTHHMMSS/vio_results/rtabmap_input_db3/
```

RTAB-Map 后端会生成：

```text
results/run_YYYYmmddTHHMMSS/offline_results/
```

## 5. 当前边界和注意点

- 当前代码是 ROS2 路径，使用 `ament_cmake`、`rclcpp`、`ros2 launch`、`ros2 bag` 和 RViz2。
- 核心运行入口是 `ros2 launch hno_vio hno_vio.launch.py` 或直接运行安装后的 `run_hno_vio` 节点并提供 `config_path`。
- 当前视觉前端依赖 OpenVINS `TrackKLT`，但状态、传播和更新是本包自己的 `HNOState`、`HNOPropagator`、`HNOUpdater`。
- `use_gt_mapping` 是当前代码中的 GT 辅助建图开关；`use_gt_init` 不在当前节点代码中被读取。
- `results/` 和 `params_eval/` 是运行和评估输出集合，不应逐个文件纳入核心结构理解。
- `run_context.json` 是 RTAB-Map 离线流程连接 VIO 输出、数据集、GT、坐标系和相机上下文的关键文件。
