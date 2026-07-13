# HNO-VIO 第一版结构重构执行计划

## 0. 文档目的

本文档用于指导 Codex 在以下代码版本基础上执行第一版系统结构重构：

- Repository: `liuhestu/HNO-VIO`
- Branch: `hno_vio_reconstruct`
- Baseline commit: `76470818f6150f96c28ba98f53b22cbf1264631d`

本次任务的核心目标不是修改算法，而是：

1. 重构当前 HNO-VIO 的函数分类、文件结构和类职责；
2. 保持现有数学公式、参数、ROS 接口和实验行为不变；
3. 集中整理诊断数据和终端日志；
4. 完善 `tools/run_vio/run_smoke.sh`，使其成为每次重构修改后的自动回归测试入口；
5. 为后续 bias 工程估计和前端替换预留清晰结构，但第一版不实现这些新功能。

本次重构完成后，必须仍然能够复现当前系统的基本行为和精度水平。

---

# 1. 第一版重构的总原则

## 1.1 只重构，不增加新算法

第一版只允许：

- 移动现有代码；
- 拆分类和函数；
- 调整 namespace；
- 重命名文件、类和函数；
- 把散落的配置、诊断数据和日志输出集中；
- 修正 launch 在 rosbag 播放结束后不能自动退出的问题；
- 完善 smoke test。

第一版不允许：

- 修改 HNO 数学公式；
- 修改当前视觉观测模型；
- 修改协方差传播公式；
- 修改 ZUPT 数学；
- 修改前端特征方法；
- 增加 bias 实时估计；
- 增加局部优化或在线回环；
- 修改 RTAB-Map 后端；
- 调整参数以“让结果更好”。

本次任务的首要目标是“行为保持”，而不是“性能优化”。

---

## 1.2 namespace 规范

项目顶层 namespace 使用：

```cpp
namespace hno_vio {
}
```

不要使用：

```cpp
namespace HNO_VIO {
}
```

原因：

- C++ 中全大写名称更像宏或编译期常量；
- namespace 更适合使用小写或 snake_case；
- 保持项目名 `hno_vio` 与 ROS2 package 名一致。

观测器数学模块使用：

```cpp
namespace hno_vio::observer {
}
```

核心类调用形式应为：

```cpp
hno_vio::observer::Propagator
hno_vio::observer::Updater
hno_vio::observer::ZuptUpdater
```

---

## 1.3 不使用复杂设计模式

优先使用：

- 普通类；
- 普通结构体；
- 类组合；
- 明确的输入输出；
- 直接依赖关系。

禁止为了“架构美观”引入：

- 复杂继承；
- 工厂模式；
- 运行时多态；
- 模板元编程；
- service locator；
- dependency injection framework；
- 过度抽象的接口层；
- 不必要的单例。

用户是 C++ 初学者，代码必须优先保证可读、可追踪和便于人工修改。

---

# 2. 当前版本功能边界

当前代码主要由以下模块组成：

- `HNOState`：状态和 15 维协方差；
- `HNOInitializer`：静止 IMU 初始化；
- `HNOPropagator`：HNO 状态和协方差传播；
- `HNOUpdater`：视觉更新和 ZUPT；
- `HNOFeature`：KLT 跟踪、双目三角化、轻量地图、feature health、视觉观测构造；
- `HNOManager`：ROS 节点、参数、订阅发布、pipeline、GT、导出、日志等大量功能；
- `run_hno_vio.cpp`：程序入口；
- `tools/run_rtabmap/`：RTAB-Map 离线后端；
- `tools/run_vio/run_smoke.sh`：当前只做编译 smoke。

本次重构重点是拆分 `HNOManager`、`HNOFeature` 和 `HNOUpdater` 的多重职责。

---

# 3. 第一版推荐目录结构

## 3.1 include 目录

```text
include/hno_vio/
  State.h
  Initializer.h
  Diagnostics.h

  observer/
    Propagator.h
    Updater.h
    ZuptUpdater.h

  frontend/
    FeatureManager.h
    StereoTriangulator.h
    LandmarkMap.h
    FeatureHealth.h

  pipeline/
    VioPipeline.h
    OdomExport.h
    ImuBuffer.h

  ros/
    HnoVioNode.h
    RosPublisher.h
    GTMapping.h
```

## 3.2 src 目录

```text
src/
  Initializer.cpp
  Diagnostics.cpp

  observer/
    Propagator.cpp
    Updater.cpp
    ZuptUpdater.cpp

  frontend/
    FeatureManager.cpp
    StereoTriangulator.cpp
    LandmarkMap.cpp
    FeatureHealth.cpp

  pipeline/
    VioPipeline.cpp
    OdomExport.cpp
    ImuBuffer.cpp

  ros/
    HnoVioNode.cpp
    RosPublisher.cpp
    GTMapping.cpp

  run_hno_vio.cpp
```

## 3.3 后端工具

以下目录完全保持不动：

```text
tools/run_rtabmap/
```

不修改其中任何文件：

- `rtabmap_preprocess.py`
- `hno_rtabmap.sh`
- `export_optimized_odom.py`
- `eval_and_analysis.py`

---

# 4. 各文件与类的职责

# 4.1 `State.h`

所有状态统一放在一个文件中，不拆成多个状态文件。

应包含：

- 姿态 `R_hat_B2I`；
- 位置 `p_hat`；
- 速度 `v_hat`；
- 三个结构向量 `e_hat`；
- `ba`；
- `bg`；
- 15 维协方差矩阵 `P`；
- 结构正交化函数；
- 15 维状态索引；
- 共享纯 C++ `Pose`，至少保存旋转和平移，并可从 `State` 构造。

不新增 `Types.h`；`Pose` 与 `State` 共同定义在 `State.h`。状态成员名保持为
`R_hat_B2I`、`p_hat`、`v_hat`、`e_hat`、`bg`、`ba`、`P`，避免迁移公式时同时改动数学变量名。

推荐结构：

```cpp
namespace hno_vio {

struct State;

struct Pose {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p = Eigen::Vector3d::Zero();

    static Pose FromState(const State& state);
};

struct State {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix3d R_hat_B2I = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_hat = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_hat = Eigen::Vector3d::Zero();

    std::array<Eigen::Vector3d, 3> e_hat = {
        Eigen::Vector3d::UnitX(),
        Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitZ()
    };

    Eigen::Vector3d bg = Eigen::Vector3d::Zero();
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();

    Eigen::Matrix<double, 15, 15> P;

    State() {
        P.setIdentity();
        P.block<3, 3>(0, 0) *= 1e-4;
        P.block<3, 3>(3, 3) *= 0.01;
        P.block<3, 3>(6, 6) *= 0.01;
        P.block<3, 3>(9, 9) *= 0.01;
        P.block<3, 3>(12, 12) *= 0.1;
    }

    void enforce_structure();
};

inline Pose Pose::FromState(const State& state) {
    return Pose{state.R_hat_B2I, state.p_hat};
}

namespace StateIndex {

constexpr int kP = 0;
constexpr int kE1 = 3;
constexpr int kE2 = 6;
constexpr int kE3 = 9;
constexpr int kV = 12;
constexpr int kSize = 15;

}

}
```

必须明确注释：

```cpp
// bg and ba are engineering IMU bias values.
// They are not included in the current 15D covariance.
```

第一版不允许：

- 将 `bg/ba` 加入 `P`；
- 将 `P` 扩展到 21x21；
- 修改状态数学定义；
- 修改 `enforce_structure()` 的数学行为。

`P` 的顺序固定为 `[p,e1,e2,e3,v]`，维度保持 15×15；`bg/ba` 不进入误差状态。
`State` 构造函数必须保持上述现有分块初值，后续初始化流程不得重新初始化 `P`。

---

# 4.2 `Initializer`

文件位置：

```text
include/hno_vio/Initializer.h
src/Initializer.cpp
```

不放在 `pipeline/` 子目录。

职责：

- 收集初始 IMU；
- 判断静止；
- 计算平均加速度；
- 计算平均角速度；
- 初始化 roll/pitch；
- 初始化 `bg`；
- 初始化 `ba=0`；
- 初始化 `R_hat_B2I/p_hat/v_hat/e_hat`。

第一版必须保持现有行为：

- `bg0 = average gyro`
- `ba0 = 0`
- 不增加 bias 在线估计；
- 不改变静止判断阈值；
- 不改变初始姿态计算方法；
- 不重新初始化或覆盖 `State` 构造函数已经设置的 `P` 分块初值。

---

# 4.3 `observer::Propagator`

文件：

```text
observer/Propagator.h
observer/Propagator.cpp
```

namespace：

```cpp
namespace hno_vio::observer {
}
```

职责：

- IMU 去 bias：
  - `omega = omega_m - bg`
  - `accel = acc_m - ba`
- 计算 `sigma_R`；
- 传播 `R/p/v/e`；
- 构造现有传播矩阵；
- 传播 15 维协方差；
- 保留当前结构投影行为。

不负责：

- ROS；
- 图像；
- GT；
- odom 导出；
- RTAB-Map；
- 终端日志格式；
- feature 管理。

第一版不允许修改传播公式。

---

# 4.4 `observer::Updater`

文件：

```text
observer/Updater.h
observer/Updater.cpp
```

职责：

- 定义视觉观测结构；
- 计算视觉残差；
- 构造 `C_i`；
- 构造 `Q_i`；
- 计算 `S_i`；
- chi-square gate；
- Kalman gain；
- delta guard；
- 更新 `p/e/v`；
- Joseph form 更新协方差；
- 可选结构正交化；
- 返回更新结果和诊断数据。

`VisualObservation` 直接定义在 `Updater.h` 中。

示例：

```cpp
namespace hno_vio::observer {

struct VisualObservation {
    Eigen::Vector3d uv_left;
    Eigen::Vector3d uv_right;
    Eigen::Vector3d landmark;
    bool has_right = false;
};

class Updater {
public:
    struct Options {
        double pixel_noise = 2.0;
        double focal_length = 460.0;
        double chi2_gate = 15.0;
        double max_delta_p = 0.2;
        double max_delta_r = 0.15;
        int min_observations = 20;
        int low_observation_hold_frames = 3;
        double warn_delta_ratio = 0.8;
        bool enforce_structure_after_update = false;
        double zupt_velocity_noise = 0.05;
    };

    bool update(
        State& state,
        const std::vector<VisualObservation>& observations,
        UpdaterDiagnostics* diagnostics);
};

}
```

上述默认值必须与当前 `HNOUpdater::Options` 一致：
`2.0 / 460.0 / 15.0 / 0.2 / 0.15 / 20 / 3 / 0.8 / false / 0.05`。
配置文件覆盖默认值属于现有行为，不得把 YAML 覆盖值误写成 C++ 默认值。

`Updater::update()` 返回值与 `UpdaterDiagnostics::update_applied` 语义必须完全一致：
只有本帧至少一个 observation 依次通过数值检查、chi-square、Kalman gain 检查和
delta guard，并实际完成 `State` 与 `P` 更新时才为 `true`。仅通过 chi-square、但随后因
gain 或 delta 检查被拒绝的 observation，不算 applied。

视觉更新继续保持当前的序贯更新、低观测 streak、按观测数动态收紧 delta 阈值、
Joseph form、`P` 对称化、负对角时重置和对角下限保护。

第一版不允许修改：

- 残差公式；
- 雅可比公式；
- 卡方逻辑；
- delta 截断逻辑；
- Joseph form；
- 参数默认值；
- 参数 YAML 数值。

---

# 4.5 `observer::ZuptUpdater`

文件：

```text
observer/ZuptUpdater.h
observer/ZuptUpdater.cpp
```

职责：

- 从原 `HNOUpdater` 中迁移 zero velocity update；
- 构造速度残差；
- 计算 ZUPT gain；
- 保留当前只修正速度的工程保护；
- 返回是否执行更新；
- 填充 ZUPT diagnostics。

第一版必须保留当前行为：

- 当前显式清零位置和姿态相关 gain 的逻辑不得改变；
- 不允许通过 ZUPT 更新 bias；
- 不允许修改 ZUPT noise；
- 不允许修改 ZUPT 触发规则。

---

# 4.6 `FeatureManager`

文件：

```text
frontend/FeatureManager.h
frontend/FeatureManager.cpp
```

职责：

- 包装现有 OpenVINS KLT tracker；
- 输入单目或双目图像；
- 获取 feature id 和 2D observation；
- 维护上一帧 track history；
- 调用 fundamental matrix RANSAC；
- 调用 `StereoTriangulator`；
- 调用 `LandmarkMap`；
- 调用 `FeatureHealth`；
- 构造并返回 `observer::VisualObservation`。

第一版可以保留较多现有 `HNOFeature` 代码，重点是先完成职责迁移，不要求一次性做极细拆分。

第一版禁止：

- 更换 KLT；
- 修改 OpenVINS tracker 参数；
- 引入 ORB、SuperPoint、LightGlue；
- 修改 RANSAC 阈值；
- 修改双目匹配行为。

---

# 4.7 `StereoTriangulator`

文件：

```text
frontend/StereoTriangulator.h
frontend/StereoTriangulator.cpp
```

职责：

- 输入左右目归一化坐标；
- 使用双目外参；
- 三角化 3D 点；
- 检查深度范围；
- 检查右目重投影误差；
- 返回成功/失败和 3D 点。

配置建议分为：

```cpp
struct Parameters {
    // 参与数学模型的标定、相机参数等
};

struct Constraints {
    double min_depth;
    double max_depth;
    double stereo_reprojection_threshold;
};
```

第一版不得修改现有三角化数学实现和阈值。

---

# 4.8 `LandmarkMap`

文件：

```text
frontend/LandmarkMap.h
frontend/LandmarkMap.cpp
```

职责：

- 管理 `feature_id -> Landmark`；
- 保存世界系路标；
- 保存 track count；
- 保存 fail count；
- 判断 mature landmark；
- 执行 map jump reject；
- 仅在原始左目 KLT id 消失时删除对应地图项；
- 输出 active map。

GT mapping 逻辑必须保留为“反解世界坐标时选择位姿来源”。

推荐调用方式：

```cpp
Pose mapping_pose = Pose::FromState(state);

if (use_gt_mapping && raw_gt_pose.has_value()) {
    // 只在首次取得有效 GT 时确定 raw-GT -> estimator 的固定变换。
    // 之后只变换 mapping pose 副本，绝不写回 State。
    mapping_pose = alignGtMappingPose(*raw_gt_pose, state);
}

landmark_map.update(
    tracks,
    mapping_pose);
```

必须注意：

- 对齐后的 GT mapping pose 只用于把机体系/相机系 3D 点转换到世界系 landmark；
- GT pose 不覆盖最终 VIO 输出；
- GT pose 不直接修改 `State`；
- GTMapping 不直接持有或修改 LandmarkMap。

首次 mapping 对齐由 `VioPipeline` 在收到第一份有效原始 GT pose 时建立：

```text
R_estimator_gt = R_state * R_raw_gt^T
t_estimator_gt = p_state - R_estimator_gt * p_raw_gt
```

这个固定变换只用于生成后续的 mapping pose 副本，不得用于重置、覆盖或修正 `State`。

`fail_count` 只记录已有路标的更新/检查失败，不触发删除。当前配置中的
`fail_limit/fail_limit_low` 没有被现有流程使用，第一版不得启用它们。RANSAC 外点只在
当前帧跳过，不会自动删除对应地图项；地图项仅在其原始左目 KLT id 消失时删除。

---

# 4.9 `FeatureHealth`

文件：

```text
frontend/FeatureHealth.h
frontend/FeatureHealth.cpp
```

职责：

- 保存和更新前端健康状态；
- 统计 tracked feature；
- 统计 stable feature；
- 统计 landmark map size；
- 统计连续低健康帧数；
- 决定是否允许视觉更新。

推荐：

```cpp
class FeatureHealth {
public:
    struct Constraints {
        int start_frame = 60;
        int min_stable_features = 20;
        int min_landmark_count = 20;
        int hold_frames = 3;
    };

    struct Status {
        int frame_id = 0;
        int tracked_count = 0;
        int stable_count = 0;
        int landmark_count = 0;
        int unhealthy_streak = 0;
        bool allow_visual_update = true;
    };
};
```

`FeatureHealth` 不负责：

- 跟踪；
- 三角化；
- 删除路标；
- Kalman update；
- 终端日志打印。

---

# 4.10 `VioPipeline`

文件：

```text
pipeline/VioPipeline.h
pipeline/VioPipeline.cpp
```

不要命名为 `HnoVioPipeline`。

职责：

- 作为纯 C++ 主流程；
- 持有 committed state 和可修订 prediction state；
- 持有 `Initializer`；
- 持有 `ImuBuffer`；
- 持有 `observer::Propagator`；
- 持有 `observer::Updater`；
- 持有 `observer::ZuptUpdater`；
- 持有 `FeatureManager`；
- 协调 IMU 和 camera 数据处理顺序；
- 收集 diagnostics。

`committed_state` 表示已经处理完截至 `committed_time` 的全部离散相机观测后的正式状态。
`prediction_state` 只能从最新 committed state 使用后续 IMU 临时传播得到，不得覆盖
committed state。相机更新后必须废弃旧 prediction，并使用仍保留的未来 IMU 重新传播。

推荐依赖关系：

```text
VioPipeline
├── State
├── Initializer
├── ImuBuffer
├── observer::Propagator
├── observer::Updater
├── observer::ZuptUpdater
├── FeatureManager
└── Diagnostics
```

`VioPipeline` 不持有、不读取也不发布 `GTMapping`，并且不持有或调用 `OdomExport`。
二者均由第 4.17 节定义的 `HnoVioNode` 持有和编排。

`VioPipeline` 不得依赖 ROS message：

- 不使用 `sensor_msgs`；
- 不使用 `nav_msgs`；
- 不使用 `geometry_msgs`；
- 不使用 `rclcpp::Time`。

应使用纯 C++ 数据结构，如：

```cpp
struct ImuData {
    double timestamp;
    Eigen::Vector3d gyro;
    Eigen::Vector3d accel;
};

struct StereoData {
    double timestamp;
    cv::Mat left;
    cv::Mat right;
};
```

---

# 4.11 `GTMapping`

`GTMapping` 的最终位置、职责和 ROS 边界以第 4.18 节为准。

# 4.12 `OdomExport`

文件：

```text
pipeline/OdomExport.h
pipeline/OdomExport.cpp
```

职责：

- 导出 `odom_raw.csv`；
- 导出对应 TUM txt；
- GT mapping smoke 时支持导出 `odom_gt.csv` 和 `odom_gt.txt`；
- 写 `run_context.json`；
- 管理输出文件打开、关闭、flush；
- 保存一次运行的元信息。

`RunContextWriter` 不再作为独立类，功能合并到 `OdomExport`。

`OdomExport` 应接收纯 C++ 数据：

```cpp
struct OdomRecord {
    double timestamp;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};
```

不要要求输入 `nav_msgs::msg::Odometry`。

---

# 4.13 `ImuBuffer`

文件：

```text
pipeline/ImuBuffer.h
pipeline/ImuBuffer.cpp
```

职责：

- 按时间戳有序保存 IMU，重复时间戳只保留最新内容；
- 判断 `[t_committed, t_camera]` 是否具有双边界覆盖；
- 返回包含起止边界的积分段；
- 非精确对齐时使用 camera time 左右 IMU 对边界测量做插值；
- 返回 `t > t_camera` 的未来 IMU，供视觉校正后重传播；
- 清理旧 IMU 时保留下一积分区间所需的边界样本。

不得用 `imu_buffer.back()` 代表 camera time 的测量，也不得使用 camera time 之后的 IMU
作为零阶保持值向过去传播。相机更新后不得删除全部未来 IMU。

---

# 4.14 `HnoVioNode`

文件：

```text
ros/HnoVioNode.h
ros/HnoVioNode.cpp
```

职责：

- ROS2 node；
- 声明和读取 ROS 参数；
- 创建 IMU subscriber；
- 创建 image subscriber；
- 双目同步；
- 将 ROS message 转为纯 C++ 数据；
- 调用 `VioPipeline`；
- 调用 `RosPublisher`；
- 调用 `GTMapping`；
- 调用 `OdomExport`；
- 管理 ROS 生命周期。

不允许把 observer 数学公式放进 Node。

---

# 4.15 `RosPublisher`

文件：

```text
ros/RosPublisher.h
ros/RosPublisher.cpp
```

职责：

- 发布 estimated pose；
- 发布 estimated odom；
- 发布 estimated path；
- 发布 `odom -> base_link` TF；
- 发布 active features；
- 发布 image track；
- 保持现有 topic 名称和 frame 语义。

estimated pose/odom/`odom -> base_link` 表达当前最新 prediction；estimated Path 只保存
相机更新后的 committed correction。不得把可修订 prediction 与 correction 永久
`push_back()` 到同一 Path。Path 内部按纳秒时间戳索引并按时间顺序生成。

`RosPublisher` 不创建任何 GT publisher，不维护 GT path，也不发送
`odom -> gt_base_link` TF；这些职责由 `GTMapping` 独占。

不得修改：

- topic 名称；
- `odom -> base_link` 语义；
- 消息类型；
- 当前可视化功能。

---


# 4.16 FeatureManager 固定调用顺序与 LandmarkMap 管理规则

Codex 必须先完整阅读重构前的 `include/hno_vio/HNOFeature.h` 和 `src/HNOFeature.cpp`，并建立旧函数到新类/新函数的对应关系。不得仅根据新类名重新设计前端流程。本文档未覆盖的实现细节，以重构前代码实际行为为准，禁止自行简化、优化或改变执行顺序。

## 4.16.1 FeatureManager 固定调用顺序

`FeatureManager::processStereo()` 或等价主函数保持以下总体顺序：

1. 调用现有 OpenVINS KLT tracker 处理当前左右图像；
2. 获取当前帧 feature ids 和左右目 2D observations；
3. 读取上一帧 history observations，建立 common tracks 并统计 median disparity；
4. 使用旧代码相同的数据执行 fundamental matrix RANSAC，当前帧跳过其外点；
5. 遍历当前有效 feature，更新已有路标或初始化新路标；
6. 已有路标达到 mature threshold 时立即构造候选 `observer::VisualObservation`；
7. 新路标只插入地图并设置 `track_count=1`，当前帧不构造候选 observation；
8. 根据原始左目 KLT ids 清理地图；
9. 更新 history observations；
10. 使用本帧统计更新 `FeatureHealth`；
11. health guard 触发时，整体清空步骤 6 已经生成的候选 observations；
12. 生成 active map、track image 和 diagnostics。

不得在 common track、RANSAC、地图更新或候选 observation 构造完成前覆盖上一帧
history。mature observation 必须先生成候选，再由 health guard 决定是否整体清空，
不得把顺序改成“先判断 health，允许后才构造 observation”。

## 4.16.2 LandmarkMap 是世界路标的唯一所有者

`LandmarkMap` 必须是 `feature_id -> Landmark` 的唯一所有者。

```cpp
struct Landmark {
    Eigen::Vector3d p_world;
    int track_count = 0;
    int fail_count = 0;
};
```

约束：

- `FeatureManager` 负责编排流程，但不保存第二份世界路标地图；
- `StereoTriangulator` 只计算 3D 点，不保存地图状态；
- `observer::Updater` 不拥有 landmark map；
- `GTMapping` 只提供 GT pose，不直接修改 landmark map；
- 不允许多个模块各自保存一份 `feature_id -> p_world`。

## 4.16.3 新特征点管理规则

对于当前 id 不在 `LandmarkMap` 中的特征：

```text
存在有效左右目观测
  → 双目三角化成功
  → 深度检查通过
  → 右目重投影检查通过
  → 选择 GT pose 或 estimated pose
  → 转换为世界坐标
  → 插入 LandmarkMap
```

缺少有效双目观测或任一检查失败时，不创建新的世界路标。第一版不得自行增加单目初始化世界路标的逻辑。
新路标插入时固定设置 `track_count=1`、`fail_count=0`，并且当前帧不送入 Updater。

## 4.16.4 已有路标管理规则

当前 id 已存在于 `LandmarkMap` 中时，必须迁移旧代码当前行为。

有有效双目观测时：

1. 重新三角化；
2. 转换为世界坐标；
3. 检查新旧世界坐标跳变；
4. 跳变过大时按旧代码增加 `fail_count`；
5. 合法时按旧代码的刷新或融合方式更新 `p_world`；
6. 通过检查后将 `fail_count` 清零，并按旧代码更新 `track_count`，上限保持 15。

无有效双目观测时：

1. 使用已有 `p_world` 和当前 pose 做重投影检查；
2. 通过时按旧代码更新 track 状态；
3. 失败时按旧代码增加 `fail_count`。

不得自行改变为每帧完全覆盖旧 `p_world`、永久固定初始 anchor、删除旧代码已有平滑/融合，或改变 `track_count`、`fail_count` 的更新时机。

地图删除只由原始左目 KLT id 是否仍存活决定。`fail_count` 只做失败记录，不参与删除；
不得启用当前未使用的 `fail_limit/fail_limit_low`。fundamental RANSAC 外点当前帧不参与
地图更新或 observation 构造，但不会因此自动删除其已有地图项。

## 4.16.5 GT mapping 的选择位置

GT pose 只允许在“已三角化的相机系/机体系 3D 点转换成世界坐标”时参与选择：

```cpp
Pose mapping_pose = Pose::FromState(state);

if (use_gt_mapping && raw_gt_pose.has_value()) {
    mapping_pose = alignGtMappingPose(*raw_gt_pose, state);
}

p_world = mapping_pose.R * p_body + mapping_pose.p;
```

`alignGtMappingPose()` 只在首次有效 GT 上建立固定的 raw-GT 到 estimator 坐标变换，
之后返回变换后的临时 `Pose`。禁止用 GT pose 或该对齐结果覆盖 `State`、参与 IMU
propagation、覆盖最终 odometry、重置估计轨迹或直接修改 `LandmarkMap`。

## 4.16.6 mature observation 规则

遍历已有路标时，满足以下条件就立即构造候选 `observer::VisualObservation`：

- 对应 id 已存在于 `LandmarkMap`；
- 满足旧代码的 mature threshold；
- 当前帧存在有效观测；
- 旧代码已有的其他检查均通过。

新建路标只进入地图，当前帧不输出。所有 feature 遍历、地图清理和 history 更新完成后，
再更新 `FeatureHealth`；若 health guard 触发，则整体清空已经生成的候选 observations。
不得让刚创建的新路标绕过 mature 规则直接进入 Updater，也不得将 health 判断提前到候选
observation 构造之前。

# 4.17 `run_hno_vio.cpp`、`HnoVioNode` 与 `VioPipeline` 的职责边界

第一版采用“纯 C++ `VioPipeline` + ROS 应用层 `HnoVioNode`”方案。

## 4.17.1 `run_hno_vio.cpp`

只负责 ROS 初始化、创建 `HnoVioNode`、spin、shutdown 和必要的顶层异常捕获。不得负责参数、YAML、subscriber/publisher、GT、VIO 数学、导出或日志。

## 4.17.2 `HnoVioNode`

`HnoVioNode` 是 ROS 应用层总控，负责持有：

```text
VioPipeline
GTMapping
OdomExport
Diagnostics
RosPublisher
```

职责：

- 声明和读取 ROS 参数；
- 创建 IMU、图像 subscriber 和双目同步；
- 将 ROS message 转换为纯 C++ `ImuData`、`StereoData`；
- 加载和查询 GT；
- 将 optional GT pose 传给 `VioPipeline`；
- 调用 `VioPipeline`；
- 调用 `RosPublisher` 发布估计结果；
- 调用 `OdomExport` 输出 odom 和 run context；
- 调用 `Diagnostics` 打印和保存诊断信息；
- 管理 ROS 生命周期。

不得在 `HnoVioNode` 中重新实现 propagation、visual update、ZUPT、三角化、landmark map 更新或 feature health。

## 4.17.3 `VioPipeline`

`VioPipeline` 是纯 C++ VIO 流程类，不是只包装 Propagator/Updater 的薄壳。

它持有：

```text
State
Initializer
ImuBuffer
observer::Propagator
observer::Updater
observer::ZuptUpdater
FeatureManager
```

IMU 流程：

```text
接收 ImuData
  → 未初始化时交给 Initializer
  → 初始化后按时间插入 ImuBuffer
  → 从 committed state 临时调用 Propagator
  → 更新 prediction state，不覆盖 committed state
```

图像流程：

```text
接收 StereoData 和 optional GT Pose
  → 等待 ImuBuffer 覆盖图像时间戳
  → 从 committed state 精确传播到图像时间戳
  → 调用 FeatureManager
  → 在世界路标反解时选择 GT pose 或 estimated pose
  → 构造 VisualObservation
  → 调用 Updater
  → 可选调用 ZuptUpdater
  → 保存新的 committed state
  → 保留并重放晚于图像时间戳的 IMU，重建 prediction state
  → 返回 committed correction 和最新 prediction
```

精确同时间戳的事件顺序固定为“IMU propagation ending at `t_camera`，然后 camera
correction at `t_camera`”。实现仍必须支持不精确对齐的相机时间戳。

`HnoVioNode` 的 IMU 和相机回调只负责分别入队，并共同调用统一的
`tryProcessReadyCameras()`。pending camera queue 必须按时间处理；即使相机先到、对应 IMU
后到，也不得提前处理或丢弃相机观测。

`VioPipeline` 可以填充 diagnostics、返回当前 State、active landmarks、track image 和 update 结果，但不得直接发布 ROS、读取 GT 文件、写 odom、写 run context 或组织终端打印格式。

推荐调用：

```cpp
auto gt_pose = gt_mapping_.getPose(timestamp);
auto result = vio_pipeline_.processStereo(stereo_data, gt_pose);
```

其中 `gt_pose` 必须是纯 C++ `std::optional<Pose>`。

# 4.18 GT path/TF 发布

`GTMapping` 由 `HnoVioNode` 持有。它匹配当前真实 ROS 接口，允许依赖
`rclcpp`、`geometry_msgs`、`nav_msgs` 和 `tf2_ros`。

推荐位置：

```text
include/hno_vio/ros/GTMapping.h
src/ros/GTMapping.cpp
```

## 4.18.1 GTMapping 职责

`GTMapping` 可以负责：

- 读取 GT 轨迹文件；
- 保存 GT 数据；
- 按时间戳查询或插值 GT pose；
- `getPose(timestamp)` 返回未经可视化对齐的纯 C++ `std::optional<Pose>`；
- `publish(timestamp, estimated_pose)` 在内部构造对齐后的 GT `PoseStamped`，追加并发布
  现有 `/hno_vio/path_gt`；
- 发布现有 `odom -> gt_base_link` TF；
- 保持当前“首次估计位姿确定可视化对齐”的逻辑。

即使内部允许使用 ROS，它传给 `VioPipeline` 的仍必须是纯 C++ `std::optional<Pose>`。
原始 GT pose、`VioPipeline` 内的 mapping 对齐 pose 和 `GTMapping` 内的可视化对齐 pose
必须分开。只有未经可视化对齐的原始 pose 可以从 `GTMapping` 传入 `VioPipeline`；
mapping 对齐结果只供路标反解使用，可视化对齐结果只供 GT path/TF 使用。

## 4.18.2 GT path/TF 唯一发布者

`GTMapping` 独占发布现有 `/hno_vio/path_gt` 和 `odom -> gt_base_link` TF。
不新增独立 GT pose topic；内部构造的 `PoseStamped` 只追加到 `Path`，不单独发布。

`RosPublisher` 不得重复发布：

- GT path。

`RosPublisher` 也不得发送 `gt_base_link` TF。

`RosPublisher` 只负责估计相关输出：

- estimated pose；
- estimated odom；
- estimated path；
- `odom -> base_link` TF；
- active features；
- image track。

必须避免 `GTMapping` 和 `RosPublisher` 同时持有 GT publisher。

## 4.18.3 GTMapping 与 VioPipeline 的边界

禁止把 `geometry_msgs::msg::PoseStamped`、`nav_msgs::msg::Path` 或 `rclcpp::Time` 传入 `VioPipeline`。

`GTMapping` 不得修改 `State`、直接修改 `LandmarkMap`、替换最终估计轨迹、参与 propagation
或直接调用 Updater。原始 GT pose 的实际使用位置仍是 `VioPipeline` 中的 mapping pose
选择和首次固定坐标系对齐；对齐后的 mapping pose 再传给 `FeatureManager`。整个过程不得
写回 `State`。

相机帧流程固定为：

```text
GTMapping 查询未经对齐的原始 Pose
  → HnoVioNode 将 optional<Pose> 传给 VioPipeline
  → VioPipeline 在 camera time 形成 committed correction 并重建 prediction
  → RosPublisher 将 committed correction 写入有序 Path，并发布最新 prediction pose/odom/TF
  → GTMapping 发布对齐后的 GT path/TF
  → OdomExport 只导出 committed correction
```


# 5. Parameters、Constraints、Diagnostics 的规则

## 5.1 Parameters

表示参与数学计算的模型参数，例如：

- gravity；
- `k_R`；
- `rho`；
- gyro noise；
- accel noise；
- pixel noise；
- focal length。

判断标准：

```text
决定公式如何计算 → Parameters
```

## 5.2 Constraints

表示工程门限、拒绝、截断和保护条件，例如：

- chi2 gate；
- max delta p；
- max delta r；
- min observations；
- feature health threshold；
- max stereo depth；
- reprojection threshold；
- map jump threshold；
- hold frames。

判断标准：

```text
决定结果能否接受 → Constraints
```

## 5.3 Diagnostics

表示运行时产生的结果和统计信息。

Diagnostics：

- 不能反向控制算法；
- 不参与门限判断；
- 不修改 State；
- 不修改 Constraints；
- 只记录、保存、打印。

---

# 6. `Diagnostics.h/.cpp`

所有诊断结构集中在：

```text
include/hno_vio/Diagnostics.h
src/Diagnostics.cpp
```

可以定义：

```cpp
struct StateDiagnostics;
struct PropagatorDiagnostics;
struct UpdaterDiagnostics;
struct ZuptDiagnostics;
struct FeatureDiagnostics;
struct PipelineDiagnostics;
```

第一版建议至少包含：

## 6.1 StateDiagnostics

- timestamp；
- position norm；
- velocity norm；
- rotation orthogonality error；
- `e_hat` orthogonality error。

## 6.2 PropagatorDiagnostics

- dt；
- corrected gyro；
- corrected accel；
- sigma_R；
- covariance norm；
- propagation count。

## 6.3 UpdaterDiagnostics

- total observations；
- chi2 passed observations；
- applied observations；
- chi2 rejected；
- numerical rejected；
- Kalman gain rejected；
- delta rejected；
- update applied；
- max chi2；
- max delta p；
- max delta r。

`chi2_passed_observations` 与 `applied_observations` 必须分开统计。前者只表示通过
chi-square；后者只统计随后也通过 Kalman gain 检查和 delta guard，且确实完成 State 与
P 更新的 observations。`update_applied == (applied_observations > 0)`，并与
`Updater::update()` 的返回值一致。

## 6.4 ZuptDiagnostics

- stationary detected；
- update applied；
- velocity residual norm。

## 6.5 FeatureDiagnostics

- tracked count；
- common track count；
- stable count；
- new landmark count；
- landmark map size；
- stereo passed；
- reprojection passed；
- feature health status。

## 6.6 PipelineDiagnostics

- initialization status；
- current timestamp；
- frame index；
- current stage；
- latest module status。

---

## 6.7 终端打印

终端打印集中在 `Diagnostics.cpp`。

推荐：

```cpp
class Diagnostics {
public:
    void printState(const StateDiagnostics&);
    void printPropagation(const PropagatorDiagnostics&);
    void printUpdate(const UpdaterDiagnostics&);
    void printFeature(const FeatureDiagnostics&);
    void printPipeline(const PipelineDiagnostics&);
};
```

可以统一控制：

- 打印频率；
- 日志前缀；
- 每帧或每 N 帧打印；
- debug 开关。

算法类只负责填充 diagnostics，不直接组织复杂终端打印格式。

允许保留必要错误日志，但常规统计日志应迁移到 Diagnostics。

---

# 7. Launch 修改

文件：

```text
launch/hno_vio.launch.py
```

Smoke test 使用：

```bash
rviz:=false
run_preprocess:=false
```

当前问题：

- rosbag 播放结束后的 shutdown 逻辑只在 `run_preprocess=true` 时注册；
- 当 `run_preprocess=false` 时，bag 播放结束后主节点可能继续 spin，launch 不自动退出。

必须修改为：

- 无论 `run_preprocess` 是 true 还是 false；
- 只要 `play_bag=true`；
- rosbag 播放结束后都触发 launch shutdown。

推荐逻辑：

```python
if play_bag:
    play_process = ExecuteProcess(...)

    actions.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=play_process,
                on_exit=[
                    TimerAction(
                        period=2.0,
                        actions=[
                            EmitEvent(
                                event=Shutdown(
                                    reason="input bag playback completed"
                                )
                            )
                        ],
                    )
                ],
            )
        )
    )
```

要求：

1. shutdown 逻辑不能依赖 `run_preprocess`；
2. 不修改已有 launch 参数名；
3. 不改变普通人工运行的其他功能；
4. 外层 smoke script 仍使用 `timeout` 作为死锁保险；
5. 正常测试不能依赖 timeout 结束；
6. timeout 触发时必须判定为失败。

现有参数名必须使用：

```text
run_preprocess
results_root
odom_output_path
use_gt_mapping
export_odom
rviz
```

不要使用：

```text
run_reprocess
default_results
```

---

# 8. `run_smoke.sh` 自动回归测试

文件：

```text
tools/run_vio/run_smoke.sh
```

当前脚本只有编译功能，需要升级成完整的一键测试入口。

每次重构修改后，Codex 都应运行：

```bash
tools/run_vio/run_smoke.sh
```

---

## 8.1 总流程

```text
1. 清理本次 smoke 输出
2. 编译 hno_vio
3. source ROS2 和 workspace
4. GT mapping 测试一次
5. 普通 VIO 测试最多三次
6. 输出 PASS/FAIL 摘要
```

阶段失败定义固定为：GT mapping 失败，或普通 VIO 三次全部失败。GT mapping 失败时立即
结束，不运行普通 VIO；普通 VIO 单次失败只占用当前 run，不直接判阶段失败。

---

## 8.2 编译检查

执行：

```bash
colcon build \
  --packages-select hno_vio \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

要求：

- 编译失败立即退出；
- 不继续运行数据集；
- 返回非零退出码；
- 输出编译日志路径。

---

## 8.3 环境加载

至少加载：

```bash
source /opt/ros/humble/setup.bash
source <workspace>/install/setup.bash
```

保持当前脚本已有的 `/usr/bin/python3` 设置。

---

## 8.4 Smoke 输出目录

固定：

```text
/home/sharpa/hno_vio_clean/src/hno_vio/smoke_results/
```

每次测试前清理本次需要写入的目录，避免误读旧文件。

推荐：

```text
smoke_results/
  logs/
  gt_mapping/
  vio_run_1/
  vio_run_2/
  vio_run_3/
  summary.txt
```

---

## 8.5 第一步：GT Mapping 测试

运行：

```bash
ros2 launch hno_vio hno_vio.launch.py \
  dataset:=V1_01_easy \
  use_gt_mapping:=true \
  run_preprocess:=false \
  export_odom:=true \
  rviz:=false \
  play_bag:=true \
  results_root:=/home/sharpa/hno_vio_clean/src/hno_vio/smoke_results/gt_mapping \
  odom_output_path:=/home/sharpa/hno_vio_clean/src/hno_vio/smoke_results/gt_mapping/odom_gt.csv
```

必须最终生成：

```text
smoke_results/gt_mapping/odom_gt.csv
smoke_results/gt_mapping/odom_gt.txt
```

评估命令：

```bash
evo_ape tum \
  ground_truth/euroc_mav/V1_01_easy.txt \
  smoke_results/gt_mapping/odom_gt.txt \
  -a -r trans_part
```

严格通过条件必须同时满足：

```text
mean <= 0.021
rmse <= 0.040
```

容差通过条件必须同时满足：

```text
mean <= 0.025
rmse <= 0.050
```

达到 strict 时正常 PASS；超过 strict 但仍在 tolerance 内时也返回成功，但终端与
`summary.txt` 必须明确打印“精度退化警告”以及实际 mean/RMSE。超过 tolerance、轨迹完整性
异常或 launch 异常时立即失败，不运行普通 VIO。

附加检查：

- launch 正常退出；
- timeout 未触发；
- `odom_gt.txt` 存在；
- 文件非空；
- 不包含 `nan`；
- 不包含 `inf`；
- evo 正常返回；
- 有效轨迹至少 1931 行且持续时间不少于 131.34 秒；
- 不能读取旧轨迹。

GT mapping 测试失败时，整体 smoke test 立即失败，不继续普通 VIO。

---

## 8.6 第二步：普通 VIO 测试

运行参数：

```bash
use_gt_mapping:=false
run_preprocess:=false
export_odom:=true
rviz:=false
play_bag:=true
```

最多运行 3 次：

```text
vio_run_1
vio_run_2
vio_run_3
```

每次输出：

```text
smoke_results/vio_run_N/odom_raw.csv
smoke_results/vio_run_N/odom_raw.txt
```

单次评估：

```bash
evo_ape tum \
  ground_truth/euroc_mav/V1_01_easy.txt \
  smoke_results/vio_run_N/odom_raw.txt \
  -a -r trans_part
```

单次通过条件必须同时满足：

```text
mean <= 0.8
rmse <= 0.8
```

单次失败时继续下一次，不判当前阶段失败。任意一次通过后立即停止，剩余未执行槽位写
`SKIPPED`。只有 run 1、run 2、run 3 三次全部失败时，普通 VIO 阶段才失败。

每次仍需检查：

- launch 正常退出；
- timeout 未触发；
- odom 文件存在；
- odom 文件非空；
- 无 `nan`/`inf`；
- 有效轨迹至少 2599 行且持续时间不少于 137.09 秒；
- evo 正常返回。

---

## 8.7 轨迹完整性检查

不能只看 APE。完整性只使用阶段 0 冻结基线确定的有效行数和持续时间阈值：

```text
GT mapping: valid rows >= 1931 and duration >= 131.34 s
Normal VIO: valid rows >= 2599 and duration >= 137.09 s
```

行数只统计合法、有限且符合 TUM 格式的数据行。持续时间使用首末有效时间戳之差。

---

## 8.8 timeout

每次 launch 外层可使用：

```bash
timeout 300s ros2 launch ...
```

要求：

- timeout 只是保险；
- 正常运行必须由 launch 在 bag 播放结束后主动 shutdown；
- timeout 返回码 124 必须判定为失败；
- 不能把 timeout 当成正常退出。

---

## 8.9 evo 结果解析

不要依赖人工读取终端。

脚本应自动解析 mean 和 rmse。

可以使用：

```bash
evo_ape ... -r trans_part --save_results ape.zip
```

再从 evo 输出或 zip 中读取统计值。

也可以使用小型 Python 代码解析 evo 保存结果。

最终必须得到数值变量：

```text
mean
rmse
```

然后和阈值比较。

---

## 8.10 最终摘要

脚本结束时打印：

```text
========================================
HNO-VIO Smoke Test
========================================
Build:              PASS

GT Mapping:
  mean:              ...
  rmse:              ...
  completeness:      rows=..., duration=...s, PASS/FAIL
  precision warning: YES/NO
  result:            PASS/FAIL

Normal VIO:
  run 1:             mean=..., rmse=..., completeness=..., PASS/FAIL
  run 2:             mean=..., rmse=..., completeness=..., PASS/FAIL/SKIPPED
  run 3:             mean=..., rmse=..., completeness=..., PASS/FAIL/SKIPPED
  result:            PASS/FAIL

Overall:             PASS/FAIL
========================================
```

`summary.txt` 必须固定列出 normal VIO run 1/2/3。每个已执行 run 都记录 mean、RMSE、
完整性（有效行数和持续时间）以及 PASS/FAIL；未执行项记录 `SKIPPED`。GT mapping 若落在
tolerance 而非 strict 区间，摘要和终端都必须保留精度退化警告。

同时写入：

```text
smoke_results/summary.txt
```

脚本退出码：

- overall PASS → `0`
- overall FAIL → 非零

Overall 失败只由两种情况构成：GT mapping 失败，或普通 VIO 三次全部失败。

---

# 9. 明确禁止事项

Codex 在执行本计划时不得：

1. 修改 HNO 数学公式；
2. 修改状态定义的数学含义；
3. 修改 15 维协方差结构；
4. 把 `ba/bg` 加入协方差；
5. 增加 `BiasFeedbackEstimator`；
6. 增加 bias 实时估计；
7. 在线更新 `ba/bg`；
8. 修改 IMU propagation 数学行为；
9. 修改视觉 residual；
10. 修改视觉 Jacobian；
11. 修改 Kalman gain；
12. 修改 Joseph covariance update；
13. 修改 ZUPT 数学；
14. 允许 ZUPT 更新 bias；
15. 修改 chi2 gate；
16. 修改 max delta p/r；
17. 修改 feature health 门限；
18. 修改三角化阈值；
19. 修改 YAML 参数值；
20. 为了通过 smoke test 调参；
21. 替换 KLT；
22. 增加 ORB、SuperPoint、LightGlue；
23. 修改双目三角化数学；
24. 修改 GT mapping 语义；
25. 用 GT pose 覆盖最终 VIO 输出；
26. 修改 RTAB-Map 后端；
27. 修改 `tools/run_rtabmap/`；
28. 修改 ground truth；
29. 修改已有 results；
30. 删除已有功能；
31. 修改 ROS topic 名称；
32. 修改 TF frame 语义；
33. 修改默认 Euroc 数据集；
34. 增加新第三方依赖；
35. 引入复杂设计模式；
36. 一次性重写整个项目；
37. 在 smoke test 失败后继续重构其他模块；
38. 把 smoke test 通过误认为可以顺便修改算法；
39. 为了消除警告而改变算法行为；
40. 未经说明改变任何输出格式。

---

# 10. 推荐执行顺序

必须小步执行。

## 阶段 0：建立基线

阶段 0 已完成并冻结，不得重跑：

```text
GT mapping:
  trajectory: results/run_20260627T011946/vio_results/odom_gt.txt
  mean:       0.019044
  RMSE:       0.038802
  rows:       2145
  duration:   138.25 s

Normal VIO:
  trajectory: results/run_20260707T114053/vio_results/odom_raw.txt
  mean:       0.306036
  RMSE:       0.352850
  rows:       2887
  duration:   144.30 s
```

第 8.7 节的完整性阈值由这些冻结基线确定：有效行数取不低于基线的 90%，持续时间取不
低于基线的 95%。后续阶段只运行自动 smoke，不覆盖或重新生成上述阶段 0 基线。

## 阶段 1：先完善 smoke test

优先修改：

- `launch/hno_vio.launch.py`
- `tools/run_vio/run_smoke.sh`

先让 smoke test 在旧结构上可以工作。

只有 smoke test 稳定后再开始文件重构。

## 阶段 2：重构 State 和 Initializer

迁移：

- `HNOState.h` → `State.h`
- `HNOInitializer.*` → `Initializer.*`

运行 smoke。

## 阶段 3：重构 observer

依次迁移：

1. `Propagator`
2. `Updater`
3. `ZuptUpdater`

每迁移一个模块：

- 编译；
- 运行 smoke；
- 失败则只修复当前模块。

## 阶段 4：重构 Frontend

先建立：

- `FeatureManager`
- `StereoTriangulator`
- `LandmarkMap`
- `FeatureHealth`

优先保持现有代码原样移动，不立即大改内部流程。

运行 smoke。

## 阶段 5：重构 Pipeline

建立：

- `VioPipeline`
- `OdomExport`
- `ImuBuffer`

把纯 C++ 流程从 `HNOManager` 移出。`GTMapping` 在 ROS 层阶段迁移，由 `HnoVioNode` 持有。

运行 smoke。

## 阶段 6：重构 ROS 层

建立：

- `HnoVioNode`
- `RosPublisher`
- `GTMapping`

让 ROS 层负责消息转换和通信。`GTMapping` 独立负责 GT 文件、插值以及现有 GT path/TF；
不新增独立 GT pose topic，`RosPublisher` 不重复发布 GT。

运行 smoke。

## 阶段 7：集中 Diagnostics

把现有常规统计日志迁移到：

- `Diagnostics.h`
- `Diagnostics.cpp`

不新增复杂诊断算法。

运行 smoke。

## 阶段 8：清理旧文件

只有在所有功能迁移完成且 smoke 通过后，才允许删除：

- `HNOState.h`
- `HNOInitializer.h/.cpp`
- `HNOPropagator.h/.cpp`
- `HNOUpdater.h/.cpp`
- `HNOFeature.h/.cpp`
- `HNOManager.h/.cpp`

删除前必须确认：

- 没有残余 include；
- CMake 已更新；
- 安装规则已更新；
- launch 可运行；
- smoke 通过。

---

# 11. 每次修改后的输出要求

Codex 每完成一个小步骤，应输出：

```text
Modified files:
- ...

Moved functionality:
- ...

Behavior changes:
- None
```

若存在任何行为变化，必须明确写出。

禁止在实际发生行为变化时写：

```text
Behavior changes:
- None
```

每次还应附上：

```text
Build:
- PASS/FAIL

Smoke:
- GT Mapping: PASS/FAIL（tolerance PASS 时附精度退化警告）
- Normal VIO: PASS/FAIL/SKIPPED
```

smoke 阶段失败只表示 GT mapping 失败，或普通 VIO 三次全部失败。

---

# 12. 第一版完成标准

第一版重构完成必须满足：

1. 项目正常编译；
2. `ros2 launch hno_vio hno_vio.launch.py` 正常运行；
3. `rviz:=false run_preprocess:=false` 时 rosbag 播放结束后 launch 自动退出；
4. 状态全部集中在 `State.h`；
5. `Initializer` 位于顶层；
6. `observer::Propagator`、`observer::Updater`、`observer::ZuptUpdater` 独立；
7. `FeatureManager`、`StereoTriangulator`、`LandmarkMap`、`FeatureHealth` 独立；
8. `VioPipeline` 为纯 C++；
9. `GTMapping` 位于 `ros/`，由 `HnoVioNode` 持有，并独占现有 GT path 和
   `odom -> gt_base_link` TF；不新增独立 GT pose topic；
10. `OdomExport` 位于 pipeline，并包含 run_context 写入；
11. ROS 代码位于 `ros/`；
12. Diagnostics 集中；
13. `tools/run_rtabmap/` 未修改；
14. bias 未增加在线估计；
15. P 仍然 15x15；
16. smoke GT mapping 达到 strict，或达到 tolerance 且终端和摘要均打印精度退化警告；
17. 普通 VIO 最多三次中至少一次通过，未执行槽位在摘要中固定写为 `SKIPPED`；
18. topic 和 TF 语义不变；
19. 当前输出功能不丢失；
20. CMake 和安装规则完整。

第一版完成时 smoke 失败定义仍固定为：GT mapping 失败，或普通 VIO 三次全部失败。

---

# 13. 第一版之后的后续任务

以下任务不属于本计划，但重构应为其预留空间：

1. `BiasFeedbackEstimator`；
2. `bg` 工程反馈估计；
3. `ba` 工程反馈估计；
4. 21 维 bias covariance；
5. 更丰富的 diagnostics；
6. 前端 3D landmark 误差统计；
7. 替换 KLT；
8. 在线 `map -> odom` correction；
9. 局部优化或回环；
10. 发散原因专项分析。

第一版完成前禁止提前实现这些内容。

---

# 14. 最终执行口令

Codex 应严格遵循：

```text
先建立自动 smoke test。
然后小步迁移。
每次修改后立即编译和运行 smoke。
失败时停止扩展，只修复当前步骤。
不改数学，不调参数，不增加 bias，不动 RTAB-Map。
GT mapping 失败立即停止；普通 VIO 只有三次全部失败才判阶段失败。
```
