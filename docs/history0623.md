# HNO-VIO 当前进展汇报

日期：2026-06-23  
工作区：`/home/sharpa/hno_vio_ws`  
数据集：EuRoC `V1_01_easy`

## 1. 当前结论

本阶段完成了 HNO-VIO 独立 ROS1 工作区整理、`V1_01_easy` 参数扫描、结构漂移诊断、姿态可视化修正、HNO 输出接入 RTAB-Map 全局优化，以及围绕 `E/e_hat` 正交性和视觉 update 断流的重复排查。

核心结论：

- 当前工程稳定基线改为 `case18 + hard projection`：
  - `visual_noise_px=3.0`
  - `chi2_gate=12.0`
  - `delta_p_max=0.20`
  - `delta_r_max=0.10`
  - `enable_structure_projection=true`
- 重复实验显示，`case18_hard_projection` 3/3 可用，final ATE 中位数约 `0.738 m`；`case18_no_projection` 和 `case28_projection` 都只有 1/3 可用。
- `E/e_hat` 非正交是无投影路径发散的主要结构性因素之一：case18 no projection 的 tail `E_orth_frob` 约 `0.17`，而 hard projection 后约 `1e-4`。
- 但 `E` 非正交不是全部问题。case28 在 `E` 已正交的情况下仍可能发散，说明还存在独立的视觉 update / landmark 表达一致性问题。
- 历史 case28 曾给出 RTAB-Map 优化 APE RMSE `0.111747 m` 的最佳单次结果，但复跑稳定性不足，不能再作为默认工程基线。

## 1.1 当前主要困境

当前系统的困难主要集中在两个方面。

### 困境一：`E/e_hat` 非正交导致结构漂移

HNO 状态中维护辅助基向量：

```text
E = [e_hat_1, e_hat_2, e_hat_3]
```

理论上应满足：

```text
E^T E = I
det(E) = 1
```

但无结构投影时，视觉更新和传播会让 `E` 逐渐偏离正交基。重复实验中：

| group | usable | final ATE median | tail E orth max |
|---|---:|---:|---:|
| case18 no projection | 1/3 | 107.696 m | 0.172944 |
| case18 hard projection | 3/3 | 0.737917 m | 0.000147 |

这说明在 case18 参数下，`E/e_hat` 非正交不是一个轻微数值误差，而是直接影响稳定性的结构性问题。hard projection 明显提高成功率，并把 `E_orth` 压到 `1e-4` 量级。

当前表述应保持边界：

- 可以说：`E/e_hat` 非正交是无投影路径发散的主要结构性因素之一。
- 不应说：`E/e_hat` 非正交是系统所有发散的唯一原因。

### 困境二：视觉特征/update 断崖式降低

case28 的多轮复跑说明，即使 `E` 已经被 hard projection 控制住，系统仍可能发散。其典型链路是：

```text
状态误差开始增大
  -> 地图点重投影/视觉残差变差
  -> chi2 reject 大量增加
  -> accepted observations 快速下降
  -> 视觉更新不足，轨迹靠 IMU 积分漂移
  -> active_map / observations 进一步掉到接近 0
```

case28 失败与成功 run 的末段对比：

| run | final ATE | path ratio | last10 obs/frame | last10 accept |
|---|---:|---:|---:|---:|
| old_success | 1.242 m | 0.987 | 80.7 | 0.890 |
| ablation_fail | 80.794 m | 2.313 | 16.8 | 0.119 |

这说明特征点并不是简单地“提取不到”，而是状态、地图点和观测之间的残差逐渐失配，导致 update gating 把大量观测拒掉，最终出现视觉更新断流。

更深层的代码一致性风险：

- `HNOFeature` 当前生成的是世界系 landmark `info.p_w`。
- `HNOUpdater` 却把 `feature.xyz` 当作 `e_hat` 基下的 landmark 系数：

```cpp
pf_hat_I = p_i1 * state->e_hat[0]
         + p_i2 * state->e_hat[1]
         + p_i3 * state->e_hat[2];
```

如果 `feature.xyz` 实际是普通世界坐标，这里会变成用 `E * p_w` 重构 landmark，造成视觉 update 模型与建图坐标表达不一致。该问题可以解释：

- case28 在 `E` 已正交时仍会发散。
- `gt_mapping` 中视觉观测很多、接受率高，但轨迹仍可能严重发散。

因此下一步不能只调参数，必须统一 landmark 表达：

- 要么 `HNOFeature` 输出 HNO 公式需要的固定 landmark 系数 `p_ij`；
- 要么 `HNOUpdater` 直接使用世界系 `feature.xyz` 并重新推导对应 Jacobian。

## 2. 代码结构

当前工作区是独立 ROS1 catkin workspace：

```text
/home/sharpa/hno_vio_ws
├── AGENTS.md
├── hno_vio.md
├── src/hno_vio
│   ├── CMakeLists.txt
│   ├── include/hno_vio
│   │   ├── HNOFeature.h
│   │   ├── HNOInitializer.h
│   │   ├── HNOManager.h
│   │   └── HNOUpdater.h
│   ├── src
│   │   ├── HNOFeature.cpp
│   │   ├── HNOInitializer.cpp
│   │   ├── HNOManager.cpp
│   │   ├── HNOUpdater.cpp
│   │   └── run_hno_vio.cpp
│   ├── launch
│   │   ├── euroc_hno.launch
│   │   └── realsense_hno.launch
│   ├── config
│   ├── ground_truth/euroc_mav
│   ├── thirdparty/openvins_core
│   ├── debug
│   │   ├── current_stable_params.yaml
│   │   ├── scripts
│   │   ├── reports
│   │   └── runs
│   └── tools/rtabmap_wang
│       ├── ros2_ws/src/euroc_wang_replay
│       └── scripts
├── build
└── devel
```

主要模块职责：

- `HNOFeature`: 图像特征跟踪、双目匹配、前端统计输出。
- `HNOInitializer`: 初始化 IMU/视觉状态。
- `HNOManager`: ROS 数据流、状态传播、更新、输出和调试指标。
- `HNOUpdater`: HNO 更新、chi-square gate、增量截断、结构诊断。
- `run_hno_vio.cpp`: ROS1 节点入口，读 launch 参数，导出 odometry 和 metrics。
- `debug/scripts`: 参数扫描、单次评估、结构漂移报告、姿态检查脚本。
- `tools/rtabmap_wang`: ROS2 replay 节点，把 HNO 导出的 odom 和 EuRoC stereo 数据送入 RTAB-Map，再导出优化后 TF 轨迹做 evo 评估。

## 3. 当前参数固化

保守默认参数写入：

- `src/hno_vio/debug/current_stable_params.yaml`
- `src/hno_vio/launch/euroc_hno.launch`

当前工程默认：

```yaml
visual_noise_px: 3.0
chi2_gate: 12.0
delta_p_max: 0.20
delta_r_max: 0.10
enable_structure_projection: true
```

说明：这个配置不是原始 no-projection 数学路径，而是当前工程稳定基线。重复实验中 `case18_hard_projection` 3/3 可用，final ATE 中位数约 `0.738 m`。

原始数学路径对照仍保留：

```yaml
visual_noise_px: 3.0
chi2_gate: 12.0
delta_p_max: 0.20
delta_r_max: 0.10
enable_structure_projection: false
```

注意：无投影版本来自历史 sweep 的 case18，但最新复跑显示它不是可复现稳定版本，只适合作为论文/数学路径对照。

历史 RTAB-Map 单次最佳仍是 case28：

```yaml
visual_noise_px: 2.5
chi2_gate: 12.0
delta_p_max: 0.20
delta_r_max: 0.15
enable_structure_projection: true
```

但 case28 重复实验只有 1/3 可用，因此不再作为当前工程默认。

## 4. 参数扫描与结构漂移

完整 30 组自动验证源目录：

```text
src/hno_vio/debug/runs/V1_01_easy_auto_20260621_102310
```

结构漂移报告：

```text
src/hno_vio/debug/reports/structure_drift_20260621_131742
```

![结构漂移与轨迹对照](src/hno_vio/debug/reports/structure_drift_20260621_131742/structure_drift_vs_trajectory.png)

代表性结果：

| case | structure projection | final error | path ratio | aligned ATE RMSE | tail E orth max | 结论 |
|---|---:|---:|---:|---:|---:|---|
| case 18 | false | 0.547 m | 0.943 | 2.355 m | 0.175 | 历史 sweep 中无投影可用，但复跑不稳定 |
| case 28 | true | 0.721 m | 0.985 | 0.432 m | 0.000 | 当前全局优化输入首选 |

结构漂移判断：

- 无投影版本中 `E_orth_err` 长期保持约 `0.17` 量级，说明结构约束存在漂移。
- 显式结构投影版本 tail `E_orth_err=0`，全局尺度和对齐误差更好。
- 因此：论文/保守数学路径仍用 no-projection；工程上给 RTAB-Map/RTAB-SAM 的输入优先用 case 28。

## 5. Roll 角震荡结论

姿态检查报告：

```text
src/hno_vio/debug/reports/orientation_20260621_131815
```

![Unwrapped RPY](src/hno_vio/debug/reports/orientation_20260621_131815/orientation_unwrapped_rpy.png)

![姿态误差指标](src/hno_vio/debug/reports/orientation_20260621_131815/orientation_error_metrics.png)

结论：

- EuRoC GT 四元数文件头为 `qx qy qz qw`，当前 TUM 输出顺序不需要修改。
- 原始 roll 图的震荡主要来自 Euler 角在 `+-180 deg` 分支附近跳变。
- 后续不再用 raw wrapped roll 判断算法发散，应改用 unwrapped RPY、相对姿态误差角、gravity direction error。

## 6. RTAB-Map 全局优化结果

### 6.1 Case 28：历史单次最好结果

运行目录：

```text
src/hno_vio/debug/runs/V1_01_easy_case28_rtabmap_20260621_144413_full
```

输入参数：

```text
visual_noise_px=2.5
chi2_gate=12.0
delta_p_max=0.20
delta_r_max=0.15
enable_structure_projection=true
```

![case 28 RTAB-Map 轨迹](src/hno_vio/debug/runs/V1_01_easy_case28_rtabmap_20260621_144413_full/rtabmap_eval/traj_page1.png)

| trajectory | APE RMSE | APE mean | APE max | RPE RMSE | RPE mean | RPE max |
|---|---:|---:|---:|---:|---:|---:|
| raw HNO odom | 0.466510 | 0.427491 | 0.874740 | 0.020841 | 0.017266 | 0.084593 |
| RTAB-Map optimized | 0.111747 | 0.104178 | 0.308494 | 0.024630 | 0.018096 | 0.378792 |

解释：

- RTAB-Map 将 APE RMSE 从 `0.466510 m` 降到 `0.111747 m`。
- RPE RMSE 略升，但仍保持在 `0.025 m` 量级。
- 这是历史单次最佳可汇报结果，但不再作为当前默认工程基线。后续重复实验显示 case28 projection 只有 1/3 可用，主要受视觉 update 断流和 landmark 表达一致性问题影响。

### 6.2 Case 18：复跑到可用后的结果

复跑目录：

```text
src/hno_vio/debug/runs/V1_01_easy_case18_repeat_20260621_151554
```

选中轨迹：

```text
src/hno_vio/debug/runs/V1_01_easy_case18_repeat_20260621_151554/attempt_07
```

case 18 参数：

```text
visual_noise_px=3.0
chi2_gate=12.0
delta_p_max=0.20
delta_r_max=0.10
enable_structure_projection=false
```

复跑结果：

| attempt | final error | path ratio | aligned ATE RMSE | usable |
|---:|---:|---:|---:|---|
| 1 | 237.710 m | 4.961 | 34.815 m | false |
| 2 | 135.510 m | 3.223 | 17.560 m | false |
| 3 | 163.785 m | 3.713 | 22.771 m | false |
| 4 | 113.108 m | 2.840 | 14.427 m | false |
| 5 | 296.813 m | 5.971 | 44.809 m | false |
| 6 | 265.275 m | 5.437 | 38.809 m | false |
| 7 | 0.716 m | 0.956 | 2.450 m | true |

复跑表中的 `status=124` 来自外层 `timeout 240s` 包装；各 attempt 的 odom CSV 均写到完整目标时间，评估指标可用。

![case 18 attempt 7 RTAB-Map 轨迹](src/hno_vio/debug/runs/V1_01_easy_case18_repeat_20260621_151554/attempt_07/rtabmap_eval/traj_page1.png)

RTAB-Map 评估：

| trajectory | APE RMSE | APE mean | APE max | RPE RMSE | RPE mean | RPE max |
|---|---:|---:|---:|---:|---:|---:|
| raw HNO odom | 1.226318 | 1.108373 | 2.063399 | 0.020802 | 0.017754 | 0.088621 |
| RTAB-Map optimized | 1.331510 | 1.102424 | 2.908127 | 0.068696 | 0.019888 | 3.086013 |

解释：

- case 18 可以偶发生成无投影可用轨迹，但复现性不足。
- 对 selected attempt 7，RTAB-Map 没有改善 APE RMSE，反而从 `1.226318 m` 增加到 `1.331510 m`。
- 因此 case 18 适合作为“保守数学路径仍可跑通”的对照，不适合作为当前全局优化输入首选。

## 7. 构建和运行方式

ROS1 HNO-VIO 构建：

```bash
sudo docker exec hno_slam_ros bash -lc "cd /home/sharpa/hno_vio_ws && source /opt/ros/noetic/setup.bash && catkin_make"
```

运行 EuRoC V1_01_easy，真实 mapping 模式：

```bash
sudo docker exec hno_slam_ros bash -lc "cd /home/sharpa/hno_vio_ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && roslaunch hno_vio euroc_hno.launch bag_path:=/home/sharpa/datasets/euroc/ROSbag/V1_01_easy.bag use_gt_mapping:=false rviz:=false"
```

运行当前工程默认参数（case18 + hard projection，launch 默认已同步）：

```bash
sudo docker exec hno_slam_ros bash -lc "cd /home/sharpa/hno_vio_ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && roslaunch hno_vio euroc_hno.launch bag_path:=/home/sharpa/datasets/euroc/ROSbag/V1_01_easy.bag use_gt_mapping:=false rviz:=false visual_noise_px:=3.0 chi2_gate:=12.0 delta_p_max:=0.20 delta_r_max:=0.10 enable_structure_projection:=true"
```

## 8. 当前问题和下一步

当前明确问题：

- `E/e_hat` 非正交是 no-projection 路径的主要结构性发散因素之一。case18 no projection 重复实验只有 1/3 可用，而 case18 hard projection 3/3 可用。
- 视觉 update 仍存在断崖式降低问题。case28 即使 `E` 已保持正交，也会因 observations / accepted 大幅下降进入 IMU dead reckoning。
- landmark 表达存在一致性风险：`HNOFeature` 输出世界系 `p_w`，`HNOUpdater` 却按 `e_hat` 基系数解释 `feature.xyz`。
- 历史 sweep 中的 no-projection case18 不应再描述为“稳定可复现”，应描述为“历史可用无投影对照”。
- 历史 case28 RTAB-Map 最佳结果不应再描述为“当前默认工程参数”，应描述为“单次最佳但复跑不稳定”。
- 当前工作树有大量未提交源码和构建产物修改，若要严格复现实验，需要先固定 git commit / diff / binary。

建议下一步：

- 优先固定当前源码状态，清理 build/devel 产物和实验报告目录的版本边界。
- 将当前工程默认固定为 `case18 + hard projection`，用于后续稳定复现实验。
- 保留 `case18 no projection` 作为论文/原数学路径对照，用来展示 `E/e_hat` 非正交的影响。
- 优先修正或重新推导 landmark 表达：统一 `feature.xyz` 是世界坐标还是 `e_hat` 基系数。
- 在修正 landmark 表达后，再重新评估 case28 和 RTAB-Map/RTAB-SAM 输入。
- 对视觉断流做更细诊断：记录 projected-in-image、out-of-bounds、behind-camera、depth reject、chi2 residual 分布和 track 生命周期。
