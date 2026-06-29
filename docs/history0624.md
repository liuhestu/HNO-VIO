# HNO-VIO 调试历史与下一步路线

记录日期：2026-06-24  
目标：尽快得到一版 EuRoC `V1_01_easy` 上真实 VIO 收敛的 HNO-VIO 轨迹，供 RTAB-Map 使用。

## 1. 总目标

最终需要的是一条可供 RTAB-Map 优化的稳定 odometry/trajectory。

当前约束：

- `use_gt_init=false`
- `use_gt_mapping=false`
- GT 只能用于离线评分，不能作为最终初始化或建图方案
- 不优先追求完整日志系统，先追求 HNO-VIO 本体收敛
- 不应只看终点欧氏误差，应重点看轨迹形状、局部连续性和后段是否发散

## 2. 已知历史提交

仓库：

`https://github.com/liuhestu/HNO-VIO.git`

关键提交：

- `92f44cf 2026-06-22 12:54:40 +0800 case 18,28跑通`
- `94e170b 2026-06-23 02:34:14 +0800 e非正交分析`
- `aafcfe1 2026-06-23 02:35:29 +0800 阶段总结文档`

曾经认为 `92f44cf` 是第一次自动化运行之后、GitHub 上已有的可恢复版本。但 clean checkout 后无法复现历史成功。

## 3. 历史成功证据

旧工作区中存在历史 run：

`/home/sharpa/hno_vio_ws/src/hno_vio/debug/runs/e_projection_repeat_20260622_083139`

其中 `case18_hard_projection` 三次重复均成功：

| run | final_path_length_ratio |
| --- | ---: |
| `round1_case18_hard_projection` | 0.988483 |
| `round2_case18_hard_projection` | 0.991132 |
| `round3_case18_hard_projection` | 0.986003 |

三次日志中可见参数一致：

- `dataset=V1_01_easy`
- `use_gt_init=false`
- `use_gt_mapping=false`
- `export_odom=false`
- `visual_noise_px=3.0`
- `chi2_gate=12.0`
- `delta_p_max=0.2`
- `delta_r_max=0.10`
- `enable_structure_projection=true`

因此，历史上 `case18 + 强制结构化` 确实出现过 3/3 收敛，不太像纯随机偶然。

## 4. 当前复现失败情况

在 clean `92f44cf` 和后续 `94e170b` 路线中，使用相同可见参数仍然失败：

- 起步误差明显偏大
- 轨迹很快乱拐
- 中段可能 `Feat` 掉到接近 0
- 路径长度比例明显爆炸
- 有时 update accept ratio 并不低，说明不是简单 chi2 门限过严导致全拒绝

结论：

历史成功很可能依赖 GitHub 提交之外的条件，例如：

- 未提交源码改动
- 旧 `build/devel` 二进制
- 当时的 workspace/source 顺序
- 特定 launch/config 副本
- 隐含脚本参数
- 或某个初始化/前端非确定性条件

因此，不应继续假设 clean main 历史提交必然能复现成功。

## 5. 工作区混乱情况

`/home/sharpa/hno_vio_ws` 曾被多次改造：

- 既像 catkin workspace，又像 package repo
- 出现过外层 repo、内层 `src/hno_vio` repo、软链接 workspace 混用
- `/home/sharpa/hno_main_ws/src/hno_vio` 曾是软链接，指向 `/home/sharpa/hno_vio_ws`
- `hno_main_ws/devel` 中存在旧二进制，但 source 后会污染 ROS package path
- ROS package 路径曾扫到 `tools/rtabmap_wang/ros2_ws`，导致 ROS2 package 的 `package.xml` email 非法错误

建议：

不要继续在旧 `/home/sharpa/hno_vio_ws` 上排查。应新建干净工作区，或直接进入路线二的 HNO-only 分支。

## 6. 路线一结论

路线一：

重新建立干净 workspace，clone main，checkout `92f44cf` 或 `94e170b`，尝试复现历史成功。

判断：

路线一已经基本到止损点。如果满足以下条件仍失败，应停止：

- `git show` 确认是目标提交
- `git status --short` 无可疑改动
- 删除 `build/ devel/` 后 clean build
- `rospack find hno_vio` 指向当前 clean workspace
- 跑 `case18 + 强制结构化 + export_odom=false`
- 至少两次完整 run 仍明显发散

当前判断：

GitHub 上 clean main 历史提交不足以复现历史成功。继续在 main/RTAB-Map/复杂日志环境中盲调参数，收益很低。

## 7. 建议执行路线二

路线二：

拉取或切换到更小的 `hno_vio` 分支，在 HNO-only 环境中调试最小 VIO 系统。

目标从“复现历史参数”改成：

找出真实 VIO 为什么从起步阶段就走错，并让 `V1_01_easy` 收敛。

优先排查：

- 初始化是否错误
- IMU/camera 坐标系是否反了
- camera-IMU 外参方向是否用反
- 时间戳或同步是否异常
- stereo feature / triangulation 是否生成错误结构
- HNOFeature 输出是否稳定
- HNOManager 中 feature/map/state 传递是否错误
- update 虽然 accepted，但残差方向或观测模型符号是否错

## 8. 代码修改约束

强约束：

不要修改：

- `src/HNOUpdater.cpp`
- `src/HNOPropagator.cpp`

优先允许检查/修改：

- `HNOFeature`
- `HNOManager`
- initializer
- launch
- config
- debug scripts

如果发现必须动 updater/propagator，应先明确证据和原因，不要直接改。

## 9. 路线二建议流程

不要一开始做大规模 sweep。先定位发散机制。

建议步骤：

1. 30 秒 smoke，记录每帧：
   - `Feat`
   - `Err`
   - `Pos`
   - `Vel`
   - accepted/rejected
2. 判断发散类型：
   - 如果一开局 `Err` 就大、方向乱拐：优先查初始化、外参、坐标系、时间戳
   - 如果前 30 秒稳定、后面 `Feat` 掉到 0：优先查 HNOFeature、tracking、reproj/stereo gating、地图管理
   - 如果 `Feat` 很多、accepted 很高但轨迹发散：优先查观测构造、残差符号、坐标变换、结构投影接入
3. 每次只改一个小变量。
4. 先跑 30 秒 smoke，再跑 180 秒 full。
5. 如果 `export_odom=true` 影响稳定，先用 `export_odom=false` 获得内部收敛轨迹，再离线导出给 RTAB-Map。

## 10. 评价指标

不要只用 HNO 当前打印的简单欧氏距离判断收敛。路线二应先补离线评估脚本，GT 只用于评分。

最低限度必须输出：

- `duration_sec`
- `odom_rows` 或 `hno_rows`
- `final_position_error`
- `path_length`
- `gt_path_length`
- `path_length_ratio`

轨迹形状指标：

- `aligned_ate_rmse`
- `aligned_ate_mean`
- `tail_ate_mean`
- `tail_ate_slope`
- `direction_cosine_mean`
- `direction_cosine_median`

局部运动指标：

- `rpe1_trans_rmse`
- `rpe1_rot_rmse`
- `step_length_ratio_mean`
- `step_length_ratio_median`

建议稳定门限：

- `duration_sec >= 130`
- `0.8 <= path_length_ratio <= 1.2`
- `aligned_ate_rmse <= 1.5`
- `rpe1_trans_rmse <= 0.5`
- `tail_ate_slope <= 0.02`
- `direction_cosine_median >= 0.6`

RTAB-Map 更关心：

- 局部 odom 连续
- 尺度正常
- 方向不乱跳
- 后段不持续发散

不要把 `final_position_error` 当唯一目标。

## 11. 建议给新 Codex 的开场

可以直接发：

```text
工作目录是新的 hno_vio 分支/HNO-only 工作区。目标是在最小 HNO-VIO 环境里让 EuRoC V1_01_easy 的真实 VIO 收敛，先不接 RTAB-Map，不做复杂日志系统。

main 历史提交 92f44cf 和 94e170b clean build 后都无法复现历史 e_projection_repeat 的 case18_hard_projection 三次成功结果。因此不要再假设历史 Git 提交本身足够复现成功。请把问题当成 HNO-VIO 最小系统稳定性调试。

强约束：不要修改 src/HNOUpdater.cpp 和 src/HNOPropagator.cpp。优先检查 HNOFeature、HNOManager、initializer、launch、config、debug scripts。

GT 只能用于离线评分，不能用于初始化或建图。请先补离线轨迹评估，至少计算 duration、path_length_ratio、aligned_ate_rmse、rpe1_trans_rmse、tail_ate_slope、direction_cosine_median。

调试重点不是盲扫参数，而是解释为什么起步阶段误差大、轨迹乱拐并发散。若一开局方向错，优先查坐标系/外参/初始化/时间戳；若前段正常但中段掉特征，查 HNOFeature 和地图管理；若特征和 accepted 都很多但状态发散，查观测构造、残差方向和坐标变换。
```

