# HNO-VIO 视觉发散修复记录

记录日期：2026-07-07  
工作区：`/home/sharpa/hno_vio_clean`  
包类型：当前 checkout 实际为 ROS2 / `ament_cmake`，不是旧文档中的 catkin workspace。  
主要数据集：EuRoC `V1_01_easy`

## 1. 本轮目标

用户反馈：上一轮“非破坏式视觉恢复”改动后，系统仍然在同一位置发散，甚至速度更快。

本轮目标不是继续调参，而是验证上一轮思路是否成立，并把失败路径收敛成明确结论：

- 保留“不能清空 KLT / `feature_db` / `history_obs`”这个约束。
- 验证 updater 整帧事务是否能阻止低质量视觉更新污染状态。
- 验证单点 landmark association reset 是否能恢复尾段视觉约束。
- 验证全局失配时是否应该 reset、recovery update、或者隔离旧视觉。

## 2. 已实现过的核心改动

### 2.1 非破坏式 3D 关联语义

`FeatureInfo` 增加：

```cpp
bool world_point_valid;
int association_cooldown;
```

语义拆分：

- KLT 二维 track 生命周期仍由 tracker 和 `history_obs` 维护。
- `track_count` 只表示当前 3D association 的成熟度。
- `world_point_valid=false` 表示该 KLT ID 仍存在，但旧 `p_w` 不得用于 updater 和 active map。
- `resetLandmarkAssociation(id)` 只清理该 ID 的 3D 关联：
  - `p_w=0`
  - `world_point_valid=false`
  - `track_count=0`
  - `fail_count=0`
  - `association_cooldown=3`
- 不执行 `feature_db.erase(id)`。

### 2.2 Health guard 改为只报告

原先 health degraded 时会清空 `observations`，这会造成尾段长期 `klt>0, depth>0, obs=0`。

已改成：

- 保留 `health_degraded` / `health_streak`。
- 保留周期日志。
- 不再因为 health degraded 清空 observations。
- `HNOVisualDiag` 增加前端和 updater 统计。

### 2.3 Updater 整帧事务

`HNOUpdater::update()` 从 `void` 改为返回 `HNOUpdateReport`。

流程：

- 非空 observation frame 更新前深拷贝完整 `HNOState`。
- 保持原 sequential point update、chi2 gate、NaN guard、单点 delta 截断。
- 帧结束后只有满足：
  - `accepted_count >= update_min_accepted_observations`
  - `accepted_ratio >= update_min_accepted_ratio`
  - 状态和协方差有限
  才提交。
- 否则只恢复 `HNOState`，不恢复 KLT / `feature_db` / `history_obs`。

当前配置：

```yaml
update_min_accepted_observations: 5
update_min_accepted_ratio: 0.3
```

## 3. 已验证失败的实验路径

### 3.1 Recovery observations：失败

尝试过的设计：

- 当 mature landmark 出现 `map_jump` 或 reprojection failure 时，不立刻 reset。
- 生成只含左目 bearing 的 recovery observation，继续使用旧 `p_w`。
- 在 updater 中对 recovery observation 放宽 chi2 gate。
- 低接受率帧允许 recovery commit，只要残差改善且 frame delta 有界。

测试结果：

- 单次运行可能看起来压住，例如终点约 `Pos:3.57 2.70 -0.29`。
- 复跑不稳定，另一轮出现 `Pos:76.32 50.61 -5.97`。
- recovery observation 帧数可达数百帧，说明它不是短暂恢复，而是在长时间把旧伪地标继续喂给 updater。

结论：

- recovery observations 会把错误锚点重新注入状态。
- 放宽 chi2 和 recovery commit 是危险路径。
- 这条实验应撤回，不能作为最终修复。

### 3.2 全帧 delta 硬门控：失败

尝试过的设计：

- 对整帧视觉更新增加最大 `delta_p / delta_v / basis_delta` commit gate。
- 超界则整帧 rollback。

测试结果：

- 会拒绝必要修正，导致更早失锁。
- 曾出现终点 `Pos:1438 514 -156` 量级的严重发散。

结论：

- 硬拒绝整帧 correction 会让系统在需要视觉拉回时直接退化成 IMU 积分。
- 不应保留为主修复策略。

### 3.3 固定 landmark、不平滑 `p_w`：失败

尝试过的设计：

- 去掉或显著减少老点 `p_w` 的平滑更新，试图避免地图被当前漂移状态污染。

测试结果：

- 中段即严重发散，约 frame 1900 时达到 `Pos:50 -68 -12` 量级。

结论：

- 当前系统的 pseudo-landmark 不是独立 SLAM landmark state。
- 完全固定 `p_w` 会让旧伪地图和状态更快不一致。

### 3.4 全局失配隔离旧视觉：本轮失败

本轮最后尝试的设计：

- 若本帧待 reset landmark 数量超过 `feature_max_association_resets_per_frame`，认为这是 global mismatch。
- 不批量 reset。
- 清空本帧 observations。
- 进入 `feature_global_mismatch_hold_frames` 帧 quarantine。
- quarantine 内保留 KLT / history / feature entry，但旧 valid landmark 不参与 updater，也不累计新的 reset failure。

当前代码树仍包含该实验：

```yaml
feature_max_association_resets_per_frame: 2
feature_global_mismatch_hold_frames: 5
```

验证命令：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/hno_ros_log timeout 240s ros2 launch hno_vio hno_vio.launch.py \
  dataset:=V1_01_easy \
  bag_path:=/home/sharpa/datasets/euroc/ros2db/V1_01_easy_db \
  bag_rate:=3.0 \
  rviz:=false \
  run_preprocess:=false \
  export_odom:=false \
  use_gt_mapping:=false \
  try_zupt:=true
```

运行结果：

- 构建通过。
- launch 能启动，但 ROS middleware 在沙箱内有 UDP socket permission warning；bag 仍能读取并播放。
- 运行中约 frame 1400 左右已经进入严重发散：

```text
frame 1442: mature 0 obs 0, association_reset_deferred=50, quarantine=5
Pos:1108.90 1157.64 -173.73

frame 1470: mature 0 obs 0, health_streak=1294
Pos:1161.58 1205.93 -181.57

frame 2228: mature 0 obs 0, association_reset_deferred=67, quarantine=5
Pos:2884.48 2960.72 -464.84
```

结论：

- “隔离旧视觉”会避免 recovery 错锚点，但也会让系统长期无视觉约束。
- 由于当前 IMU 初始化 / bias / gravity 对齐无法单独长时间稳定，状态会被惯导积分快速放大。
- 这条策略失败得更早，不应作为最终修复。

## 4. 当前更可靠的事实

### 4.1 爆炸不是单纯尾段问题

本轮 smoke 显示：

- 到 frame 1400 左右已经是千米级发散。
- `health_streak` 已经超过 1200，说明 health degraded 从很早开始持续存在。
- 尾段看到的发散只是长期无有效视觉约束后的结果，不是唯一触发点。

下一轮应优先找第一次从：

```text
mature > 0, obs > 0
```

掉到：

```text
mature = 0, obs = 0
```

的帧，而不是继续在尾段加 recovery patch。

### 4.2 Updater 事务本身是必要但不充分的

事务 rollback 能防止低接受率帧直接提交坏状态，例如：

```text
accepted_count < 5
accepted_ratio < 0.3
=> committed=0, rollback=insufficient_accepted_*
```

但如果前端长期不给有效 observations，事务机制不会产生任何修正，状态仍会通过 IMU propagation 发散。

### 4.3 当前 pseudo-map 不是可用于重定位的固定地图

当前 `feature_db` 中的 `p_w` 是由当前估计状态和双目深度不断生成/平滑出来的伪世界点。

因此：

- 旧 `p_w` 不能当成全局可靠 anchor。
- global mismatch 时用旧 `p_w` 强行 recovery update 会错拉状态。
- global mismatch 时完全不用旧 `p_w` 又会失去视觉约束。

这说明仅靠 `feature_db` reset / cooldown / recovery 不能解决本质问题。

## 5. 当前代码状态

当前工作树包含以下中间改动：

- `include/hno_vio/HNOFeature.h`
- `src/HNOFeature.cpp`
- `include/hno_vio/HNOUpdater.h`
- `src/HNOUpdater.cpp`
- `src/HNOManager.cpp`
- `config/euroc_mav/estimator_config.yaml`
- `docs/construct.md`

其中：

- 非破坏式 association reset、world_point_valid、cooldown、diagnostic report 是相对合理的基础改动。
- updater 整帧事务是相对合理的基础改动。
- 本轮新增的 `association_quarantine_remaining` / `feature_global_mismatch_hold_frames` 已被 smoke 证明失败，不应直接交付。
- 之前的 recovery observations / recovery commit 已从代码中撤掉，但若未来在 diff 或历史里看到，不应恢复。

构建验证：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select hno_vio --event-handlers console_cohesion+
```

结果：

```text
Summary: 1 package finished
```

不适用命令：

```bash
catkin_make
```

当前环境没有 `catkin_make`，且该 checkout 的 `package.xml` 声明为 `ament_cmake`。

## 6. 建议下一步

### 6.1 先回退失败实验

建议回退或禁用：

- `association_quarantine_remaining`
- `feature_global_mismatch_hold_frames`
- quarantine 内清空 observations / 跳过旧 landmark updater 的逻辑

保留：

- `world_point_valid`
- 单点 `resetLandmarkAssociation(id)`
- cooldown
- health 只报告
- updater 整帧事务
- `HNOVisualDiag`

### 6.2 定位首次失锁帧

下一轮不要直接全量跑到尾段，应先做日志提取：

```bash
rg 'HNOVisualDiag' <run-log> \
  | sed -E 's/.*frame ([0-9]+).*klt ([0-9]+).*depth ([0-9]+).*mature ([0-9]+).*obs ([0-9]+).*accepted ([0-9]+).*committed ([01]).*association_reset=([0-9]+),association_reset_deferred=([0-9]+).*/\1 \2 \3 \4 \5 \6 \7 \8 \9/' \
  | awk '$4==0 || $5==0 || $7==0 {print}'
```

目标是找：

- 第一次 `mature` 明显下降的帧。
- 第一次 `obs` 从正常值掉到 0 的帧。
- 第一次 `accepted_ratio` 持续低于 0.3 的帧。
- 第一次大量 `map_jump` / `reprojection` 的帧。

### 6.3 修复方向应转向前端建图语义

当前更可能的问题不是“某个 landmark reset 策略不够好”，而是：

- pseudo-landmark 使用当前漂移状态生成，缺少独立 landmark state。
- 新旧 KLT track 的 2D 生命周期和 3D association 生命周期刚被拆开，但成熟/初始化策略仍可能让地图在某一段整体失效。
- updater rollback 后，前端仍可能基于 rollback 前后不一致的状态语义继续维护地图。

更可行的方向：

- 在 updater rollback 帧，不要让基于该帧状态刷新出来的 `p_w` 成为后续地图事实。
- 将“更新 old landmark `p_w`”延迟到 updater commit 后，或者至少在 rollback 后撤销本帧对 `p_w` 的平滑刷新。
- observation 构造和 landmark maintenance 分成两个阶段：
  1. 用上一 committed map 构造 observations。
  2. updater commit 后再提交本帧 map maintenance。
  3. updater rollback 时只丢弃本帧 map maintenance，不动 KLT/history。

这个方向比 recovery observations 或 quarantine 更符合当前失败证据。

