# HNO-VIO / RTAB-Map 进展记录

记录日期：2026-06-25
工作区：`/home/sharpa/hno_vio_clean`
分支：`src/hno_vio` 内的 `hno_vio_rtabmap`
数据集：EuRoC `V1_01_easy`

## 1. 今日总目标

今天的重点从 HNO-VIO 本体调参转向 RTAB-Map 离线评估链路整理：

- 固化 HNO-VIO 保存 odometry 的 run 目录格式。
- 自动生成 RTAB-Map replay 所需 `run_context.json`。
- 打通 `run_rtabmap.sh` 一键离线 pipeline。
- 修正 RPY 图中 `+/-180 deg` 分支造成的视觉误判。
- 提高 RTAB-Map optimized graph 的节点密度，减少优化轨迹折线感。
- 整理文档、脚本头部说明、Git 忽略规则和推送前大文件清理策略。

当前主流程已经转为：

```text
roslaunch hno_vio euroc_hno.launch export_odom:=true
-> results/run_YYYYmmddTHHMMSS/
-> tools/run_rtabmap/scripts/run_rtabmap.sh <run>/vio_results/odom_raw.csv
-> offline_results/
```

## 2. HNO-VIO Run 输出格式固化

`euroc_hno.launch` 的 odom export 输出目录固定为：

```text
src/hno_vio/results/run_YYYYmmddTHHMMSS/
  run_context.json
  vio_results/
    odom_raw.csv
    odom_raw.txt
```

关键变更：

- `run_id` 仍使用 `run_YYYYmmddTHHMMSS`。
- `run_id` 明确按北京时间 / UTC+8 生成，避免 Docker 默认 UTC 导致目录时间少 8 小时。
- 仅当 `odom_output_path` 包含 `{run_id}` 时替换。
- `odom_raw.csv` 使用 `timestamp_ns,tx,ty,tz,qx,qy,qz,qw`。
- `odom_raw.txt` 使用 evo/TUM 格式。

`euroc_hno.launch` 新增传入节点的上下文参数：

```text
dataset
bag_path
euroc_mav0
```

保持已有参数：

```text
path_gt
odom_output_path
export_odom
```

未暴露为 launch 参数、直接固定写入 context 的字段：

```text
odom_frame=odom
base_frame=base_link
camera_left_frame=cam0_rect
camera_right_frame=cam1_rect
odom_semantic=T_odom_base
```

## 3. `run_context.json`

当前 JSON 示例：

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

生成策略：

- `dataset` 优先来自 launch 参数；为空时从 `bag_path` 文件名推断。
- `euroc_mav0` 来自 launch 参数。
- `ground_truth_tum` 来自 `path_gt`。
- CSV/TXT 输出文件都成功打开后才写 `run_context.json`。
- 写 JSON 失败只 `ROS_WARN`，不阻塞 VIO 运行。

验证过短启动：

```bash
docker exec hno_slam_ros bash -lc \
  'cd /home/sharpa/hno_vio_clean && source /opt/ros/noetic/setup.bash && source devel/setup.bash && timeout 5s roslaunch hno_vio euroc_hno.launch dobag:=false rviz:=false'
```

结果：能创建 `run_context.json`，字段完整。

## 4. RTAB-Map 离线 Pipeline 当前状态

主入口：

```bash
cd /home/sharpa/hno_vio_clean
src/hno_vio/tools/run_rtabmap/scripts/run_rtabmap.sh \
  src/hno_vio/results/run_YYYYmmddTHHMMSS/vio_results/odom_raw.csv
```

也支持传入目录：

```bash
src/hno_vio/tools/run_rtabmap/scripts/run_rtabmap.sh \
  src/hno_vio/results/run_YYYYmmddTHHMMSS/vio_results
```

`run_rtabmap.sh` 会自动删除并重建：

```text
<run>/offline_results/
```

不需要手动删除旧结果。

执行顺序：

```text
00_check_inputs.sh
01_prepare_replay.sh
02_build_ros2_ws.sh       可用 --skip-build 跳过
03_record_input_bag.sh
04_run_rtabmap.sh
05_export_and_eval.sh
```

输出布局：

```text
offline_results/
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
```

`rtabmap_input.bag/`、`rtabmap_output.bag/` 和 `rtabmap.db` 已加入 `.gitignore`。

## 5. RTAB-Map 输出语义

当前 optimized trajectory 的主输出为：

```text
offline_results/odom_optimized.txt
```

来源：

```text
/rtabmap/mapData.graph.poses
```

原因：

- `map -> odom` TF 在当前 pipeline 中通常接近 identity。
- dense `map->odom * odom->base_link` 不能代表实际 graph 优化效果。
- 真正有全局优化效果的是 RTAB-Map final graph poses。

因此报告中应表述为：

```text
The RTAB-Map optimized trajectory is exported from the final optimized graph in /rtabmap/mapData.graph.poses.
```

## 6. RPY 图修正

之前问题：

- roll 接近 `+180/-180` 时，曲线被画到纵坐标轴两端。
- `evo_traj_gt_raw_optimized.pdf` 中 evo 自带 RPY 页仍会按 `+/-180 deg` 包络显示。

今日修正：

- 在 `tf_utils.py` 增加 `angle_diff_degrees()`。
- 在 `eval_raw_vs_optimized.py` 中：
  - 每条 RPY 曲线先按时间 unwrap。
  - optimized / GT 再投到 raw 最近时间戳对应的同一 360 度分支。
  - CSV 和 PDF 使用同一套 branch-safe 数据。

使用规则：

```text
evo_traj_gt_raw_optimized.pdf:
  保留 evo 的 xyz component 页面，适合看 x/y/z 分量。
  其中 evo 自带 RPY 页不做分支修正。

logs/rpy_raw_vs_optimized.pdf:
  branch-safe RPY 图，讨论 roll/pitch/yaw 时使用这个。
```

已验证：

- 合成用例 `raw=+179 deg`、`optimized/GT=-179 deg` 被放到同一角度分支。
- `run_20260625T211722` 的 RPY CSV 中 roll 不再分布在 `+180/-180` 两端。

## 7. RTAB-Map Graph 密度调参

观察到的问题：

```text
raw poses:       2362 over 142.8 s, about 16.5 Hz
optimized poses: 115 over 142.0 s, about 0.8 Hz
optimized median step: about 0.44 m
optimized max step:    about 1.27 m
```

结论：

```text
优化后曲线不平滑、误差大处像折线，主要来自 RTAB-Map graph pose 太稀疏，而不是绘图层问题。
```

今日修改 RTAB-Map 默认参数：

```text
RTABMAP_DETECTION_RATE=5
RTABMAP_LINEAR_UPDATE=0.03
RTABMAP_ANGULAR_UPDATE=0.03
RTABMAP_CREATE_INTERMEDIATE_NODES=true
```

对应 RTAB-Map 参数：

```text
Rtabmap/DetectionRate
RGBD/LinearUpdate
RGBD/AngularUpdate
Rtabmap/CreateIntermediateNodes
```

可运行时覆盖：

```bash
RTABMAP_DETECTION_RATE=10 RTABMAP_LINEAR_UPDATE=0.02 \
RTABMAP_ANGULAR_UPDATE=0.02 \
src/hno_vio/tools/run_rtabmap/scripts/run_rtabmap.sh \
  src/hno_vio/results/run_YYYYmmddTHHMMSS/vio_results/odom_raw.csv
```

`export_optimized_tf.py` 新增 graph 密度统计：

```text
graph_final_pose_count
graph_final_duration_sec
graph_final_mean_hz
graph_final_median_dt_sec
graph_final_max_dt_sec
graph_final_median_step_m
graph_final_max_step_m
```

下一次 rerun 后，应重点检查 `offline_results/logs/export_report.txt`。

## 8. 文档与脚本整理

今日更新文档：

- `src/docs/hno_rtabmap_plan.md`
- `src/docs/wang algorithm.md`
- `AGENTS.md`
- `src/hno_vio/tools/run_rtabmap/README.md`

`wang algorithm.md` 修复了 Markdown 编译风险：

- 内联公式从 `$$...$$` 改成 `$...$`。
- 块级公式 `$$` 独占一行。
- 编号段落改为标准标题。

`hno_rtabmap_plan.md` 更新为当前实际流程：

- `results/run_*/run_context.json`
- `run_rtabmap.sh` 主入口
- `offline_results/` 输出契约
- RPY/evo 图使用规则
- RTAB-Map graph 密度调参

所有 `tools/run_rtabmap/scripts/` shell 脚本开头补充了：

```text
Usage
Inputs
Outputs
Notes
```

ROS 2 console entry Python 文件也补充了模块级说明：

- `prepare_odom.py`
- `replay_node.py`
- `export_optimized_tf.py`
- `eval_raw_vs_optimized.py`
- `rtabmap_diagnostics.py`

## 9. Git 与大文件处理

`.gitignore` 已包含：

```gitignore
rtabmap_input.bag/
rtabmap_output.bag/
rtabmap.db
```

今日 push 曾因 GitHub 100 MB 限制失败：

```text
results/run_20260625T211722/offline_results/rtabmap.db is 106.31 MB
```

原因：

- `.gitignore` 只能阻止后续新增。
- `rtabmap.db` 已进入本地 commit 历史，必须从历史中移除。

已执行的方向：

```bash
git filter-branch --force --index-filter \
  'git rm -r --cached --ignore-unmatch results/*/offline_results/rtabmap.db' \
  --prune-empty -- hno_vio_rtabmap
```

当前检查：

```bash
git ls-files 'results/*/offline_results/rtabmap.db'
```

应无输出。

若远端因历史重写拒绝普通 push，需要：

```bash
git fetch origin hno_vio_rtabmap
git push --force-with-lease origin hno_vio_rtabmap
```

如果确认要覆盖远端当前分支，可使用：

```bash
git push --force origin hno_vio_rtabmap
```

本地若仍有旧历史引用，`git log --all -- results/*/offline_results/rtabmap.db` 会继续看到旧 commit。可清理：

```bash
git update-ref -d refs/original/refs/heads/hno_vio_rtabmap
git branch -D backup-before-remove-rtabmap-db
git reflog expire --expire=now --all
git gc --prune=now
```

## 10. 当前待办

短期优先级：

1. 用新 RTAB-Map 密度参数 rerun：

   ```bash
   cd /home/sharpa/hno_vio_clean
   src/hno_vio/tools/run_rtabmap/scripts/run_rtabmap.sh \
     src/hno_vio/results/run_20260625T211722/vio_results/odom_raw.csv
   ```

2. 检查：

   ```text
   offline_results/logs/export_report.txt
   graph_final_mean_hz
   graph_final_max_step_m
   graph_final_pose_count
   ```

3. 判断 optimized curve 是否仍明显折线。

4. 如果仍稀疏，继续提高：

   ```text
   RTABMAP_DETECTION_RATE
   ```

   或降低：

   ```text
   RTABMAP_LINEAR_UPDATE
   RTABMAP_ANGULAR_UPDATE
   ```

5. 推送前确认：

   ```bash
   git ls-files 'results/*/offline_results/rtabmap.db'
   git log --all -- 'results/*/offline_results/rtabmap.db'
   ```

   目标是两者都无输出，或至少当前可推送历史中不包含大 DB。

## 11. 当前结论

截至 2026-06-25，RTAB-Map 离线评估链路已经从临时手动流程整理为可复用 pipeline：

```text
HNO-VIO run_context + saved odom
-> run_rtabmap.sh
-> RTAB-Map graph optimized odom
-> evo evaluation
-> branch-safe RPY diagnostics
```

剩余主要工程问题是 RTAB-Map optimized graph 的节点密度与视觉平滑性，需要通过 rerun 验证新参数是否足够。如果密度提升后曲线仍有大折线，应再区分：

- graph 仍稀疏；
- loop/proximity 约束引入局部跳变；
- raw odometry 本身在对应区间误差过大；
- 或需要后处理插值生成 dense visualization，但不能把插值当作优化结果本身。
