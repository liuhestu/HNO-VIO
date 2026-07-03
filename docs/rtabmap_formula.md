# RTAB-Map 后端图优化公式说明

## 1. 检查结论

这份文档的核心数学抽象是正确的：可以把 RTAB-Map 后端理解为一个 `SE(3)` pose graph optimization 问题，节点是关键帧/图节点位姿，边是相对位姿约束，优化目标是最小化所有边的加权相对位姿误差。

但旧版文档有三处需要修正：

1. 旧版文件名已经过时。当前 pipeline 使用 `vio_results/odom_raw.txt` 和 `offline_results/odom_optimized.txt` 作为 raw / optimized 主轨迹文件。
2. 当前 optimized 主结果是：

   ```text
   offline_results/odom_optimized.txt
   ```

   来源是：

   ```text
   /rtabmap/mapData.graph.poses
   ```

3. HNO-VIO 与 RTAB-Map graph 边的关系需要更精确地表述为：HNO-VIO 通过 `odom -> base_link` TF 提供外部里程计先验，RTAB-Map 在自己创建的 graph 节点之间形成 odometry / neighbor constraints。graph 节点通常比 raw odometry 采样稀疏，不是每一帧 raw odom 都直接成为优化节点。

下面是修正后的当前版本。

## 2. RTAB-Map 在本实验中的角色

在本实验中，HNO-VIO 提供局部里程计先验，RTAB-Map 作为离线图优化后端。RTAB-Map 接收：

- EuRoC stereo 图像；
- CameraInfo；
- `odom -> base_link` TF；
- `/hno_vio/odom` 调试/记录话题；

然后构建 pose graph，并在检测到 loop closure 或 proximity constraint 后进行全局图优化。

当前主流程为：

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

当前输入和输出文件名：

| 文件 | 来源 | 含义 |
| --- | --- | --- |
| `vio_results/odom_raw.txt` | HNO-VIO saved odometry | raw HNO-VIO TUM trajectory |
| `vio_results/odom_raw.csv` | HNO-VIO saved odometry | replay input CSV |
| `offline_results/odom_optimized.txt` | `/rtabmap/mapData.graph.poses` | RTAB-Map graph-final optimized node trajectory |
| `ground_truth_tum` | `run_context.json` | EuRoC ground truth TUM trajectory |

## 3. Pose Graph 定义

RTAB-Map 后端可以抽象为一个位姿图优化问题。图中的节点表示 RTAB-Map graph 节点位姿：

```math
\mathcal{X} = \{\mathbf{T}_1, \mathbf{T}_2, \dots, \mathbf{T}_N\},
\quad
\mathbf{T}_i \in SE(3)
```

其中：

- $\mathbf{T}_i$ 表示第 $i$ 个 graph 节点在 map frame 下的位姿；
- $SE(3)$ 表示三维刚体变换群；
- 每个节点包含旋转和平移：

```math
\mathbf{T}_i =
\begin{bmatrix}
\mathbf{R}_i & \mathbf{t}_i \\
\mathbf{0}^{T} & 1
\end{bmatrix},
\quad
\mathbf{R}_i \in SO(3),
\quad
\mathbf{t}_i \in \mathbb{R}^3
```

图中的边表示两个节点之间的相对位姿约束：

```math
\mathcal{E} = \{(i,j,\mathbf{Z}_{ij},\mathbf{\Omega}_{ij})\}
```

其中：

- $(i,j)$ 是相连的两个节点；
- $\mathbf{Z}_{ij}$ 是测量得到的相对位姿约束；
- $\mathbf{\Omega}_{ij}$ 是该约束的信息矩阵，可理解为协方差矩阵的逆；
- 边的来源包括 neighbor / odometry link、loop closure link、proximity link 等。

注意：这里是概念模型。RTAB-Map 内部具体边类型、协方差估计和优化器实现会随参数和后端变化。

## 4. 相对位姿约束误差

对于一条连接节点 $i$ 和节点 $j$ 的边，优化变量给出的预测相对位姿为：

```math
\hat{\mathbf{Z}}_{ij} = \mathbf{T}_i^{-1}\mathbf{T}_j
```

RTAB-Map 从 odometry、视觉匹配、回环检测或邻近检测得到的测量约束为：

```math
\mathbf{Z}_{ij}
```

该边的位姿残差可写为：

```math
\mathbf{e}_{ij}
=
\operatorname{Log}_{SE(3)}
\left(
\mathbf{Z}_{ij}^{-1}\mathbf{T}_i^{-1}\mathbf{T}_j
\right)
```

其中：

- $\operatorname{Log}_{SE(3)}(\cdot)$ 将 $SE(3)$ 上的位姿误差映射到李代数向量空间；
- $\mathbf{e}_{ij}$ 通常是 6 维误差向量：

```math
\mathbf{e}_{ij}
=
\begin{bmatrix}
\mathbf{e}_{t,ij} \\
\mathbf{e}_{r,ij}
\end{bmatrix}
\in \mathbb{R}^6
```

其中 $\mathbf{e}_t$ 是平移误差，$\mathbf{e}_r$ 是旋转误差。

这个残差写法是常见左乘误差形式。实际后端可能使用等价但实现细节不同的局部参数化。

## 5. 图优化目标函数

RTAB-Map 的 pose graph optimization 可以抽象为非线性最小二乘：

```math
\mathcal{X}^{*}
=
\arg\min_{\mathcal{X}}
\sum_{(i,j)\in\mathcal{E}}
\mathbf{e}_{ij}^{T}\mathbf{\Omega}_{ij}\mathbf{e}_{ij}
```

也就是：

```math
\mathcal{X}^{*}
=
\arg\min_{\mathbf{T}_1,\dots,\mathbf{T}_N}
\sum_{(i,j)\in\mathcal{E}}
\left\|
\operatorname{Log}_{SE(3)}
\left(
\mathbf{Z}_{ij}^{-1}\mathbf{T}_i^{-1}\mathbf{T}_j
\right)
\right\|_{\mathbf{\Omega}_{ij}}^2
```

加权范数定义为：

```math
\|\mathbf{e}\|_{\mathbf{\Omega}}^2
=
\mathbf{e}^{T}\mathbf{\Omega}\mathbf{e}
```

这个目标函数的含义是：寻找一组全局节点位姿，使得所有边约束的预测相对位姿尽量接近实际测量到的相对位姿。

## 6. 不同类型约束的作用

### 6.1 Neighbor / Odometry Link

HNO-VIO 通过 replay 节点发布：

```text
odom -> base_link
```

RTAB-Map 使用这个外部 odometry 作为节点创建和相邻节点约束的重要先验。对于相邻 graph 节点 $i$ 和 $i+1$，可以近似写为：

```math
\mathbf{Z}_{i,i+1}^{odom}
\approx
\left(\mathbf{T}_{i}^{HNO}\right)^{-1}
\mathbf{T}_{i+1}^{HNO}
```

对应残差：

```math
\mathbf{e}_{i,i+1}^{odom}
=
\operatorname{Log}_{SE(3)}
\left(
\left(\mathbf{Z}_{i,i+1}^{odom}\right)^{-1}
\mathbf{T}_{i}^{-1}\mathbf{T}_{i+1}
\right)
```

更准确地说，$i$ 和 $i+1$ 是 RTAB-Map 创建的 graph 节点，不一定对应 HNO-VIO 的每一个 raw odometry 采样。当前实验中 raw odometry 约为 16 Hz，而 graph poses 可能只有数 Hz 或更低。

### 6.2 Loop Closure Link

当 RTAB-Map 判断当前图像与历史图像属于同一地点，并通过几何验证后，会添加 loop closure 边。对于非相邻节点 $i$ 和 $j$：

```math
\mathbf{Z}_{ij}^{loop}
```

表示回环估计出的相对位姿约束。对应残差：

```math
\mathbf{e}_{ij}^{loop}
=
\operatorname{Log}_{SE(3)}
\left(
\left(\mathbf{Z}_{ij}^{loop}\right)^{-1}
\mathbf{T}_{i}^{-1}\mathbf{T}_{j}
\right)
```

该项约束全局一致性。如果 odometry 长期漂移，loop closure 会把重访区域的 graph 轨迹拉回一致位置。

### 6.3 Proximity Link

除外观回环外，RTAB-Map 也可以根据空间邻近关系添加 proximity constraint。其数学形式和 loop closure 类似：

```math
\mathbf{e}_{ij}^{prox}
=
\operatorname{Log}_{SE(3)}
\left(
\left(\mathbf{Z}_{ij}^{prox}\right)^{-1}
\mathbf{T}_{i}^{-1}\mathbf{T}_{j}
\right)
```

proximity link 可以增强局部地图一致性，也可能形成额外的非相邻图约束。

## 7. 完整目标函数展开

把不同边类型分开写，目标函数可表示为：

```math
\mathcal{X}^{*}
=
\arg\min_{\mathcal{X}}
\left(
\sum_{(i,i+1)\in\mathcal{E}_{odom}}
\left\|\mathbf{e}_{i,i+1}^{odom}\right\|_{\mathbf{\Omega}_{i,i+1}^{odom}}^2
+
\sum_{(i,j)\in\mathcal{E}_{loop}}
\left\|\mathbf{e}_{ij}^{loop}\right\|_{\mathbf{\Omega}_{ij}^{loop}}^2
+
\sum_{(i,j)\in\mathcal{E}_{prox}}
\left\|\mathbf{e}_{ij}^{prox}\right\|_{\mathbf{\Omega}_{ij}^{prox}}^2
\right)
```

其中：

- $\mathcal{E}_{odom}$ 是由外部 odometry / neighbor link 形成的顺序边；
- $\mathcal{E}_{loop}$ 是由 loop closure 生成的非相邻边；
- $\mathcal{E}_{prox}$ 是由 proximity detection 生成的非相邻边；
- $\mathbf{\Omega}$ 控制不同约束的置信度或权重。

## 8. 鲁棒图优化形式

如果存在错误回环，普通最小二乘会受到强烈影响。RTAB-Map 支持 robust graph optimization，例如通过 Vertigo、g2o 或 GTSAM 后端提高对错误 loop closure 的鲁棒性。概念上可写为：

```math
\mathcal{X}^{*}
=
\arg\min_{\mathcal{X}}
\sum_{(i,j)\in\mathcal{E}}
\rho\left(
\mathbf{e}_{ij}^{T}\mathbf{\Omega}_{ij}\mathbf{e}_{ij}
\right)
```

其中 $\rho(\cdot)$ 是鲁棒核函数或等价的鲁棒约束处理方式。其作用是降低错误回环边对整体优化结果的破坏性。

是否真正启用 robust graph optimization 取决于 RTAB-Map 构建选项和运行参数；本文档只给出后端可解释的抽象形式。

## 9. 当前实验中的导出语义

当前 pipeline 的核心导出是：

```text
offline_results/odom_optimized.txt
```

它由：

```text
/rtabmap/mapData.graph.poses
```

导出，而不是由 `map -> odom` TF 导出。

原因：

```text
T_map_base = T_map_odom * T_odom_base
```

这一路在当前实验中主要用于诊断。历史运行中 `map -> odom` 往往接近 identity，不能稳定代表最终 optimized graph poses。真正用于报告和 evo 评估的 optimized result 是 `mapData.graph.poses`。

当前应比较：

```text
vio_results/odom_raw.txt vs ground_truth_tum
offline_results/odom_optimized.txt vs ground_truth_tum
```

不应把 dense `map->odom * odom->base_link` 诊断轨迹称为最终 RTAB-Map optimized trajectory。

## 10. 与当前运行结果的对应关系

以 `run_20260625T211722` 为例，密度调参后当前诊断显示：

```text
raw_pose_count: 2362
graph_final_pose_count: 301
graph_final_duration_sec: 142.600000000
graph_final_mean_hz: 2.103786816
graph_final_median_dt_sec: 0.300000000
graph_final_max_dt_sec: 3.149999872
graph_final_median_step_m: 0.127184414
graph_final_max_step_m: 1.231330061
optimized_source: mapData.graph.poses
```

这说明：

- optimized graph 已经比早期 115 poses / 0.8 Hz 更密；
- 但 graph 仍明显比 raw HNO-VIO odometry 稀疏；
- 如果可视化曲线仍显折线，应先检查 `graph_final_mean_hz` 和 `graph_final_max_step_m`，再决定是否提高 `RTABMAP_DETECTION_RATE` 或降低 `RGBD/LinearUpdate` / `RGBD/AngularUpdate`。

## 11. 报告可用表述

可使用：

```text
We model the RTAB-Map backend as an SE(3) pose graph optimization problem.
Each graph node represents a keyframe pose, and edges encode relative-pose
constraints from odometry/neighbor links, loop closures, and proximity links.
The optimized trajectory used in this report is exported from
/rtabmap/mapData.graph.poses as offline_results/odom_optimized.txt.
```

中文表述：

```text
本文将 RTAB-Map 后端建模为 SE(3) 位姿图优化问题。图中每个节点表示一个 RTAB-Map graph 节点位姿，边表示由外部里程计、回环检测或空间邻近检测形成的相对位姿约束。优化目标是最小化所有边约束的加权相对位姿误差。当前实验中，最终优化轨迹由 /rtabmap/mapData.graph.poses 导出为 offline_results/odom_optimized.txt，并与 HNO-VIO raw trajectory 和 EuRoC ground truth 进行对比。
```

## 12. References

1. RTAB-Map official site: RTAB-Map is described as an RGB-D, stereo and lidar graph-based SLAM approach. When a loop closure hypothesis is accepted, a new constraint is added to the map graph and a graph optimizer minimizes the map error.
2. RTAB-Map robust graph optimization documentation: RTAB-Map supports robust graph optimization and can be built with g2o or GTSAM support.
