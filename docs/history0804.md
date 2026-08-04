# HNO-VIO `e—sigma_R—地图反馈`链路排查记录

记录日期：2026-08-04  
主要数据集：EuRoC `V1_02_medium`  
完整实验目录：[`next_correct/`](../next_correct/)

## 1. 排查目标

本轮没有继续调参，而是判断系统发散主要由哪条状态链路触发：

```text
e 的视觉修正
→ sigma_R
→ R 倾角误差
→ 重力投影污染 v/p
→ 错误路标和视觉修正进一步放大
→ 系统失稳
```

观测器中与该问题直接相关的关系为：

\[
\begin{aligned}
\dot{\hat R} &= \hat R(\omega+\hat R^\top\sigma_R)^\times,\\
\dot{\hat p} &= \hat v+\sigma_R^\times\hat p+\hat R K_p\sigma_y,\\
\dot{\hat v} &= \hat g+\hat R a+\sigma_R^\times\hat v+\hat R K_v\sigma_y,\\
\dot{\hat e}_i &= \sigma_R^\times\hat e_i+\hat R K_i\sigma_y.
\end{aligned}
\]

\[
\sigma_R=\frac{k_R}{2}\sum_{i=1}^{3}\rho_i\hat e_i\times e_i
\]

视觉创新为：

\[
\sigma_{yi}=\sum_{s=1}^{2}\pi(R_{cs}y_i^s)
\left(\hat R^\top(\hat p_i-\hat p)-p_{cs}\right).
\]

## 2. 主要实验现象

V1_02 baseline 中，R 的持续增长约从 3.65 s 开始，当时整体旋转误差只有约 1.5°。第一次 p/v 越阈值只是早期瞬态，不能用它判断最终因果顺序。

真正持续的 v/p 爆涨发生在约 46–48 s。此时 R 误差约 16–20°，重力方向倾角约 15°，对应约 2.5 m/s² 的虚假水平重力分量；随后视觉更新持续中断，p/v 快速发散。

三组关键对照的含义如下：

| 对照 | 保留的机制 | 主要用途 |
|---|---|---|
| normal-e | 完整 e、`sigma_R`、视觉和 estimated mapping | 发散基线 |
| fixed-e | 视觉更新仍执行，但提交前把 e 恢复为单位基 | 判断 e 名义状态是否为必要上游因素 |
| normal-e + `sigma_R=0` | e、视觉更新和 estimated mapping 正常运行，只屏蔽 `sigma_R` 的应用 | 判断 `sigma_R` 介导的状态耦合是否触发失稳 |

在 V1_02 上，normal-e + `sigma_R=0` 虽然仍有较大的轨迹误差，但 R 不再像 baseline 一样持续爆涨，p/v 也没有进入同等级的发散。这说明 `sigma_R` 介导的状态耦合是姿态崩溃的主要触发路径。

fixed-e 不是纯视觉消融：它会丢弃视觉更新产生的 e 修正，并同时压低后续 `sigma_R`，因此只能作为 oracle 对照，不能单独用来判断视觉闭环是否是崩溃原因。

## 3. GT mapping 补全的视觉链路

GT mapping 使用相同的图像、特征提取、匹配、视觉残差和 Jacobian，只把生成路标时使用的位姿替换为真值。其 APE RMSE 约为 0.013 m，说明这些视觉模块在真值邻域内具有足够精度，不是本轮发散的主要原始误差源。

estimated mapping 的关键差异是：当前 R/p 的误差会被写入路标 \(\hat p_i\)。错误路标产生的视觉创新不再只是近似零均值的像素噪声，而会形成与当前状态误差相关的持续偏置：

\[
\delta\hat p_i
\rightarrow \sigma_y
\rightarrow \hat R K_i\sigma_y
\rightarrow \delta e_i
\rightarrow \sigma_R.
\]

若 \(\hat e_i=e_i+\delta e_i\)，则在小误差下：

\[
\sigma_R\approx\frac{k_R}{2}
\sum_i\rho_i\,\delta e_i\times e_i.
\]

因此，视觉链路的主要问题不是单次特征提取噪声，而是错误地图把位姿误差转化为有方向性的视觉创新，持续推偏 e，再通过 `sigma_R` 反馈到状态。

本轮最符合数据的完整解释为：

```text
小幅 R/p 误差
→ estimated mapping 生成有偏路标
→ sigma_y 出现系统性偏置
→ 视觉更新持续推偏 e
→ sigma_R 长期小幅累积
→ R 倾角增大
→ 重力投影污染 v/p
→ 路标误差和视觉更新进一步恶化
→ 系统失稳
```

## 4. 当前结论与边界

- GT mapping 的高精度支持“视觉前端和 Jacobian 在真值邻域内足够准确”，但不证明它们在大残差区间内始终正确。
- estimated mapping 产生的有偏路标及视觉创新是重要放大器；`sigma_R` 介导的状态耦合是 V1_02 姿态崩溃的主要触发路径。
- `sigma_R=0` 同时删除 R/p/v/e 方程中的全部 `sigma_R` 项。因此严格结论不是“只证明了 R 方程中的一项”，而是“证明了 `sigma_R` 介导的整条状态耦合路径具有关键作用”。
- V1_03 difficult 中，fixed-e 和 `sigma_R=0` 仍可能因稳定路标及视觉约束丢失而发散，说明困难场景还存在独立的地图失效路径。
- 系统失稳后无法从数据观察 R 的渐近行为，因此不声称 R 最终一定收敛。

## 5. `next_correct/` 目录说明

`next_correct/` 是 V1_02 e 瞬态实验的完整交付目录：

```text
next_correct/
├── README.md          # 实验运行和复现方法
├── REPORT.md          # 正式实验的主报告
├── configs/           # 四组消融的固定参数
├── scripts/           # 运行矩阵和离线分析脚本
├── reproducibility/   # 运行环境、源码及二进制记录说明
├── data/              # 原始日志和完整轨迹，默认被 Git 忽略
├── summary/           # 逐运行指标、事件、滞后和消融 CSV
└── figures/           # 连续曲线和消融对比图
```

该目录只承载 V1_02 正式实验，不应把旧 parity、收敛域、调参实验或后续独立运行的结论混入主报告。

## 6. 结果查看顺序

建议按以下顺序查看，避免只看首次越阈值事件而误判因果顺序：

1. 本文：先理解最终链路和结论边界。
2. [`next_correct/REPORT.md`](../next_correct/REPORT.md)：查看正式指标、对照结果和数据完整性。
3. [`figures/early_growth.png`](../next_correct/figures/early_growth.png)：最关键，联合查看 e、`sigma_R`、R/倾角、p/v 增长率、重力投影和视觉更新中断。
4. [`figures/state_feedback.png`](../next_correct/figures/state_feedback.png)：比较 baseline、fixed-e 和 `sigma_R=0` 的 R/p/v 全程变化。
5. [`figures/e_transient.png`](../next_correct/figures/e_transient.png)：查看 e 的收敛、非正交误差和 `sigma_R` 演化。
6. [`figures/causal_ablation.png`](../next_correct/figures/causal_ablation.png)：查看两种干预的总体效果。
7. `summary/growth_rate_diagnostics.csv`、`run_summary.csv` 和 `causal_ablation.csv`：核对图中的关键时间和数值。
8. `summary/lag_analysis.csv` 和 `event_order.csv`：只作为去趋势关联和阈值事件的辅助证据。
9. `data/` 及每次运行的 manifest：需要复查原始轨迹、日志、参数和校验和时再查看。
10. [`README.md`](../next_correct/README.md) 和 [`reproducibility/README.md`](../next_correct/reproducibility/README.md)：需要重新运行或复现实验时查看。

