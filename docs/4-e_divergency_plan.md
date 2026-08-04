# e 瞬态—sigma_R—状态发散因果实验最终方案

  ## 摘要

  实验固定使用 V1_02、estimated-live mapping、bag_rate=1.0、关闭 ZUPT；GT mapping 不再作为实验变量或重复验证。比较四组：

   组别             e               正交化    sigma_R                                      运行
  ━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━  ━━━━━━━━  ━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
   baseline         normal          开启      正常                                     3 次完整
  ───────────────  ──────────────  ────────  ────────────  ─────────────────────────────────────
   fixed-e          固定为单位基    开启      正常计算                                 3 次完整
  ───────────────  ──────────────  ────────  ────────────  ─────────────────────────────────────
   sigma-zero       normal          开启      强制应用 0                               3 次完整
  ───────────────  ──────────────  ────────  ────────────  ─────────────────────────────────────
   no-projection    normal          关闭      正常          3 次，每次恰好 900 committed frames

  目标是判断 e 的非正交或非收敛是否在有限时间内先于并触发/放大 R、p、v 误差，以及主要路径是否为 e → sigma_R → R。

  ## 实现变更

  - 新增启动参数：
      - experiment_fix_e_hat:=false
      - experiment_force_sigma_R_zero:=false
      - experiment_max_frames:=0，0 表示不限帧。
      - 正交化继续使用现有 update_enforce_structure。

  - fixed-e 在初始化后、每次计算 sigma_R 前，以及视觉/ZUPT 更新后提交状态前，将三个 e_hat 固定为单位基；不冻结协方差、Kalman 增益或交叉协方差。
  - sigma-zero 组正常演化 E，正常计算 sigma_R_raw，但传给 R 传播的 sigma_R_applied 强制为零；其他传播、视觉更新和协方差计算保持不变。
  - 每次视觉更新按以下顺序采样：更新前状态 → 未投影候选 E_raw → 投影候选 E_projected → 实验干预 → committed state。fixed-e 中仍保存被丢弃的 E_raw 和 raw_visual_dE。
  - 在每个 committed camera state 写出 vio_results/e_diagnostics.csv：
      - 时间戳、帧号、实验开关和更新状态；
      - E_raw、E_projected、E_committed 的矩阵元素；
      - theta_E = angle(Proj_SO(3)(E_raw))，SVD 投影需修正负行列式；
      - epsilon_orth、epsilon_det、实际 projection_correction；
      - sigma_R_raw 与 sigma_R_applied；
      - 估计的 R/p/v；
      - 视觉批次实际状态差 dR/dp/dv/dE，均在投影和 fixed-e clamp 前计算；
      - 状态有限性标记。

  - 当前 Updater 不直接修改 R，因此视觉 dR 预计为零；仍记录实测值，但主要检验间接路径 visual dE → sigma_R → 后续 R。
  - 更新 docs/e_transient_map_feedback_final_plan.md 为本四组方案，删除旧的 7 组地图矩阵、parity 门禁、收敛域及调参内容。

  ## 执行与分析

  - 先用同一最终二进制各运行 100 帧验证：
      - baseline：能同时捕获投影前后 E；
      - fixed-e：committed E=I、传播时两个 sigma_R 均为零，但 raw_visual_dE 可非零；
      - sigma-zero：E 正常变化、sigma_R_raw 可非零、sigma_R_applied=0；

  - 三轮完整实验采用轮换顺序，降低运行顺序偏差：
      1. baseline → fixed-e → sigma-zero
      2. sigma-zero → baseline → fixed-e
      3. fixed-e → sigma-zero → baseline
         随后执行三次 900 帧 no-projection。

  - 全部正式运行锁定同一源码 SHA 和二进制 SHA-256；保存参数、数据集路径、开始/结束时间、退出码及文件校验和。算法崩溃或非有限状态作为有效失稳结果，基础设施失败才重跑并保留失败记录。
  - R/p/v 误差离线计算：
      - 使用首个有效匹配位姿确定一次固定 SE(3) 对齐，禁止全轨迹优化对齐；
      - 优先读取 EuRoC GT 自带速度，并施加同一对齐旋转；
      - 只有速度字段缺失或不可用时才由 GT 位置中心差分，并在结果中记录 gt_velocity_source。

  - 事件均要求持续 5 帧：
      - E 不收敛：theta_E>2° 或 0.5 秒增加超过 1°；
      - E 非正交：epsilon_orth 或 epsilon_det 超过 baseline 前 5 秒汇总值的 median+5MAD；
      - R：误差 >5° 或 0.5 秒增加超过 3°；
      - p/v：分别超过 0.5m、0.5m/s；
      - 视觉修正：dp/dv/dE 分别超过其稳定段 median+5MAD；
      - 失稳：R 持续 >10°、p >5m、v >10m/s 或出现非有限状态。

  - 所有曲线按数据集时间重采样到 20 Hz。滞后分析仅使用首个失稳事件之前的数据：
      - 对 E 负担、R/p/v 误差和 0.5 秒视觉修正总量计算 0.5 秒增量；
      - 在 0–2s 正滞后范围计算 Spearman 关联；
      - 不直接对原始单调上升曲线做相关分析，避免共同趋势造成虚假相关。

  - 每次匹配的 baseline/fixed-e/sigma-zero 三元组使用其共同有效时段比较累计误差；另行报告完整运行是否失稳、峰值、峰值时间、最终 5 秒统计、累计误差及事件领先时间。

  ## 验收与结论规则

  - 链路时序证据：至少 2/3 baseline 中，E 事件先于 R，R 再先于 p/v 或视觉修正放大；E 增量与后续 0–2s 下游误差增量的峰值关联必须位于正滞后。
  - “明显改善”定义：相对 baseline，中位累计 R、p、v、视觉修正四类指标至少三类降低 ≥50%，且没有一类恶化超过 25%。
  - “稳定”定义：至少 2/3 完整运行无失稳事件，并且最后 5 秒 R/p/v 中位误差分别 <5°、<0.5m、<0.5m/s。
  - 结论解释：
      - fixed-e 与 sigma-zero 均稳定或明显改善：主要危险路径为 e → sigma_R → R。
      - fixed-e 稳定，但 sigma-zero 仍异常：e 还通过视觉更新、其他名义状态或协方差交叉耦合产生影响。
      - 两者均无明显改善：e 不是唯一或主要上游机制。
      - sigma-zero 改善但 fixed-e 不改善，或重复间方向矛盾：视为干预交互或运行不一致，不作因果结论，优先核查 clamp 时序和协方差耦合。
      - no-projection 只用于判断非正交误差是否自然积累，不参与最终轨迹精度或干预效果排名。

  - 报告不得声称“实验验证了 R 最终一定收敛”。允许的结论措辞为：

    > 实验支持 e 瞬态在有限时间内触发或放大系统误差，因此仅有 R 渐近收敛保证不足以保证运行安全。

  ## 交付与假设

  - next_correct/REPORT.md 是唯一主入口，包含结论、四组对比、事件时间线、累计指标和滞后分析；支撑 CSV、配置、脚本和图表放在相应子目录。
  - 原始日志与完整轨迹保存在 next_correct/data/ 并由该目录 .gitignore 排除；报告和汇总表不得混入旧 parity、地图消融、收敛域或调参结论。
  - 默认四组均使用 estimated-live mapping；GT 轨迹仅用于计算 R/p/v 参考误差，不启用 use_gt_mapping。
  - fixed-e 与 sigma-zero 正式实验不同时开启；若误配置为同时开启，运行脚本应拒绝启动并报告参数冲突。
