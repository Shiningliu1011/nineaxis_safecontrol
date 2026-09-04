# 07B — 选定冗余目标与优先级（grilling）

**What to build:** HITL 决策票（只能由用户拍板，AFK agent 无法回答），由 07A 的
对比表 + 06B 的跟踪语义决策驱动，回答：

- **冗余目标 4 选 1**：① QP 内零空间自运动（关节限位裕度最大化 / 回避奇异 β /
  stay-in-posture）；② 提高工具轴跟踪品质（角速度最小化 / 任务空间权重调制）；
  ③ 增加任务维数（6D 位姿或第 9 关节独立任务）；④ 保持现状（QP 自然分配 + w_joint=0.1）；
- **是否启用 `use_nullspace_policy`？**任务与自运动的权重/优先级如何设；
- **避障 vs 任务优先级：**现在 CBF 是硬约束=最高；零空间自运动是否允许在 CBF
  限制内优选构型（当前未实现，是否纳入目标）；
- **评价指标取舍：**最大工具轴角速度 / 限位裕度 / CBF 主动率，哪个作为验收维度。

**Blocked by:** 06B（冗余目标依赖任务定义：语义/5D-6D）+ 07A（策略可行性数据）。

**Type:** grilling（HITL）

**Queue:** wayfinder-core — 决策票：冗余目标与优先级（MAP 讨论项 F），排在 06B 与 07A 之后。
**Tracker:** #16 (https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/16)

**Status:** ready-for-human

- [ ] 选定冗余策略（4 选 1 + 指标取舍）
- [ ] 确定零空间启用/权重/优先级（改默认值则给出新配置值）
- [ ] 明确后续 impl 票面（如选中 ②/③）
