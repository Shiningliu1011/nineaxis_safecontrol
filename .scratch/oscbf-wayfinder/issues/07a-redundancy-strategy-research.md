# 07A — 冗余自由度策略对比（9-DOF 对 5D 工具轴任务）（research）

**What to build:** 9 自由度臂（J1 棱柱 + J2–J9 旋转）目前只投影到 5D 工具轴任务
（`task_mode: tool_axis_5d`、`use_nullspace_policy: false`），冗余自由度既没有
显式零空间策略也没有被 CBF-QP 利用。本票调研并在仿真中评估候选，**对比其与
本控制器架构（速度级 P-only OSC + CBF 硬约束）的适配性**：

1. QP 内零空间自运动（关节限位裕度最大化 / 回避奇异 β、stay-in-posture）；
2. 提高工具轴跟踪品质（角速度最小化 / 任务空间权重调制）；
3. 增加任务维数（6D 位姿、或第 9 关节独立任务）是否更符合真实需求；
4. 保持现状（让 QP 自然分配 + w_joint=0.1 关节阻尼项）。

产出：候选实现的接口改动面、各自的评价指标（最大工具轴角速度、限位裕度、
CBF 主动率）、推荐结果。「是否采用及优先级」由 **07B（grilling 决策票）** 拍板。

**Blocked by:** None — 候选策略与架构对比可独立进行；目标优先级在 07B（依赖 06B）定。

**Type:** research（AFK）

**Queue:** wayfinder-core — 策略对比 AFK 可做；取舍决策在 07B。
**Tracker:** #5 (https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/5)

**Status:** ready-for-agent

- [ ] 梳理 9-DOF 相对 5D 任务的冗余度（数值：零空间维度）
- [ ] 实现或调用现有 nullspace 路径，量化 2–3 种候选
- [ ] 对比表：指标 × 策略（附仿真数据与代码引用）
- [ ] 推荐候选 + 需用户决策的选项清单
