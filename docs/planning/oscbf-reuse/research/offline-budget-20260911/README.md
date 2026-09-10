# 控制预算离线算式与故障回放

2026-09-11；对应 #17 的可独立推进部分。只新增此目录的研究资产；不连接设备、不导入 ROS/CAN、不修改产品配置、现有交接或安全决议。所有示例数字均为纯合成，**不是实测、推荐阈值或生产参数**。

## 本轮完成

`budget.py` 仅用 Python 标准库，检查逐贡献源输入并计算：

```
A_i = consume - acquisition_start_i + clock_error_i
S_i = available_separation_i - minimum_clearance - geometry_error_i - stop_relative_displacement_i
T_allowed_i = S_i / relative_speed_i - A_i       # speed > 0
margin_i = S_i - relative_speed_i * (A_i + T_remaining)
```

双源分别计算，取最严格时间限制；不能用最新源时间刷新另一源。时刻须已映射到同一可信时钟域和会话，采集开始/结束有效；未来时间一律拒绝，这是本草案的保守处理，未实现误差范围内的未来时间容忍。单调秒值用于合成事件线，尚非生产 int64 ns 消息接口。

消费后必须依次给出 remaining_control、final_command、transport、actuator_reaction 四个相邻区间，无重叠、无遗漏间隙。区间端点是用于预算表示的上界累计事件线，不是把一次执行采样自动认作上界。remaining_control 应包含消费后的几何、求解、必要拷贝/等待；final_command 应包含最终校验、调度、滤波相关处理；transport 覆盖 ROS/CAN；actuator_reaction 到制动起始为止。真实低通动态不能仅折算为 tau 的固定时延，需由停止/执行模型验证。消费前处理已在年龄中，制动开始后的机器人位移和障碍接近已在 stop_relative_displacement 中，均不得再次相加。

所有界必须附 basis 和 evidence；拒绝 p95、p99、sample_max 等基准。`documented_bound` 只表示调用者声称提供了文档界，程序**不验证证据真实性或适用工况**；`synthetic_assumption` 明确用于算式演示。无论算式是否可行，输出 `robot_admission: false`。本工具不推导生产控制率、不证明系统安全、不替代看门狗和执行保护。可用时间还需分配给各阶段，不可直接倒数当控制频率。

未知、缺失、负数、非有限、布尔值不会被当作有效数值。已知速度为零单独保留空间检查，时间结果为 null 表示该空间算式没有有限时间限制，**不表示允许无限期复用观测**。非正余量按不可行报告，不钳为零；本草案要求严格正余量。

## 复现与结果

在仓库根目录，Python 3.10.12，无新增依赖：

```bash
python3 docs/planning/oscbf-reuse/research/offline-budget-20260911/check.py
python3 docs/planning/oscbf-reuse/research/offline-budget-20260911/budget.py docs/planning/oscbf-reuse/research/offline-budget-20260911/synthetic-example.json
python3 docs/planning/oscbf-reuse/research/offline-budget-20260911/budget.py docs/planning/oscbf-reuse/research/offline-budget-20260911/synthetic-infeasible.json
python3 docs/planning/oscbf-reuse/research/offline-budget-20260911/budget.py docs/planning/oscbf-reuse/research/offline-budget-20260911/synthetic-invalid.json
```

本轮实际运行：检查器 28 个案例通过，exit 0；三个 CLI 分别 exit 0（仅算式可行）、1（空间耗尽）、2（时钟不可信）。结构化结果见 `results.json`。检查器会在此目录重写三个合成输入及 results.json。

| 合成源 | 保守年龄 | 允许消费到制动时延 | 采用 20 ms 后空间余量 |
|---|---:|---:|---:|
| camera | 52 ms | 308 ms | 144 mm |
| lidar | 102 ms | 258 ms | 119 mm |

独立手算校验：`(.4-.1-.02-.1)/.5-.102 = .258 s`。这里的每个数字均为合成，不能填写到现有 YAML。

故障覆盖：五种界缺失，NaN/Infinity/负值/布尔值，未来/零/反序采集时间，时钟不可信/域不符，会话不可信/重启，必要源缺失/重复，消费后区间重叠/间隙/缺段，预测几何重复补偿风险，分位数和样本最大值冒充上界，空间耗尽，零速度的空间条件。它验证此独立草案的检查行为，不证明现有 FusionEngine 或 HardwareBridge 已修复。

## 核对依据与仍缺证据

读取交接 `handoffs/17-control-budget.md`、`7-perception-time-model.md`、`13-socketcan.md`、`8-obb-environment-geometry.md`。当前源码再核对：`portable_oscbf/work/fusion_engine.py:168,180,231` 使用最新源年龄与融合最大时间；`src/robot_safecontrol_moveit/hardware_bridge.py:149,158` 使用 now 并置 feedback_ok；`src/robot_safecontrol_moveit/oscbf_controller.py:459,580,759` 分别为单步计时入表、低通、p95 汇总。当前存在的字段不足以自动填入此预算草案。

| 缺项 | 可继续离线工作 | 需要目标设备/工况证据 |
|---|---|---|
| 逐源 provenance 与时钟界 | 时间字段、故障回放和消息契约由 #7/#21/#23 落地 | 真实驱动采集语义、映射误差、重启/失锁记录 |
| 几何与速度上界 | #8 独立几何/导数对照、附件模型与覆盖审计 | 实际装配/标定误差、工具/载荷、目标速度和最小间距 |
| 消费后阶段界 | #3 长时组合负载与分段记录，#13/#33 离线执行保护实现 | CAN/执行反馈、调度故障界、独立期限保护行为 |
| 制动后相对位移 | 定义机器人/障碍位移的非重叠记录格式 | 目标速度/载荷下制动时间与停止距离，承重轴行为 |
| 最终参数决议 | 汇集证据后形成可审阅的候选表 | 用户确认适用工况及最终周期/后端/期限方案 |

此轮补齐了预算草案的可运行形式与拒绝路径；上述前置与实机证据仍未闭合，#17 不满足关闭条件。
