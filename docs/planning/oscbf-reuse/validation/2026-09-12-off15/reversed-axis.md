# 路径跟踪评价报告

- 任务质量：不通过；在线门控：通过
- 全部所需判据：不通过（仅限下述证据范围，不授予实机准入）
- 证据：analytic；测量边界：kernel_candidate
- 运行：reversed-axis；工况：reversed-axis
- 测量定义：analytic positions and same-time reference；时间基准：explicit test seconds
- 模型 / 配置 / 路径 / 数据身份：analytic point on x-axis; tool X direction / OFF-15 inherited default thresholds / line-x:0..1m / sha256:ccba1c4f28d70ffea4eea5e0ca3ef83d37c770e6c4fe403475fec19dcd3acbdb#reversed-axis
- 总样本：11；终止原因：completed
- 样本记录 SHA-256：03c349862fbd923519308ce8c2563e9966704988c444d10798a4989116d4be29
- 声明范围：EvaluationScope(total_length_m=1.0, start_m=0.0, end_m=1.0)；初始/最终投影弧长：0 / 1 m
- 绝对弧长比：1；本次区间覆盖比：1
- 本次区间完成：True；整条路径覆盖：True；全部所需判据下整条路径验证：False
- 采样耗时：10 s；计算 deadline：20 ms；超期率：0
- 未采集边界：filtered_command, simulated_state, real_feedback

## 误差及诊断统计

统计按样本等权；部分有效样本仍可供诊断，缺失/无效样本阻止对应指标通过。

| 指标（单位见定义） | 有效/缺失/无效 | 均值 | RMS | p95 | 最小 | 最大 |
|---|---|---|---|---|---|---|
| actual_tangent_speed_m_s | 0/11/0 | 未测 | 未测 | 未测 | 未测 | 未测 |
| cross_track_m | 11/0/0 | 0 | 0 | 0 | 0 | 0 |
| delta_slack | 0/11/0 | 未测 | 未测 | 未测 | 未测 | 未测 |
| feedrate_m_s | 11/0/0 | 0.01 | 0.01 | 0.01 | 0.01 | 0.01 |
| legacy_orientation_error | 0/11/0 | 未测 | 未测 | 未测 | 未测 | 未测 |
| obstacle_clearance_m | 0/11/0 | 未测 | 未测 | 未测 | 未测 | 未测 |
| obstacle_margin_m | 0/11/0 | 未测 | 未测 | 未测 | 未测 | 未测 |
| online_cross_track_m | 0/11/0 | 未测 | 未测 | 未测 | 未测 | 未测 |
| pos_error_m | 11/0/0 | 0 | 0 | 0 | 0 | 0 |
| qp_primal_residual | 0/11/0 | 未测 | 未测 | 未测 | 未测 | 未测 |
| source_time_s | 11/0/0 | 1005 | 1005 | 1009.5 | 1000 | 1010 |
| step_latency_ms | 11/0/0 | 1 | 1 | 1 | 1 | 1 |
| tool_axis_error_rad | 11/0/0 | 3.14159 | 3.14159 | 3.14159 | 3.14159 | 3.14159 |

## 逐项判据与来源

| 指标 | 类别 | 阈值/单位 | 状态 | 测量值 | 结论 | 来源/原因 |
|---|---|---|---|---|---|---|
| cross_track_m.rms | task | <= 0.00015 m | accepted | 0 | 通过 | GitHub #14 resolution 2026-09-04; complete samples compared with sourced threshold |
| cross_track_m.p95 | task | <= 0.0005 m | accepted | 0 | 通过 | GitHub #14 resolution 2026-09-04; complete samples compared with sourced threshold |
| cross_track_m.maximum | task | <= 0.002 m | accepted | 0 | 通过 | GitHub #14 resolution 2026-09-04; complete samples compared with sourced threshold |
| tool_axis_error_rad.rms | task | <= 0.000872665 rad | accepted | 3.14159 | 不通过 | GitHub #14 resolution 2026-09-04; OFF-15 user decision: true axis angle 2026-09-12; complete samples compared with sourced threshold |
| tool_axis_error_rad.maximum | task | <= 0.00872665 rad | accepted | 3.14159 | 不通过 | GitHub #14 resolution 2026-09-04; OFF-15 user decision: true axis angle 2026-09-12; complete samples compared with sourced threshold |
| qp_success_rate | task | >= 0.999 1 | accepted | 1 | 通过 | GitHub #14 resolution 2026-09-04; complete samples compared with sourced threshold |
| obstacle_clearance_nonnegative | safety | >= 0 m | accepted | 未测 | 证据不足 | GitHub #14 resolution 2026-09-04; nonnegative clearance alone does not prove non-overlap; required samples missing or invalid |
| obstacle_clearance_m.minimum | safety | >= 0.03 m | provisional | 未测 | 证据不足 | GitHub #14 resolution 2026-09-04; geometry/scope pending #8/#18; threshold not accepted for this scope |
| deadline_miss_rate | timing | <= 0.01 1 | provisional | 0 | 证据不足 | GitHub #14 resolution 2026-09-04; deadline must match current measured boundary/config; threshold not accepted for this scope |

## 在线门控与约束

- QP：success=1，失败=0，缺失/无效=0
- 准入：{'accepted': 11}；重叠：{'clear': 11}
- 状态事件：{}；限制因素：{}
- 几何裕度、原始约束残差和松弛分别报告；不同量纲不相加。

- legacy_orientation_error 是旧控制误差诊断；delta_slack/qp_primal_residual 是跨类求解诊断，不能当作米解释，物理量请使用逐类数据。

```json
{}
```
