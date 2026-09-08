# 13 — 更新 tracking_evaluator 指标定义与阈值（impl）

**What to build:** 根据 06B 定案更新 `tracking_evaluator.py` 的指标定义、阈值和
验收逻辑:

- cross_track 升级为 mandatory primary（已有,需移除 pos_error 的 primary 地位）;
- pos_error(3D) 降级为 diagnostic;
- completion_fraction 改用弧长比替代源时间比;
- 新增 feedrate diagnostic 指标（joint utilization ratio, slowdown_count）;
- 新增 controller timing 指标（step_ms, deadline_miss_rate）;
- 验收阈值表写入 evaluator 配置（见 06B acceptance table）;
- tracking_score 权重重分配（cross_track 权重提升,pos_error 降级）。

**Blocked by:** 06B（验收语义定案）。

**Type:** impl

**Queue:** wayfinder-core — evaluator 是验收的执行层,06B 定案后立即可做。
**Tracker:** TBD

**Status:** ready-for-agent

- [ ] cross_track 升级为 mandatory primary,移除 pos_error primary
- [ ] completion_fraction 改用弧长比
- [ ] 新增 feedrate diagnostic（joint utilization, slowdown_count）
- [ ] 新增 controller timing（step_ms, deadline_miss_rate）
- [ ] 验收阈值写入配置（cross_track/orient/completion/QP/clearance/deadline）
- [ ] tracking_score 权重重分配
- [ ] 更新单元测试覆盖新定义
