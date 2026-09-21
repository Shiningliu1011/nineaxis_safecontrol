---
status: accepted
date: 2026-09-21
---

# 场景更新采用前视复查与风险触发重新规划

新 `scene_revision` 到达后，轨迹执行器立即使用当前 revision 重新证明执行前视区间，完成
以前输出保持命令。前视区间通过后可以继续执行，同时后台验证完整剩余名义轨迹。剩余
轨迹出现未通过区间、覆盖不足、动态预测冲突，或者最小 `remaining_margin_mm` 低于
`replan_margin_mm` 时启动 AEB-RRT* 与 JAX trajectory optimization。

只影响远处且剩余轨迹全部通过的新 revision 更新 `admitted_revision`，不重新生成轨迹。
重新规划期间，只有当前 revision 已经证明安全的前视区间可以继续执行。

## Considered Options

曾考虑每次 revision 更新都重新规划完整剩余任务。LiDAR 持续更新会让无关环境变化重复
触发全局搜索与轨迹优化，因此采用前视复查、后台验证和风险触发。

## Consequences

前视证明、剩余轨迹验证与重新规划需要分开报告状态和期限。`replan_margin_mm` 必须大于
执行期最低安全间距，并结合规划耗时、停车距离和障碍速度范围验证后确定。
