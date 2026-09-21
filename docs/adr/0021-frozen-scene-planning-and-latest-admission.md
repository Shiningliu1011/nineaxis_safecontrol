---
status: accepted
date: 2026-09-21
---

# 规划使用冻结场景并在最新场景上取得执行准入

规划请求固定使用一个不可变 `CollisionScene`，规划结果携带 `scene_revision` 与
`geometry_hash`。规划完成时若最新 revision 已改变，碰撞安全模块使用最新场景复查整条
路径；通过后记录 `admitted_revision`，未通过时在最新场景上重新规划。轨迹执行只能发送
已经通过当前 revision 前视验证的区间。

新 revision 到达后，旧的未来区间证明失效；完成新前视验证以前输出保持命令。OSCBF 每个
控制周期使用当前已经通过有效性检查的场景。

## Considered Options

曾考虑每次 revision 改变都取消规划。LiDAR 持续更新时，该规则可能让规划长期无法完成。
冻结场景使一次规划具有稳定输入，最新场景完整复查确保旧场景结果不能直接进入执行。

## Consequences

规划结果、区间证明和执行状态都必须携带 revision 与 geometry identity。前视验证需要具有
明确计算期限；期限内无法取得当前 revision 的证明时保持命令，不能沿用旧证明。执行期
重新规划触发见 [ADR 0030](0030-risk-triggered-remaining-task-replanning.md)。
