---
status: accepted
date: 2026-09-21
---

# 安全间距越界后使用人工确认的受限恢复模式

当前状态存在 `h < 0`，同时全部相关 self pair 满足 `proximity_scale > 1`、全部相关 environment
pair 满足 `distance_mm > 0`，并且场景、identity、solver、梯度、容量和 required-space coverage
均有效时，系统可以准备受限恢复计划。原任务立即暂停，恢复计划取得人工确认前保持命令。

恢复计划必须使用同一个 `CollisionSafety` Module，并满足：

- 使用低速专用限制，不能携带原任务的进给目标；
- 每个证明区间都让所有已违反 barrier 严格单调增加；
- 所有未违反碰撞行继续满足各自 CBF 条件；
- 所有碰撞行保持不可松弛；
- 完整恢复轨迹通过当前 PreparedScene 的区间证明；
- 确认信息绑定 `scene_epoch`、`scene_revision`、`geometry_hash`、`kernel_version`、碰撞策略
  身份和恢复轨迹 hash。

scene revision、身份或轨迹发生变化时撤销确认并重新验证。不存在满足全部条件的恢复轨迹时
继续保持故障锁存。全部 barrier 回到配置的恢复阈值并保持规定时间后退出恢复模式，原任务仍需
在当前场景重新规划和准入。

任一相关 pair 出现 `proximity_scale <= 1` 或 `distance_mm <= 0`，以及 unknown、solver 异常、
梯度异常、容量异常或场景身份异常时，禁止通过机器人命令执行该恢复模式。

## Decision Basis

安全间距越界后，普通任务轨迹已经失去准入依据。受限恢复模式把运动目标限定为恢复全部
barrier，并通过人工确认、区间证明、低速限制与 revision 绑定控制运动范围。

## Consequences

运行状态需要增加 `RECOVERY_PENDING_ACK` 与 `RECOVERY_ACTIVE`。恢复规划、确认、撤销、完成
和失败都必须记录完整身份与证明结果。恢复模式不能作为规划失败、unknown 或真实接触的继续
运行路径。
