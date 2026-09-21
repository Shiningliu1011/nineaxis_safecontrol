---
status: accepted
date: 2026-09-21
---

# 碰撞计算统一使用 float64

LiDAR 驱动与 ROS 消息允许提供 `float32` 原始坐标。`CollisionSafety.prepare_scene` 在构造
`PreparedScene` 时完成一次 `float64` 转换，并同时检查有限值、坐标单位、范围和 shape。
`PreparedScene` 中参与碰撞计算的数组统一存储为 `float64`。

机器人状态、轨迹状态、组合 ellipsoid 几何、DCOL、point-scale、barrier、梯度、运动上界、
区间证明、毫米距离查询以及命令 QP 的数值输入全部使用 `float64`。碰撞安全模块启用并验证
JAX x64，内部禁止将数据转换回 `float32`，也禁止在同一条碰撞计算链中混合两种精度。
精度策略属于 `kernel_version` 的身份内容。

公开的 `clearance_mm`、`distance_mm` 和 `remaining_margin_mm` 保持毫米语义；碰撞安全模块在
Interface 边界完成单位转换，内部几何计算统一使用米。诊断输出携带原始 `float64` 结果，
显示层可以单独控制小数位数。

## Decision Basis

毫米级安全阈值、DCOL 接触边界、barrier 梯度和区间下界都需要一致的数值精度。统一
`float64` 可以消除模块内部因隐式类型提升或降级产生的边界差异，并使规划、执行和 OSCBF
对同一状态给出相同结果。

## Consequences

运行时间验收必须覆盖 `prepare_scene`、批量碰撞查询、区间证明和 100 Hz 命令路径。
控制查询与 QP 超过 deadline 时按 ADR 0035 进入碰撞故障锁存；运行期间不能通过降低精度
继续发送命令。
