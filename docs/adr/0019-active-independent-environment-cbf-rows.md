---
status: accepted
date: 2026-09-21
---

# 环境碰撞采用固定容量活动集合与独立 CBF 行

JAX 批量计算全部有效 ellipsoid–point barrier。每个 robot ellipsoid 选择最多 `K` 个
最危险 support pairs 进入环境活动约束集合，每个选中 pair 保留独立 barrier、关节梯度、
时间项和对象身份。只有能够证明在下一次检查前不会进入激活范围的 pair 才能省略；需要
进入集合的 pair 超过 `K` 时，状态变为 `CONSTRAINT_OVERFLOW` 并保持命令。

活动集合使用不同的进入与退出阈值，并保留 witness identity，减少边界附近的 pair 频繁
切换。固定槽位与 `active_mask` 使 QP 行数保持为
`robot_ellipsoid_count * K`。

## Considered Options

曾考虑让每个 robot ellipsoid 对全部 support points 使用一个 `softmin` 聚合行。多个
相反接触方向可能让聚合梯度接近零，Q4 原型已经验证该类抵消会在单个控制周期内放过危险
命令，因此环境 point pair 同样保留独立行。

## Consequences

`K`、进入阈值、退出阈值和省略 pair 的运动下界必须由 JAX/QP 运行时间、控制周期与近场
occupied 密度共同验证。诊断可以汇总每个 robot ellipsoid 的最小 scale，但汇总值不能
替代独立约束。
