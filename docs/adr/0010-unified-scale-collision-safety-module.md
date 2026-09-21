---
status: accepted
date: 2026-09-21
---

# 统一碰撞安全模块采用共享尺度语义与两类查询内核

规划、轨迹执行与 OSCBF 共同使用一个具有碰撞判定权的模块、同一份机器人 ellipsoid
几何和同一份带版本 LiDAR `CollisionScene`。模块对外统一返回线性
`proximity_scale`、`scale_margin`、barrier、关节梯度、危险对象身份和查询状态；
统一安全集合为 `proximity_scale >= scale_margin`。

自碰撞采用 ellipsoid–ellipsoid DCOL，使用
`h = alpha_star - scale_margin`。LiDAR 环境碰撞采用 ellipsoid–point scale，每个进入
活动集合的 pair 使用 `h_ij = point_scale_sq_ij - scale_margin_ij^2`。两类查询共享安全
集合，同时保留各自已有的梯度推导。规划器、轨迹执行器和 OSCBF 必须调用该模块及对应
场景版本，不能各自维护另一套碰撞几何或判定含义。

## Considered Options

曾考虑把 LiDAR 点云先转换成数量受控的 convex primitives，再让所有碰撞关系进入同一个
DCOL 优化器。该方法会引入聚类身份变化、凸外包占用可通行区域，以及
ellipsoid–polytope 连续可微证明范围不足的问题，因此未采用。

## Consequences

当前 OBB 米制距离、自碰撞 OBB 保留决定、动态包围球和可选环境距离场均不能继续作为
统一碰撞判定来源；程序迁移及验证由后续设计决定推进。点云时间语义仍需单独决定。
机器人几何容量与表示见
[ADR 0011](0011-fixed-capacity-compound-ellipsoid-geometry.md)，自碰撞候选关系见
[ADR 0012](0012-complete-self-collision-pairs.md)，LiDAR 环境表示见
[ADR 0014](0014-local-lidar-occupancy-scene.md)，米制范围转换见
[ADR 0017](0017-support-point-metric-radius.md)，环境约束组织见
[ADR 0019](0019-active-independent-environment-cbf-rows.md)，连续区间验证见
[ADR 0020](0020-adaptive-collision-segment-certification.md)，碰撞约束准入见
[ADR 0022](0022-non-relaxable-command-collision-rows.md)，进程与模块边界见
[ADR 0023](0023-shared-local-jax-collision-module.md)，规划方法见
[ADR 0024](0024-aeb-rrtstar-with-jax-trajectory-optimization.md)，自碰撞 DCOL solver 见
[ADR 0025](0025-exact-jax-ellipsoid-dcol.md)，自碰撞米制间距转换见
[ADR 0026](0026-pair-specific-self-clearance-mm.md)，外部距离查询见
[ADR 0027](0027-independent-environment-distance-mm.md)，LiDAR 自体过滤见
[ADR 0028](0028-mesh-ray-lidar-self-filter.md)，等待与故障恢复见
[ADR 0031](0031-transient-wait-and-latched-collision-fault.md)。
