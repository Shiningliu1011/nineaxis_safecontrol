---
status: accepted
date: 2026-09-21
---

# 碰撞能力采用各进程本地调用的共享 JAX 模块

碰撞安全模块位于 `portable_oscbf`，不依赖 ROS。它统一提供机器人几何、版本化场景、
self scale query、environment scale query、自适应区间证明和 facade。规划器、轨迹执行器
与 OSCBF 在各自运行进程内加载同一模块并执行本地 JAX 查询；ROS 层只负责传输
`CollisionScene`、机器人状态和诊断。

每项查询结果携带 `geometry_hash`、`kernel_version` 与 `scene_revision`。三个身份分别
确认机器人几何、查询算法与环境快照，任何不一致都不能取得规划或命令准入。

## Considered Options

曾考虑建立中央 collision ROS 服务。单一运行实例可以集中状态，但批量规划查询会增加
序列化与通信开销，JAX 梯度也无法跨 ROS 调用直接传播，因此采用共享本地模块。

## Consequences

多个进程可以拥有各自的 JIT 编译结果，但必须加载相同 kernel identity 与 geometry
identity。ROS adapter 不能复制碰撞公式或自行解释安全阈值；所有碰撞判定经共享 facade
进入模块。外部 seam 与 Interface 见
[ADR 0034](0034-deep-collision-safety-module-interface.md)，执行频率与线程职责见
[ADR 0035](0035-two-rate-prepared-scene-execution.md)。
