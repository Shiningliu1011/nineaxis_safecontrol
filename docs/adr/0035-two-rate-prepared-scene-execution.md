---
status: accepted
date: 2026-09-21
---

# 感知准备场景与控制查询采用双频执行

感知工作线程按 LiDAR 更新构造并验证新的不可变 `PreparedScene`，完成后通过原子引用一次
发布。100 Hz 控制线程只读取一个完整 PreparedScene，执行
`query(..., OSCBF_MODE)` 与 QP；规划、完整剩余轨迹验证和重新规划使用独立工作线程。

新 revision 正在准备时进入 `REVISION_PENDING` 并输出保持命令。场景准备失败时进入对应
故障锁存。控制线程不能等待锁、文件、ROS 调用、场景构造或动态容量扩展，并分别记录
scene prepare、collision query 与 QP 的运行时间和 deadline 状态。

## Considered Options

曾考虑让控制线程每个周期同时处理 LiDAR 场景、构造 PreparedScene 和求解 QP。射线更新、
support 合并、track 关联和活动集合更新会占用 10 ms 控制期限，因此采用双频执行。

## Consequences

PreparedScene 必须完全不可变，发布操作不能让控制线程看到部分更新。控制查询和 QP 的总
运行时间超过 deadline 时进入碰撞故障锁存，不能跳过本周期碰撞检查后继续发送命令。
