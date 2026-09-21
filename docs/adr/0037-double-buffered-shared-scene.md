---
status: accepted
date: 2026-09-21
---

# PreparedScene 通过双缓冲共享内存发布

场景准备工作线程持有两个固定布局的 host shared-memory slot。它只写当前未启用的 slot，依次
完成 `float64` 转换、容量检查、覆盖检查、identity 检查、checksum 计算和场景健康判定。
全部成功后，以一次原子操作发布新的 `scene_revision` 与 active slot index。已启用 slot 在被
替换前保持不可变。

共享布局包含固定容量的 occupied support、`active_mask`、`rho_mm`、可选运动状态、不确定性
范围、时间信息、覆盖状态、固定环境模型 revision、`geometry_hash`、`kernel_version`、shape
版本和 checksum。布局版本发生变化时必须更换身份，读取进程不能推断缺失字段。

ROS 只发布 `scene_revision`、采集时间、准备时间、active slot index、布局身份和健康状态。
控制器、规划器与轨迹验证器在发现新 revision 后检查 header、identity、checksum 和时间，随后
把该 revision 复制一次到本进程的 JAX device buffer，形成各自不可变的 `PreparedScene` 引用。

读取进程在复制期间检测到 revision 变化、checksum 不一致、revision 跳跃、slot 超时或布局
身份不匹配时，拒绝该副本。新的有效 revision 尚未完成本地 device 准备时进入
`REVISION_PENDING`；共享内存完整性、身份或时间依据失效时进入碰撞故障锁存。

## Decision Basis

固定容量场景包含大量数组，并被多个进程按相同 revision 使用。双缓冲共享内存减少 ROS 大
数组的重复序列化与复制，同时让发布者在原子切换前完成全部场景验证。每个读取进程保留本地
JAX device buffer，因此 100 Hz 查询期间不访问共享内存，也不进行跨进程请求。

## Consequences

共享布局、所有权、slot 生命周期、进程重启、旧 revision 回收和 checksum 算法必须有明确
测试。记录与回放工具需要保存完整 slot 内容及其 header，单独保存 ROS revision 消息不能重建
碰撞场景。
