---
status: accepted
date: 2026-09-21
---

# 动态 support 结合 barrier 时间导数与可达范围

occupancy 继续决定外部障碍物是否存在。tracking 只为已存在的 occupied support 提供名义速度、
加速度范围、关联时间和估计误差，不能创建、删除或清除 occupancy。

100 Hz OSCBF 对当前时刻的 support 计算 point-scale barrier：

`h(q, t) = s_squared(q, p(t)) - beta(t)^2`

其中 `p(t)` 使用 tracking 给出的名义运动，`beta(t)` 包含 support 范围以及传感器、tracking、
加速度、感知延迟和控制延迟产生的不确定性膨胀。`query(..., OSCBF_MODE)` 返回关节梯度和
`partial_h_partial_t`；后者同时包含 support 名义速度项与不确定性范围增长产生的
`beta_dot` 项。命令 QP 对每条活动约束使用完整时间变化 CBF 条件。

规划与 `certify` 根据轨迹时间构造 time-indexed reachable support tube。每个规划边和轨迹
区间必须对该时间段内的机器人运动范围与障碍物可达范围共同取得证明。tracking 失效或关联
中断后，occupied support 仍然存在，并改用已验证的未跟踪速度与加速度上界。所需运动上界
缺失、过期或无法覆盖验证区间时，对应空间按 unknown 处理。

## Decision Basis

barrier 时间导数能让控制器直接响应障碍物接近速度；可达范围为未来规划区间覆盖速度变化、
加速度和预测误差。名义运动进入时间导数，不确定性进入半径范围，两部分分别记录来源，防止
同一误差被重复计入。

## Consequences

共享场景必须携带 tracking 身份、名义速度、加速度范围、估计误差、更新时间和适用时限。
距离查询返回当前查询时间下包含不确定性范围的毫米结果，并附带 track 状态。测试需要覆盖
接近、远离、横向运动、关联中断和可达范围扩张。
