---
status: accepted
date: 2026-09-21
---

# Occupied voxel 使用 support point 与米制包围范围

每个 occupied voxel 生成中心 `support_point` 和米制包围范围 `rho_m`。`rho_m` 覆盖
voxel 半对角线、LiDAR 测量误差、外参标定误差与观测年龄对应的运动范围；期望米制间距
保存为独立的 `required_clearance_m_ij`。对于最短半轴为 `a_min_i` 的机器人 ellipsoid，
环境 pair 使用 `barrier_radius_m_ij = rho_m_j + required_clearance_m_ij` 和
`scale_margin_ij = 1 + barrier_radius_m_ij / a_min_i`，并保持
`h_ij = point_scale_sq_ij - scale_margin_ij^2`。

该转换来自 ellipsoid 归一化范数的上界：米制球内任意位置造成的归一化距离变化不超过
`barrier_radius_m_ij / a_min_i`。因此只要中心 point scale 达到上述 margin，整个米制
包围范围与要求间距都在机器人 ellipsoid 外。

## Considered Options

曾考虑直接求 ellipsoid 与 voxel box 的最小 scale。该方法可以减少空间占用，但需要新的
box-constrained 优化 kernel，并会在最近面、边和角切换时产生分段梯度，因此未采用。

## Consequences

每个 `rho_m` 分量与 `required_clearance_m` 必须具有来源与有效期，且同一范围不能重复计入。
机器人 ellipsoid 几何改变后必须重新计算 pair-specific `scale_margin`。环境 CBF 继续使用解析
ellipsoid–point 梯度，并接受最短半轴上界带来的保守范围。固定容量处理见
[ADR 0018](0018-conservative-support-capacity.md)。

公开配置与诊断中的 voxel 范围、测量误差、标定误差、运动范围和期望间距统一使用带
`_mm` 后缀的毫米值。collision facade 在进入 JAX kernel 前集中转换为 `rho_m` 与
`required_clearance_m`；业务接口不能接收无单位数值。独立毫米距离查询只使用 `rho_m`
计算有符号距离，并单独返回要求间距，见 ADR 0027。
