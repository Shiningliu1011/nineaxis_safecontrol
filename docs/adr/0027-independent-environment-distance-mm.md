---
status: accepted
date: 2026-09-21
---

# 保留独立毫米距离查询并维持 environment point-scale barrier

碰撞安全模块提供 `query_environment_distance_mm()`，计算机器人组合 ellipsoid 与 occupied
support sphere 保守包络之间的最小有符号欧氏距离。support sphere 范围包含 voxel、测量、
标定和运动误差，不包含期望安全间距。查询返回 `distance_mm`、
`required_clearance_mm`、`remaining_margin_mm`、最近 robot link、ellipsoid slot、support
identity、双方最近点、`scene_revision`、采集时间与 query status。

`distance_mm` 大于零表示分离，等于零表示接触，小于零表示保守包络相交。AEB-RRT*、
trajectory optimization、日志和可视化可以把距离用于代价或诊断。OSCBF 的环境安全约束
继续使用 ellipsoid–point scale barrier，距离查询不能授权安全状态。

## Consequences

support 的观测误差范围与 `required_clearance_mm` 必须分开保存，避免期望间距被计入距离
两次。距离结果必须携带 witness 与 scene identity，缺少有效场景时不能返回看似正常的
数值。
