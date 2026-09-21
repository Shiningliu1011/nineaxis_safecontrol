---
status: accepted
date: 2026-09-21
---

# 自碰撞间距使用毫米配置与 pair-specific scale margin

每个 self collision ellipsoid pair 使用明确的 `clearance_mm_ij`。collision facade 先转换
`clearance_m_ij = clearance_mm_ij * 1e-3`，再根据两个 ellipsoid 的最短半轴生成
`scale_margin_ij = 1 + clearance_m_ij / (a_min_i + a_min_j)`，并使用
`h_ij = alpha_star_ij - scale_margin_ij`。

公开配置、查询、诊断和报告中的碰撞间距统一使用带 `_mm` 后缀的毫米值。机器人运动学、
ellipsoid 参数和 JAX kernel 内部继续使用米，单位转换只允许出现在 collision facade
边界。

## Considered Options

所有 pair 使用同一个全局 scale margin 会让不同尺寸 ellipsoid 获得不同的米制保护范围。
pair-specific 转换保持毫米间距含义一致，因此采用该方法。

## Consequences

配置加载必须拒绝无单位字段、负数和非有限值。几何最短半轴或 clearance 改变时需要重新
生成 scale margin 与 `geometry_hash`。外部障碍距离查询同样使用毫米输出，职责见
[ADR 0027](0027-independent-environment-distance-mm.md)。
