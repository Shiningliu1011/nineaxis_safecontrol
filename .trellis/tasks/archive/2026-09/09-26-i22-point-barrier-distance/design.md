# 技术设计

## 当前入口与修改边界

`collision_safety.py` 已有 self DCOL、identity 与固定结果。环境和距离字段尚未计算。行为位于 Module Interface 和内部 JAX 查询中。

- `_collision_geometry.py` 保存有效 slot identity，提供 self 与 environment 共用的九轴 world geometry 变换。
- `_ellipsoid_dcol.py` 复用该变换，保留现有 self 数值行为。
- `_ellipsoid_point.py` 实现固定批次 point-scale、九轴梯度及独立最近点求解。
- `collision_safety.py` 负责毫米转换、scene track 状态验证、两种 payload、row identity、容量和健康检查。
- `test_ellipsoid_point.py` 通过公开操作进行解析与独立高精度验证；现有接口输入增加明确 track 状态。
- `docs/modules/portable-oscbf/collision-safety.md` 说明字段、算法、容量与当前能力。

## Barrier

采用 `u = R.T @ (support - center)` 与 `sum((u / radii)^2)`。`jax.jacfwd` 经现有 POE 运动学计算九轴梯度；support 的名义时间导数为 `2 * (u / radii^2) dot (R.T @ velocity)`。返回的 `proximity_scale` 保持线性 scale，barrier 使用平方 scale。

完整计算所有有效 ellipsoid–support pair。本票没有省略 pair 的安全证明，因此全部保留；超过每 ellipsoid 的 K 或总行容量时返回 `CONSTRAINT_OVERFLOW`。self 与 environment 行用明确的 primitive kind 区分。动态误差与后续活动集合规则不在本票中新增。

## 独立毫米距离

在 ellipsoid 局部坐标求距离最小的边界点。一般情况用单调 secular equation 的有界二分；内部点采用平移后的 Lagrange multiplier，显式处理最短半轴分量为零、中心及重复最短半轴。通过根区间两端 witness 的欧氏差界限与表面修正量检查距离容差，迭代与容差来自 parameter artifact。

有符号 point distance 减去 `rho_m` 得到 support sphere 距离；要求间距仅用于计算 `remaining_margin_mm`。双方 witness 满足位移沿 ellipsoid 外法线，长度与有符号距离一致。组合几何返回所有有效 pair 中最小的有符号 pair 距离，平局按固定 slot/support 顺序选择。

`DISTANCE_MM` 仅启用 `distance_valid_mask`。它不执行 self DCOL，不生成 `valid_mask`、`state_valid_mask` 或 `state_valid`。空 support 没有最近 pair，结果保持距离无效 mask；不会生成无限距离作为测量值。

## 身份与拒绝

scene 增加逐 support 的明确 track 状态输入；primitive id 继续使用 `link_index * slot_capacity + slot_index`。距离结果显式给出 link index、slot index 与 support id，保留 scene header。公共 header 的 `source_stamp_ns` 持续携带输入场景采集时间，包括失败结果。活动 support id 必须非负且唯一。

任一活动 pair 数值或距离健康失败时，整体状态为 `SOLVER_UNHEALTHY`，所有准入 mask 无效。scene 和 deadline 继续使用现有拒绝路径。JIT 编译计入运行时间，测试分别检查数值输出与预热后的状态。
