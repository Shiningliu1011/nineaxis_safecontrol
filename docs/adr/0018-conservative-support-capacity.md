---
status: accepted
date: 2026-09-21
---

# Support point 容量采用保守合并与超限拒绝

`CollisionScene` 使用固定 `MAX_SUPPORT_POINTS` 和 `active_mask`。support point 超过容量时，
邻近 point 通过新的中心 `c` 与
`rho_merged = max_j(norm(p_j - c) + rho_j)` 保守合并，确保新的米制范围完整覆盖全部输入
范围。若无法在允许的范围与计算时间内证明覆盖，场景状态变为 `CAPACITY_OVERFLOW`，规划请求
被拒绝，轨迹执行与 OSCBF 输出保持命令。

## Considered Options

超出容量后立即拒绝场景同样具有明确安全含义，但高密度点云会产生更多停止。保守合并
可以在保持固定 JAX 形状的同时继续使用完整占据证据，因此将拒绝运动作为合并失败后的
处理。

## Consequences

合并过程必须保留来源时间、最大运动范围和最严格有效期，不能通过选择较近 point 或截断
数组来满足容量。`MAX_SUPPORT_POINTS` 由 LiDAR 覆盖范围、voxel 尺寸、合并范围和控制
周期测试共同确定。
