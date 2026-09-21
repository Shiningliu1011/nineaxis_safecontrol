---
status: accepted
date: 2026-09-21
---

# LiDAR 环境采用带射线更新的局部占据场景

碰撞安全模块接收由 LiDAR 射线更新的局部占据场景，明确保存 `occupied`、`free` 和
`unknown`。射线终点提供占据证据，传感器到终点之间提供自由证据，未观测区域保持未知；
每次有效更新生成新的 `scene_revision`。`occupied` voxel 生成 ellipsoid–point 查询的
support points，`free/unknown` 进入规划、轨迹执行和 OSCBF 的覆盖准入。

## Considered Options

曾考虑让最新单帧点集直接进入碰撞查询。该方法无法区分自由空间、遮挡区域、视场外区域
和无返回区域，障碍暂时被遮挡后也会立即退出输入，因此未采用。

## Consequences

`CollisionScene` 必须保存传感器原点、射线更新结果、采集时间、覆盖状态与场景版本。
无返回区域不能自动作为自由空间。占据证据的清除与过期规则见
[ADR 0015](0015-asymmetric-occupancy-evidence.md)，运动误差参数仍需单独确定。经验证固定环境
模型与 LiDAR occupied 的并集规则见 [ADR 0006](0006-continuous-obstacle-observation.md)。
