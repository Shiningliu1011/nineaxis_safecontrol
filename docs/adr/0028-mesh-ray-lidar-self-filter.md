---
status: accepted
date: 2026-09-21
---

# LiDAR 自体过滤采用无 MoveIt 的 mesh 射线查询

自体过滤使用机器人 collision mesh、共享运动学与 LiDAR point 采集时间对应的关节状态。
每条射线从传感器原点查询机器人 mesh 的预计首次交点：测量距离与预计表面一致时标记为
self；测量点位于预计表面之前时保留为 environment occupied；机器人表面之后的射线区域
保持 unknown，不能标为 free。

测量、标定与时间误差范围内无法明确分类的返回标记为 ambiguous，对应空间不能取得覆盖
准入。组合 ellipsoid 只用于碰撞查询，不能作为删除 LiDAR point 的自体过滤几何。

## Considered Options

曾考虑使用组合 ellipsoid 删除内部与附近点。该包络包含机器人 mesh 外的空间，可能删除
紧邻机械臂的外部障碍，因此采用 mesh 射线查询。

## Consequences

LiDAR 输入必须提供可用的 point 时间信息，关节状态历史必须覆盖完整扫描区间。mesh、
运动学、传感器外参或时间状态无效时，自体过滤结果无效，不能生成可准入的
`CollisionScene`。
