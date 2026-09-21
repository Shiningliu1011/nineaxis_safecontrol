---
status: accepted
date: 2026-09-21
---

# 占据场景采用非对称证据规则

一次时间与坐标均有效的 LiDAR hit 立即把对应 voxel 标记为 `occupied`。只有连续、独立且
时间有效的 free 射线证据才能把 occupied 转为 `free`；时间经过本身只能让陈旧 occupied
转为 `unknown`。有效期内的 occupied 按年龄增加障碍运动范围，每次状态或有效范围变化都
生成新的 `scene_revision`。

## Considered Options

曾考虑让最新射线直接覆盖旧状态。该方法可以更快清除移动障碍留下的占据，但单次噪声、
外参误差或时间不同步也可能直接删除仍然存在的障碍，因此未采用。

## Consequences

局部占据场景必须为 voxel 保存最近 hit、free 证据序列、状态年龄与来源时间。连续 free
证据数量、有效时间范围、运动速度上界和 unknown 转换阈值需要由 LiDAR 数据与停车能力
验证后确定，不能仅按控制频率设置。动态运动估计见
[ADR 0016](0016-occupancy-with-optional-motion-tracking.md)。
