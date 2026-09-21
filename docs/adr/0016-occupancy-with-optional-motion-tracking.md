---
status: accepted
date: 2026-09-21
---

# 占据场景之上增加可选运动跟踪层

局部占据场景负责障碍是否存在，运动跟踪层只为具有可信 track 的关联 support points
提供速度、预测范围和 barrier 时间项。关联失败、速度状态无效或 track 过期时，support
points 使用障碍速度上界与观测年龄增加范围；track 消失不能删除 occupied voxel。

## Considered Options

曾考虑不做跨帧运动估计，并让所有 occupied voxel 使用统一速度上界。该方法保持安全
含义简单，但移动人员或设备会产生更大的运动范围和更多停止，因此采用可选跟踪增强。

## Consequences

track 必须保留来源时间、关联状态、速度有效性与误差范围。可信 track 通过
`partial_h_partial_t` 进入 OSCBF，并向规划前视区间提供预测范围；未通过有效性判断的
track 立即使用速度上界路径。占据场景仍是规划、轨迹执行和 OSCBF 的共同障碍来源。
