---
status: accepted
date: 2026-09-21
---

# 规划边与轨迹前视采用自适应碰撞区间证明

碰撞安全模块对关节区间 `q(s) = q_a + s * (q_b - q_a)` 计算 link 最大运动范围，并据此
得到全部有效 self 与 environment pair 的 `proximity_scale` 下界。下界达到对应
`scale_margin` 时整段通过；无法证明时递归二分区间。达到计算深度或时间预算仍未获得
证明时，该区间判为未通过。

规划器的边有效性和轨迹执行器的前视验证必须调用同一个区间验证器。动态 support 的预测
范围与观测年龄需要覆盖所验证的时间区间，验证结果携带 `scene_revision` 与
`geometry_hash`。

## Considered Options

曾考虑对每条关节区间求解全局最小 `proximity_scale`。articulated link 的旋转和多个
pair 会形成高维非凸问题，需要具有全局下界证明的区间优化器。自适应运动上界可以复用
JAX 运动学与现有 scale 查询，因此采用该方法。

## Consequences

状态端点通过不能单独代表边有效。区间验证器需要经过独立数值验证，证明运动上界不会低估
任意机器人 ellipsoid 的位姿变化；计算预算不足的结果必须保持“未通过”，不能转为安全。
规划与执行的 revision 规则见
[ADR 0021](0021-frozen-scene-planning-and-latest-admission.md)。
