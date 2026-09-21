---
status: accepted
date: 2026-09-21
---

# 任务起点采用 JAX multi-start IK 目标集合

JAX multi-start IK 使用多组确定性初始关节状态，为任务起点生成多组冗余关节构型。每个
候选必须满足关节限位、末端位置、工具轴误差和当前 `CollisionScene` 的 state collision
条件；重复候选被合并，不同冗余构型组成 IK 目标集合。AEB-RRT* 可以连接集合中的任意
有效目标，trajectory optimization 联合比较过渡段与完整任务名义轨迹代价。

最终候选和完整轨迹必须经过当前 revision 的自适应区间证明，IK 通过本身不产生执行准入。

## Considered Options

曾考虑只使用一组 IK 目标。九轴冗余与自由 roll 允许多个任务等价构型，单个解可能靠近
关节限位、自碰撞或外部障碍，并提前限制全局规划，因此采用目标集合。

## Consequences

multi-start 初始状态集合、候选去重阈值、末端位置与工具轴容差必须固定记录。有效目标集合
为空时规划请求失败，不能放宽碰撞、关节或任务约束来生成目标。
