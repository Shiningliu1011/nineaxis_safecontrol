---
status: accepted
date: 2026-09-21
---

# 自研规划器采用 AEB-RRT* 与 JAX trajectory optimization

规划阶段不再使用 MoveIt/OMPL/FCL 运行路径。自研 AEB-RRT* 在九轴关节空间搜索全局
连通路径，状态与边全部调用共享碰撞安全模块；随后 JAX trajectory optimization 以该
路径为初始值，优化轨迹长度、平滑程度、时间和安全余量。两个阶段使用同一
`CollisionScene`、机器人 geometry、scale 语义与 query kernel。

JAX 优化后的全部状态和区间必须重新执行自适应碰撞区间证明。优化失败不能改变原路径的
证明内容，任何未经重新证明的优化结果都不能进入执行准入。

## Considered Options

单独使用 AEB-RRT* 能提供全局连通搜索，但不能直接利用碰撞梯度改善轨迹。单独使用 JAX
trajectory optimization 可以利用梯度，但对初始轨迹敏感。组合方法同时保留全局搜索与
梯度优化能力，因此采用该方法。

## Consequences

AEB-RRT*、trajectory optimization 和区间证明需要共享关节顺序、限制、scene identity
与 geometry identity。规划输出必须是带时间信息并取得最新场景准入的轨迹，不能只返回
未经验证的几何节点序列。规划任务范围见
[ADR 0029](0029-full-task-nominal-trajectory.md)，任务起点目标生成见
[ADR 0033](0033-multistart-ik-goal-set.md)。
