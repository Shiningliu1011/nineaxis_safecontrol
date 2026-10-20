# 连续运动范围

已读取 Ruckig v0.12.2 的 trajectory.hpp、profile.hpp 和 Python 绑定全文。
Python 公开 `Trajectory.profiles`，包括制动前段、七个恒 jerk 区间以及末端状态。
本项目使用单段 Velocity 控制，无路径中间点。

将实际公开系数交给 SciPy PPoly；查询位置、速度和加速度的连续极值，
覆盖一个控制周期及随后完整停止运动。轨迹短于控制周期时，按 Ruckig 的
末端恒加速度延伸计算。导数根与区间边界上的状态由 Ruckig.at_time 求值。
SciPy PPoly 的恒零区间根返回 NaN，该标记按文档语义处理。

连续几何证明采用每个关节相对区间中心的最大位置变化，以及机器人刚体
位置变化系数。每个要求组合都保留独立间距检查。

实际状态来源：`output/static-experiments/s1-joint-full-geometry-02`。
重放该运行的末次候选，五次候选计算平均 21.509 ms。
该数值只代表此状态的候选计算，完整闭环周期另行记录。
