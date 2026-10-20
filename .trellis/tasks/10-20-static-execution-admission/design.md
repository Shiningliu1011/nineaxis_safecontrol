# 运动与证据设计

`static_motion` 生成带停止过程的 Ruckig 候选；`static_motion_bounds` 使用
Ruckig 公开分段系数与 SciPy PPoly 求连续范围；连续几何证明由公共查询完成。
`static_execution` 在指定的 2 ms 积分步激活状态一致且已通过检查的候选。

controller 与 plant 分别记录求解、发送、准入及实际反馈。
独立评价读取实际激活的 command_id，核查 plant 的准入记录，并用
`reference_start + feed * min(elapsed, period)` 重算每个积分步的参考进度。
缺少任一对应记录返回证据不足；拒绝候选被激活或进度不一致返回失败。

运行 manifest 保存实际关节、速度、加速度和 jerk 限制。
完整周期从候选开始计算至准入回复与同步记录结束，统计 P99.9、最大值及超期。
按弧长等距采样实际有序进度，避免暂停和终点保持时间改变弧长统计权重。
