# [07A] 冗余策略对比：9-DOF 对 5D 工具轴任务

- Tracker：[冗余策略对比：9-DOF 对 5D 工具轴任务](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/5)
- 日期：2026-09-10；本票为 AFK research，研究结论不代替下游人审。
- 起点：`786e0aec61b7abbf8e8c0a5c5aa99d20c3e54f21`；主目录 tracked diff 为空，既有未跟踪标定、驱动资料保留。
- 研究保存于独立 `research/ticket5-redundancy` 分支；本目录同步可审阅副本，不修改运行控制参数。

## 完成证据

见[研究报告](../research/5-redundancy-strategies.md)、[原始结果](../research/5-redundancy-evidence/summary.json)、[环境与源码指纹](../research/5-redundancy-evidence/metadata.json)及同目录逐步记录。

复用本地 JAX/CBFpy/qpax、九轴运动学、5D 任务和现有策略接口，完成默认、关节中点、现有可操作度梯度三种候选的隔离比较。两个既有测试初态，每种各 300 步、dt=0.01s，总计 1800 步；使用固定末端目标和 Euler 积分，无 ROS、外部障碍或硬件发布权。

所有步 QP 返回成功，采样 5D Jacobian 满秩、零空间维度为 4。关节中点偏好与现有梯度在两个初态下都降低了最小归一化限位余量，不能作为限位保护的充分手段。最大位置偏差分别约 0.493 mm、0.463 mm；默认策略固定端点误差接近数值水平。这不是动态路径上的策略胜负结论。

首次启动因未配置仓库 vendored dpax 的导入路径退出；随后通过现有 vendor 路径重跑成功，没有安装或升级依赖。最终命令和范围写在证据 README，6 个汇总与 1800 条有限数值原始记录核对一致。输出最大 elastic slack 约 6.99e-9，约束活跃步比例为零；不将此无冲突工况解释成硬约束或避障验收通过。

## 用户决定与剩余选择

用户本轮指令：“处理下一个ticket”。继承已经接受的九轴/5D、roll 自由、冗余优先、必要降进给、OBB 和故障规则。本票只提出候选与证据，**没有新的人审决定**。

建议保留当前默认作为比较基线，继续评估考虑限位的 5D 冗余偏好；这项建议尚未成为生产选型。具体目标、严格层级或容差表达、归一化尺度和权重、激活/恢复条件由[选定冗余目标与优先级](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/16)向用户展示后收敛。

## 编码门与复验

可以继续做隔离策略比较；**尚不能据本票启用生产零空间策略或宣称控制接入完成**。生产实施需上票人审输出明确的目标、优先关系和验收口径，再交[控制模块复用与九轴适配](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/32)。

后续复验必须覆盖完整路径、近奇异、限位/自碰撞/环境同时活跃、原速可行与降进给可行、失败/松弛/最终命令门控和端到端时延；几何真值仍受[自碰撞 OBB 与环境几何模型核对](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/8)约束。既有[CBF/QP 可行性实证测量](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/6)承接接入后的实测，不新建同义票或反向加入循环依赖。

复用结论：保持现有统一控制入口与求解基础设施；上游版本、许可证、接口适配与不直接替换的依据详见研究报告。没有证据支持本轮更换求解器或重写通用算法。

## 回退与下一窗口

本次只有研究文档与离线测量资产，无运行行为变化。回退只撤销本票新增资料，不清理其他窗口文件。下一窗口为[选定冗余目标与优先级](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/16)，本次不自动认领或开始。
