# 现有项目怎样提供标定所需的关节与模型数据

类型：`wayfinder:research`；处理方式：AFK；状态：研究完成；负责人：Lsn。

归属：[激光跟踪仪零位标定准备](../README.md)。研究资料由当前 Trellis 任务保存。

## Question

当前源码中编码器到关节坐标的换算、J1 传动输入、名义 URDF、末端 J9 轴线与 tool0、现有 FK 接口和真实反馈分别处于什么状态？哪些接口可以为离线标定提供依据，哪些数值仍需现场核实？

按实际代码和模型给出证据。若能使用现成 URDF 解析库与矩阵工具，以纯离线计算核查 J9 轴线与 tool0 的几何关系；任何名义模型结果均须标明适用范围。

## Resolution

2026-10-26 完成源码与名义模型核查。现有换算接口明确使用编码器输出轴 degree；J1 需要经确认的传动数据。名义 URDF 与 `NineaxisKinematics` 的所检查配置一致，tool0 距 J9 轴线 0.235 m；真实球心安装坐标与测量记录仍需取得。

完整结论、源码位置及实际命令见[项目接口研究](../../../../.trellis/tasks/10-26-laser-tracker-calibration-preparation/research/project-interfaces.md)。该研究给出后续接口设计依据，实机标定状态由实际测量决定。
