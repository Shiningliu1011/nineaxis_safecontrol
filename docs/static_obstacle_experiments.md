# 静态障碍物仿真实验

[规格说明书](specs/static-obstacle-experiments.md)定义九轴机械臂在固定障碍物环境中的路径跟随、臂身避让、进给协调和构型调整后恢复。对应的 [GitHub 规格为 #171](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/171)。

## 实施任务

[任务编号与前置关系](planning/nineaxis-full-project/dependencies.md#静态障碍物仿真实验)列出 61–72 号任务及各自的直接前置，GitHub issue 为 #172–#183。每项任务保存交付行为及验收条件。

可以从 [61 可重复的 S0 仿真与反馈评价](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/172)开始。共同入口完成后，五维参考与外部障碍物接入可以分别推进；A 组完成后，联合求解与改进恢复可以分别推进。

## 实验要求

- 场景包含 S0 基础跟随、S1 单障碍、S2 多连杆受限、S3 行程受限、S4 构型调整后恢复、S5 持续阻塞。
- 方法包含 A/B/C/D 两因素对照和 E 组 Generalized-SNS，全部方法共享场景、限制、构型辅助和评价。
- 位置与工具轴使用同一个路径参考；TCP 作业距离为 50 mm，外部间距为 30 mm，非相邻连杆间距为 10 mm。
- 正式实验采用三组初态、三档名义进给，每个组合重复三次；完整周期测量确定共同控制周期。
- 路径距离 RMS/P95/最大值要求为 0.15/0.5/2 mm；真实工具轴夹角 RMS/最大值要求为 0.05°/0.5°。

规格与任务已发布，功能和实验结果按各任务的验收条件记录。当前仿真执行器及跟踪结果见[跟踪平顺性](tracking_smoothness.md)，当前评价接口见[跟踪评价](tracking_evaluation.md)。
