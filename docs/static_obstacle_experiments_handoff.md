# 静态实验前期证据与保存位置

2026-10-27，静态实验已取消。#171、#182、#183 以 `not_planned` 关闭；#172–#181 保持已完成。上述 13 项均移出当前地图。项目开发入口为[完整项目地图](planning/nineaxis-full-project/README.md)。

## 已完成任务

| issue | 已有能力 |
|---|---|
| #172 | S0 仿真与反馈评价 |
| #173 | 共享路径参考的五维跟随 |
| #174 | 单障碍臂身避让 |
| #175 | 非相邻连杆 mesh 自碰撞约束 |
| #176 | Ruckig 后的命令检查与进度结算 |
| #177 | S2–S5 场景与可行性见证 |
| #178 | 构型调整与基础暂停恢复 |
| #179 | 关节速度与进给联合求解 |
| #180 | 持续确认与渐增进给恢复 |
| #181 | Generalized-SNS 外部参考 |

各项完成范围以原 issue 和[历史接手记录](static_obstacle_experiments_history.md)中的真实验收为准。主线通过 #148 核对复用能力；#144 负责全项目验收定义，#169 负责三方法验证、周期测量与正式统计。

## 原始记录

2026-10-26 的容量核查见[资料依据](planning/nineaxis-full-project/requirements-evidence.md)。以下数值对应该次核查：

| 内容 | 保存范围 |
|---|---|
| `output/static-*` | 9,004,523,520 bytes |
| 全部 `output/` | 12,457,467,904 bytes |
| 20 份报告关联数组 | 60 个数组与报告共同保存 |
| NURBS、MATLAB、标定和原始模型 | 原路径、校验和与资料身份 |

历史共同周期保存 A/B 在 20 ms 下的两份失败报告，测量范围为 2/20；正式采集为 0/1080，`formal_bound=false`。这些结果的运行身份见[原始记录](../.trellis/tasks/10-20-static-formal-experiments/research/proposal-pause.md)。

历史卷身份：

- UUID `083CBB753CBB5BFE`，路径 `/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/`。
- UUID `6C001EAA001E7B72`，路径 `/media/lsn/Windows/robot_safecontrol_static_experiments_20261021/`。

恢复访问后依据 manifest 与 SHA-256 核验。数据复现使用[历史程序说明](static_obstacle_experiments_legacy.md)和[存储说明](static_experiment_storage.md)，保留每份记录对应的源码、配置和评价身份。
