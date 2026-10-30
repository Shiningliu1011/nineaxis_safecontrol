# 文档总目录

本文只提供主题入口。每篇说明聚焦一个主题；详细步骤、依据和历史记录从对应页面继续查阅。

## 项目与运行

- [九轴手动定位](joint_positioning.md)：Python 顶部修改九轴绝对目标、CSV 按 Enter 逐组定位及到位后的持续保持与监测。
- [电机定位诊断](motor_positioning_diagnostics.md)：G 系列连续计数、有人值守动作记录、原始帧和离线复算。
- [同步接口与断电恢复](planning/motor-positioning/live-interface-experiments.md)：共同时间与连续参考实机比较、非零位置断电记录和上电参数变化。
- [标定姿态表](calibration_joint_table.md)：相机与激光跟踪仪分别采集，完整关节目标输入、模型检查和离线逐组流程。
- [运动参数与反馈证据](joint_motion_limits_evidence.md)：位置模式、速度换算、完整真实回放及当前零位与通信检查。
- [项目入门与结构](ONBOARDING.md)：源码入口、闭环数据流和开发环境。
- [Linux 主机迁移](project_migration.md)：完整资料包、skills、环境恢复与接手提示词。
- [双设备工作流程](two_machine_workflow.md)：笔记本与台式机通过 GitHub 交替开发、同步进度和传输数据。
- [运行与测试](PROJECT_GUIDE.md)：仿真、配置、测试及硬件模式。
- [跟踪评价](tracking_evaluation.md)：指标、证据和报告。
- [跟踪平顺性](tracking_smoothness.md)：控制周期、执行器运动参数和全过程图片。
- [跟踪执行与过渡结果](tracking_execution.md)：公开入口、状态归属、报告生命周期和验证入口。
- [真机操作手册](real_robot_runbook.md)与[笔记本连接检查](hardware_laptop_quickstart.md)：硬件操作入口。
- [三路九个电机通信记录](drempower_communication_20261016.md)：500 kbps 设置、实际验证与参数恢复方法。
- [电机方向配置与恢复记录](motor_direction_change_20261021.md)：关节方向符号、生效条件和手动恢复步骤。
- [单关节低速执行与异常处理](single_joint_execution.md)：反馈、限位、到位判定、异常保持和现场验证范围。
- [关节动作实验问题与跨设备交接](joint_motion_handoff_20261028.md)：J1 模型起点、九轴现场问题、修复入口与真实数据位置。

## 模块与领域

- [领域词汇](CONTEXT.md)：系统结构、机器人、轨迹、观测和安全要求。
- [全项目单位约定](units.md)：项目输入与报告的 mm/°、计算内核的 m/rad 及边界换算。
- [管道路径与运动 Python 工具](pipe_assets.md)：曲面模型、路径画笔、NURBS、进给插补与连续姿态生成。
- [控制内核](modules/portable_oscbf.md)：模块、依赖和移植入口。
- [独立碰撞场景服务](modules/collision_scene_server.md)：逐点数据入口、场景来源、双缓冲共享内存及消费者恢复。
- [MID-360 / MID-360S 驱动](modules/livox_mid360.md)：安装、设备接口和验证范围。
- [传感器布局与标定](sensor_layout_and_calibration.md)：坐标和安装方案。
- [雷达与相机相对外参](lidar_camera_calibration.md)：当前安装关系的标定结果、静态多姿态测量、独立验收与本机复现命令。

## 设计与实施资料

- [九轴电机定位与 J1 全行程规格](planning/nineaxis-full-project/motor-positioning-spec.md)：九值 Python 入口、同步定位、平滑往返、定位精度与设备调查。
- [九轴电机独立任务清单](planning/motor-positioning/tasks.md)：#184 的 11 项子任务，覆盖单组定位、J1 完整行程与蝴蝶曲线跟踪，与 #147 并列管理。
- [预测协调控制总体规格](specs/predictive-coordination-control.md)：研究目标、工艺与电机限制、运动参考协调、MPC、三方法和完整项目验收。
- [2026-10-20 主机恢复](host_recovery_20261020.md)：环境、skills、字体、资料身份和验证入口。
- [九轴曲面作业完整项目](planning/nineaxis-full-project/README.md)：M01–M12、QP/MPC、系统集成、实机实验与个人项目成果的 Wayfinder 入口。
- [激光跟踪仪零位标定准备](planning/laser-tracker-calibration/README.md)：测量条件、可辨识性、现场采集、离线求解与独立验收的本地规划资料。
- [设计决定](adr/)：按编号阅读已记录的决定。
- [规格](specs/)：各主题的详细要求。
- [复用研究索引](planning/oscbf-reuse/README.md)：研究、交接和验证证据。
- [代码审查检查项](CODING_STANDARDS.md)与[经验教训](LESSONS_LEARNED.md)：修改相关模块时按主题查阅。

## 历史资料

- [静态实验前期资料](static_obstacle_experiments.md)与[证据保存位置](static_obstacle_experiments_handoff.md)：已完成基础能力和原始数据。
- [静态障碍物开题资料](proposal-static-obstacles/README.md)：前期实验数据、文字、图片和跨设备使用说明。
- [静态实验紧凑存储](static_experiment_storage.md)：记录读取、旧数据转换与恢复验证。
- [OSCBF 移植指南](archive/OSCBF_PORTING_GUIDE.md)与[执行计划](archive/OSCBF_EXECUTION_PLAN.md)：旧阶段目标和实施记录，当前功能以源码与测试为准。
- [静态实验历史规格](archive/static-obstacle-experiments-20261020.md)、[历史程序说明](static_obstacle_experiments_legacy.md)与[历史接手记录](static_obstacle_experiments_history.md)：原五组实验的要求、程序入口和原始证据。

## 文档维护

[维护规则](documentation.md)说明存放位置、页面组织、功能修改时的同步要求，以及定期检查方法。
