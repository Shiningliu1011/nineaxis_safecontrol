# 文档总目录

本文只提供主题入口。每篇说明聚焦一个主题；详细步骤、依据和历史记录从对应页面继续查阅。

## 项目与运行

- [项目入门与结构](ONBOARDING.md)：源码入口、闭环数据流和开发环境。
- [Linux 主机迁移](project_migration.md)：完整资料包、skills、环境恢复与接手提示词。
- [双设备工作流程](two_machine_workflow.md)：笔记本与台式机通过 GitHub 交替开发、同步进度和传输数据。
- [运行与测试](PROJECT_GUIDE.md)：仿真、配置、测试及硬件模式。
- [跟踪评价](tracking_evaluation.md)：指标、证据和报告。
- [跟踪平顺性](tracking_smoothness.md)：控制周期、执行器运动参数和全过程图片。
- [跟踪执行与过渡结果](tracking_execution.md)：公开入口、状态归属、报告生命周期和验证入口。
- [真机操作手册](real_robot_runbook.md)与[笔记本连接检查](hardware_laptop_quickstart.md)：硬件操作入口。
- [三路九个电机通信记录](drempower_communication_20261016.md)：500 kbps 设置、实际验证与参数恢复方法。

## 模块与领域

- [领域词汇](CONTEXT.md)：系统结构、机器人、轨迹、观测和安全要求。
- [全项目单位约定](units.md)：项目输入与报告的 mm/°、计算内核的 m/rad 及边界换算。
- [控制内核](modules/portable_oscbf.md)：模块、依赖和移植入口。
- [MID-360 / MID-360S 驱动](modules/livox_mid360.md)：安装、设备接口和验证范围。
- [传感器布局与标定](sensor_layout_and_calibration.md)：坐标和安装方案。
- [雷达与相机相对外参](lidar_camera_calibration.md)：静态多姿态测量、独立验收结果与本机复现命令。

## 设计与实施资料

- [静态障碍物仿真实验](static_obstacle_experiments.md)：规格、S0–S5 场景、A/B/C/D/E 对照及实施任务。
- [静态实验接手记录](static_obstacle_experiments_handoff.md)：当前验证、数据容量、保存位置和剩余工作。
- [静态障碍物开题资料](proposal-static-obstacles/README.md)：前期实验数据与结论、可修改文字、图片和跨设备使用说明。
- [静态实验紧凑存储](static_experiment_storage.md)：无损记录、旧数据转换、按需数组及恢复验证。
- [2026-10-20 主机恢复](host_recovery_20261020.md)：环境、skills、字体、资料身份和验证入口。
- [九轴曲面作业完整项目](planning/nineaxis-full-project/README.md)：M01–M12、QP/MPC、系统集成、实机实验与个人项目成果的 Wayfinder 入口。
- [激光跟踪仪零位标定准备](planning/laser-tracker-calibration/README.md)：测量条件、可辨识性、现场采集、离线求解与独立验收的本地规划资料。
- [设计决定](adr/)：按编号阅读已记录的决定。
- [规格](specs/)：各主题的详细要求。
- [复用研究索引](planning/oscbf-reuse/README.md)：研究、交接和验证证据。
- [代码审查检查项](CODING_STANDARDS.md)与[经验教训](LESSONS_LEARNED.md)：修改相关模块时按主题查阅。

## 历史资料

- [OSCBF 移植指南](archive/OSCBF_PORTING_GUIDE.md)与[执行计划](archive/OSCBF_EXECUTION_PLAN.md)：旧阶段目标和实施记录，当前功能以源码与测试为准。

## 文档维护

[维护规则](documentation.md)说明存放位置、页面组织、功能修改时的同步要求，以及定期检查方法。
