# 文档总目录

本文只提供主题入口。每篇说明聚焦一个主题；详细步骤、依据和历史记录从对应页面继续查阅。

## 项目与运行

- [项目入门与结构](ONBOARDING.md)：源码入口、闭环数据流和开发环境。
- [运行与测试](PROJECT_GUIDE.md)：仿真、配置、测试及硬件模式。
- [跟踪评价](tracking_evaluation.md)：指标、证据和报告。
- [真机操作手册](real_robot_runbook.md)与[笔记本连接检查](hardware_laptop_quickstart.md)：硬件操作入口。

## 模块与领域

- [领域词汇](CONTEXT.md)：系统结构、机器人、轨迹、观测和安全要求。
- [控制内核](modules/portable_oscbf.md)：模块、依赖和移植入口。
- [MID-360 / MID-360S 驱动](modules/xy-mid-360-s.md)：安装、设备接口和验证范围。
- [传感器布局与标定](sensor_layout_and_calibration.md)：坐标和安装方案。

## 设计与实施资料

- [设计决定](adr/)：按编号阅读已记录的决定。
- [规格](specs/)：各主题的详细要求。
- [OSCBF 移植指南](guides/OSCBF_PORTING_GUIDE.md)与[执行计划](guides/OSCBF_EXECUTION_PLAN.md)：移植目标和阶段安排。
- [复用研究索引](planning/oscbf-reuse/README.md)：研究、交接和验证证据。
- [代码审查检查项](CODING_STANDARDS.md)与[经验教训](LESSONS_LEARNED.md)：修改相关模块时按主题查阅。

## 文档维护

[维护规则](documentation.md)说明存放位置、页面组织、功能修改时的同步要求，以及定期检查方法。
