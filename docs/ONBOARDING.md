# 项目入门

本项目通过 ROS 2、MoveIt2、MuJoCo 和 JAX OSCBF 形成九轴机械臂仿真闭环。按需要阅读对应主题；当前能力以源码和测试为准。

MID-360 / MID-360S 驱动维护来源位于 `src/robot_safecontrol_moveit/livox_mid360/`，使用说明见[驱动文档](modules/livox_mid360.md)。AEB-RRT* 插件的四个 ctest 已纳入[全量测试入口](project-guide/testing.md)。

- [项目概览](onboarding/overview.md)
- [使用的技术](onboarding/technologies.md)
- [系统结构](onboarding/architecture.md)
- [目录说明](onboarding/directories.md)
- [闭环数据流](onboarding/lifecycle.md)
- [代码约定](onboarding/conventions.md)
- [常用操作](onboarding/common-tasks.md)
- [主要入口](onboarding/entry-points.md)
- [相关文档](onboarding/documents.md)
