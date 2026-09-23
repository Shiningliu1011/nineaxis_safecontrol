# robot_safecontrol

9 自由度（1P8R）机械臂安全控制项目，使用 MuJoCo、ROS 2 Humble、MoveIt2 与 JAX OSCBF 控制内核。任务采用 5D 工具轴路径跟随，控制末端位置与工具轴方向。当前仓库包含仿真闭环、感知与硬件接口，以及主包内的 MID-360 / MID-360S 驱动。当前硬件模式与验证范围见对应文档。

## 文档入口

- [文档总目录](docs/README.md)：按主题查找说明、设计、运行资料与历史证据。
- [项目入门](docs/ONBOARDING.md)：项目结构、运行入口和状态流。
- [运行与测试](docs/PROJECT_GUIDE.md)：仿真启动、生产配置、验证入口和硬件模式。
- [领域词汇](docs/CONTEXT.md)：机器人、轨迹、坐标和安全术语。
- [控制内核](docs/modules/portable_oscbf.md)与[MID-360 驱动](docs/modules/livox_mid360.md)：模块说明。
- [文档维护规则](docs/documentation.md)：目录、内容组织与功能修改时的同步要求。

设计目标、当前代码行为和历史验收分别核对；运行状态以当前代码及测试结果为准。
