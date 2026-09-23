# 仓库目录

- `src/robot_safecontrol_moveit/`：ROS 2 Python 包，包含 `livox_mid360/` 驱动、规划服务器和控制节点。
- `src/aeb_rrtstar_ompl/`：当前 MoveIt2 AEB-RRT* 插件及其四个 ctest。
- `portable_oscbf/`：JAX 控制内核与独立测试。
- `models/ninezzhou/`：九轴机械臂 URDF、网格与关节配置；`urdf/ninezzhou.urdf` 是运动学和位置限幅的手工来源。
- `models/ninezzhou_moveit_config/`：MoveIt2 的 SRDF、控制器和规划配置。
- `config/`、`launch/`：生产参数与 ROS 进程入口。
- `tests/`：主包测试，包含 MID-360 协议与节点测试。
- `docs/archive/`：旧 OSCBF 移植指南和阶段执行计划。
- `build/`、`install/`、`log/`、`output/`：本地生成内容。
