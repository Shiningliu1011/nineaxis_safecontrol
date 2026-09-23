## Directory Map

| Path | Purpose |
|------|---------|
| `src/robot_safecontrol_moveit/` | 主 ROS2 Python 包（节点 + 纯逻辑模块） |
| `src/aeb_rrtstar/` | 独立 Python AEB-RRT* 规划器 + 基准测试（不依赖 ROS） |
| `src/aeb_rrtstar_ompl/` | 嵌套 C++ MoveIt2 OMPL 插件包 |
| `portable_oscbf/` | 可移植 JAX OSCBF 控制核心（work/ 为 Python 包，随包分发） |
| `portable_oscbf/work/` | 控制内核 Python 包（零 ROS 依赖，统一 `from work.X` import） |
| `portable_oscbf/vendor/dpax/` | 内嵌的 DCOL 可微碰撞库 |
| `portable_oscbf/tests/` | 控制核心独立测试 |
| `portable_oscbf/scripts/` | 内核调试/标定脚本 |
| `models/ninezzhou/` | 9 轴机械臂 URDF + STL 网格 |
| `models/ninezzhou_moveit_config/` | MoveIt2 配置（SRDF、控制器、运动学） |
| `config/` | 节点 YAML 参数（oscbf_controller.yaml、drempower.yaml 等） |
| `launch/` | launch 文件（mujoco_transition_final.launch.py 为完整闭环） |
| `data/nurbs/` | NURBS 轨迹数据（ik_input.mat 逆运动学输入） |
| `tests/` | 主包 pytest 测试（含 launch 集成测试） |
| `scripts/` | 辅助脚本（零位标定、vcan 测试、清场启动） |
| `docs/` | 文档（本指南、ADR、specs、runbook） |
| `output/` | 生成文件（已 gitignore） |
| `build/ install/ log/` | colcon 产物（已 gitignore） |
