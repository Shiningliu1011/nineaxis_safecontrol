# robot_safecontrol — Onboarding Guide
**Generated:** 2026-08-22  **Stack:** ROS 2 Humble / MoveIt 2 / MuJoCo / JAX / C++ OMPL

## Overview

9-DOF 冗余机械臂（1 棱柱关节 J1 + 8 旋转关节 J2-J9）的安全控制项目：在 MuJoCo
物理仿真中，机械臂从任意随机工作位姿出发，自动规划无碰撞过渡（AEB-RRT*）到
蝴蝶形参考轨迹起点，随后 OSCBF 安全控制器接管，在保证碰撞/关节限位安全的前提下
完成末端轨迹跟踪（基于 Morton & Pavone, *Safe, Task-Consistent Manipulation with
OSCBF*, IROS 2025）。全流程无需键盘，一键运行。

## Tech Stack

| Layer | Technology | Version |
|-------|-----------|---------|
| ROS 2 | Humble (rclpy / rclcpp) | Humble |
| Python 包 | robot_safecontrol_moveit (ament_python), setup.py | 0.1.0 |
| 运动规划 | MoveIt 2 + pymoveit2 + OMPL 插件 | — |
| 物理仿真 | MuJoCo | — |
| 控制内核 | JAX (JIT), qpax 弹性 QP, cbfpy 框架 | — |
| 碰撞 | FCL, OBB 包络, DCOL, 17-球模型 | — |
| C++ 插件 | aeb_rrtstar_ompl (ament_cmake) | 0.1.0 |
| 数值 | numpy, scipy, yaml | — |

## Architecture

**ROS 2 工作区 + 嵌套包**：根目录是 `robot_safecontrol_moveit`（ament_python），
其 `src/aeb_rrtstar_ompl` 是嵌套的 ament_cmake 包（MoveIt2 OMPL 插件），普通
`colcon build` 发现不了它，必须用 `build_aeb_moveit.sh` 指定 `--base-paths`。

**控制闭环（单条链路）**：MuJoCo 查看器发布 `/mujoco_joint_states` →
`oscbf_controller` 节点订阅植物状态、运行纯 JAX OSCBF 内核 → 安全命令发布到
`/oscbf_command` → `oscbf_plant` 节点（带加速度/jerk 限幅的 S 曲线驱动仿真器）
积分后把状态发回 `/mujoco_joint_states`。控制器独立于 MoveIt，不依赖 move_group。

**自主过渡流程**：`transition_planning_server` 监听开始信号后，调用 move_group
的 AEB-RRT* 规划器从随机位姿规划无碰撞过渡到轨迹起点，回放到
`/transition_replay_viz`；回放结束 → OSCBF 控制器接管（`oscbf_wait_for_start`）。

**坐标系**：URDF 为 Y-up，MuJoCo 为 Z-up，程序通过 `display_frame` body 的
euler 旋转自动转换。圆柱轴心拟合口径在轨迹生成端、过渡端、控制器端必须一致
（默认最小二乘圆拟合轨迹自动求轴心）。

## Directory Map

| Path | Purpose |
|------|---------|
| `src/robot_safecontrol_moveit/` | 主 ROS2 Python 包（节点 + 纯逻辑模块：transition_executor、cylinder_geometry、robot_spec、ros_conventions） |
| `src/aeb_rrtstar/` | 独立 Python AEB-RRT* 规划器 + 基准测试（不依赖 ROS） |
| `src/aeb_rrtstar_ompl/` | 嵌套 C++ MoveIt2 OMPL 插件包 |
| `portable_oscbf/` | 可移植 JAX OSCBF 控制核心（work/ 为 Python 包，随包分发） |
| `portable_oscbf/vendor/dpax/` | 内嵌的 DCOL 可微碰撞库 |
| `models/ninezzhou/` | 9 轴机械臂 URDF + STL 网格 |
| `models/ninezzhou_moveit_config/` | MoveIt2 配置（SRDF、控制器、运动学） |
| `config/` | 节点 YAML 参数（oscbf_controller.yaml 等） |
| `launch/` | launch 文件（mujoco_transition_final.launch.py 为完整闭环） |
| `data/nurbs/` | NURBS 轨迹数据（ik_input.mat 逆运动学输入） |
| `tests/` | 主包 pytest 测试（含 launch 集成测试） |
| `docs/` | 文档（本指南、planning/aeb_rrtstar 基准报告） |
| `output/` | 生成文件（已 gitignore） |
| `build/ install/ log/` | colcon 产物（已 gitignore） |

## Request Lifecycle（闭环数据流）

1. `mujoco_viewer_with_cylinder.py` 加载 `models/ninezzhou` URDF → MuJoCo 仿真，
   发布关节状态到 `/mujoco_joint_states`（transient local QoS）。
2. `oscbf_controller.py` 订阅状态（BEST_EFFORT），加载仓库蝴蝶轨迹
   （`oscbf_trajectory.py` 统一变换），经 `portable_oscbf/work` 的
   `JaxControlLoop` facade（输入归一化 → JIT 内核：OSC + CBF + QP + 积分）
   计算安全关节速度/命令，发布到 `/oscbf_command`。
3. `oscbf_plant.py` 的 `SCurveDriverSimulator` 对命令做位置环 + 加速度/jerk
   限幅，持续发布状态回 `/mujoco_joint_states`（即使命令丢失也不会卡死，
   模拟真实编码器行为）。
4. `transition_planning_server.py` 在开始前用 move_group(AEB-RRT* 插件 +
   FCL 碰撞检查) 规划任意位姿→轨迹起点的无碰撞过渡路径。
5. 轨迹变换统一：查看器、过渡服务器、控制器共用 `oscbf_trajectory.py`，
   保证 tool0 与显示的蝴蝶曲线重合。

## Conventions Detected

- **命名**：模块小写下划线命名（`oscbf_controller.py`、`transition_planning_server.py`）；
  类用 PascalCase（`JaxControlLoop`、`SCurveDriverSimulator`）；测试 `test_*.py`。
- **节点风格**：每个节点一个模块，`main()` 入口注册为 console_scripts；
  模块级 docstring 说明职责与话题约定。
- **错误处理**：launch 文件对必需配置文件做启动时 `FileNotFoundError` 校验；
  内核侧有 QP 健康检查（`qp_solver_health.py`、`safety_snapshot.py`）。
- **测试**：pytest（`testpaths = tests`），主包含 launch 集成测试
  （launch_testing）；`portable_oscbf/tests` 有独立 conftest 与 15+ 单测
  （FK 与 URDF 一致性、QP 健康、JIT 等价性、动态障碍契约等）；C++ 包有
  自测可执行文件（test_aeb_full.cpp 等）。
- **Git**：单 main 分支；提交信息中英混合、多为 feat:/fix: 前缀或中文摘要；
  近期提交围绕 OSCBF 闭环与自主过渡。

## Common Tasks

```bash
# 构建（必须用脚本，普通 colcon build 发现不了嵌套 C++ 插件）
bash build_aeb_moveit.sh
source install/setup.bash

# 全自动演示：随机起始位姿 → 无碰撞过渡 → OSCBF 跟踪蝴蝶轨迹（无需键盘）
bash run_demo.sh

# 测试
pytest                                   # 主包测试
pytest portable_oscbf/tests              # 控制核心测试
colcon test --packages-select aeb_rrtstar_ompl 2>/dev/null || \
  (cd src/aeb_rrtstar_ompl && colcon build --base-paths . && colcon test --base-paths .)

# 独立脚本（无 ROS）
python3 src/aeb_rrtstar/single_run.py    # 查看 aeb_rrtstar 用法
```

注意：修改 AEB C++ 代码后必须重新 `bash build_aeb_moveit.sh` 并重启整个
launch——运行中的 move_group 不会自动加载新编译的 `.so` 插件。

## Key Entry Points

- `src/robot_safecontrol_moveit/oscbf_controller.py` — OSCBF 安全控制节点（M10）
- `src/robot_safecontrol_moveit/oscbf_plant.py` — jerk 限幅执行器仿真节点
- `src/robot_safecontrol_moveit/transition_planning_server.py` — 过渡规划服务器（薄 ROS 壳）
- `src/robot_safecontrol_moveit/transition_executor.py` — 过渡管线相位机（纯逻辑，无 ROS）
- `src/robot_safecontrol_moveit/mujoco_viewer_with_cylinder.py` — MuJoCo 查看器/仿真
- `src/robot_safecontrol_moveit/oscbf_trajectory.py` — 统一轨迹变换（三端共享）
- `portable_oscbf/work/jax_control_facade.py` — JAX 控制内核主机端入口
- `src/aeb_rrtstar_ompl/src/AEBRRTstar.cpp` — C++ AEB-RRT* 插件实现
- `launch/mujoco_transition_final.launch.py` — 完整闭环 launch
- `run_demo.sh` / `build_aeb_moveit.sh` — 一键运行/构建

## Key Documents

- `README.md` — 项目概览、运行方式、关节配置表
- `OSCBF_PORTING_GUIDE.md` — OSCBF 从零移植指南（架构 + 步骤 + 验收门）
- `OSCBF_EXECUTION_PLAN.md` — 执行计划（M6-M12 里程碑）
- `LESSONS_LEARNED.md` — 踩坑经验（现象→根因→修复→教训）
- `docs/planning/aeb_rrtstar/benchmark_report.md` — AEB-RRT* 基准报告

## Skills to Load

无官方栈技能匹配（非 Web/移动项目）；适合 `codebase-design`、
`code-review`、`tdd` 等通用技能。
