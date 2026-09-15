# robot_safecontrol — Onboarding Guide
**Updated:** 2026-09-11  **Stack:** ROS 2 Humble / MoveIt 2 / MuJoCo / JAX / C++ OMPL

## Overview

9-DOF 冗余机械臂（1 棱柱关节 J1 + 8 旋转关节 J2-J9）的安全控制项目。
当前仿真演示从随机位姿经 AEB-RRT* 过渡到蝴蝶轨迹起点，再由 OSCBF 控制器跟踪。
真机执行端处于 fail-closed containment：sim inert，shadow 仅记录且无 CAN I/O，
live 无条件拒绝。仿真验证不代表真实硬件能力或实机验收完成。

## Tech Stack

| Layer | Technology | Version |
|-------|-----------|---------|
| ROS 2 | Humble (rclpy / rclcpp) | Humble |
| Python 包 | robot_safecontrol_moveit (ament_python), setup.py | 0.1.0 |
| 运动规划 | MoveIt 2 + pymoveit2 + OMPL 插件 | — |
| 物理仿真 | MuJoCo | — |
| 控制内核 | JAX 0.6.2 (JIT), qpax 弹性 QP | — |
| 碰撞 | FCL, OBB 包络, DCOL, 17-球模型 | — |
| C++ 插件 | aeb_rrtstar_ompl (ament_cmake) | 0.1.0 |
| 真机通信 | python-can / SocketCAN, DrEmpower 协议 | — |
| 数值 | numpy, scipy, yaml | — |

## Architecture

**ROS 2 工作区 + 嵌套包**：根目录是 `robot_safecontrol_moveit`（ament_python），
其 `src/aeb_rrtstar_ompl` 是嵌套的 ament_cmake 包（MoveIt2 OMPL 插件），普通
`colcon build` 发现不了它，必须用 `build_aeb_moveit.sh` 指定 `--base-paths`。

**仿真闭环（`run_demo.sh` 配置）**：脚本启用 `oscbf_plant`，它订阅
`/oscbf_command`，通过带加速度/jerk 限幅的执行器仿真积分并持续发布
`/mujoco_joint_states`。`oscbf_controller` 订阅该状态流，运行 JAX 内核后
发布命令。viewer 只订阅状态做 MuJoCo 显示，不发布关节状态。
过渡服务器经 `trajectory_execution.py` 将回放发送到 `/transition_replay_viz`，
并向命令流发送过渡命令；回放及收敛完成后交接给控制器。

**默认 launch 与 demo 的差别**：直接启动最终 launch 时 `start_oscbf_plant=false`，
`transition_replay_topic=/mujoco_joint_states`；过渡回放会向此状态话题发布，
但没有被控对象持续积分形成上述闭环。不能把 demo 的状态 ownership 套到所有配置。
当前 containment 的 hardware bridge 在 sim/shadow 都不发布硬件状态。

**自主过渡流程**：`transition_planning_server` 监听开始信号后，调用 move_group
的 AEB-RRT* 规划器从随机位姿规划无碰撞过渡到轨迹起点，经 Ruckig 平滑后回放；
回放结束 → OSCBF 控制器接管（`oscbf_wait_for_start`）。

**真机目标与当前实现**：未来目标是 command → 安全网关 → CAN → 真实反馈 →
硬件状态流。当前 `hardware_bridge.py` 未接通该链路：sim 不创建控制订阅、
polling timer 或状态 publisher；shadow 订阅命令并记录请求/拒绝，不创建 CAN
backend、不收发 CAN；live 在创建硬件控制实体之前无条件失败。
无真实反馈时 `feedback_ok`、`watchdog_ok` 均为 false，acknowledge 不能制造健康状态。
final launch 在启动任何节点前拒绝 `hardware_mode=live`；`hardware_bridge` 自身也保留独立拒绝，保护 direct node 入口。未来解除 live containment 时，两层必须独立审查。
剩余实现及真机准入见 [GitHub #13](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13)。

**感知管线**：`perception_bridge` 接收点云 → `obstacle_extractor` 聚类拟合球/
圆柱几何 → 发布到 `/perception/tracks` → `oscbf_controller` 经 `obs_*` 接口注入
控制内核做动态避障。

**标定记录（外参唯一真源）**：感知外参只来自 `config/sensor_extrinsics.yaml`
（经 ament share 路径加载；逐源分节含矩阵与 provenance）。`perception_bridge` 启动时校验
「记录可读 / 矩阵是合法 SE(3) / 帧绑定与配置一致 / 每个被启用源过标定门」，任一条不满足即
**拒绝启动**（退出码 3，不发布任何诊断）；运行期每帧校验 `header.frame_id`，不符即丢弃。
旧参数 `use_tf` / `camera_to_world_static` / `lidar_to_world_static` 只保留声明作探测器
（非默认值即拒绝启动），**没有 TF 输入路径，也没有 identity 兜底**。身份与准入结论发布在
`/perception/calibration_status`（`DiagnosticArray`，1 Hz 心跳 + 启动即发一份）。
未标定几何只允许在部署 profile 显式逐源声明 `allow_uncalibrated_debug` 时启动，
且不构成准入。契约见 ADR 0004 与 ADR 0009。

**坐标系**：URDF 为 Y-up，MuJoCo 为 Z-up，程序通过 `display_frame` body 的
euler 旋转自动转换。圆柱轴心拟合口径在轨迹生成端、过渡端、控制器端必须一致
（默认最小二乘圆拟合轨迹自动求轴心）。

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

## Request Lifecycle（闭环数据流）

1. `run_demo.sh` 启用被控对象并设置随机起始位姿；`oscbf_plant.py` 持续发布仿真状态。
2. `transition_planning_server.py` 调用 MoveIt/AEB-RRT* 与 FCL 规划过渡，经 Ruckig
   平滑并回放命令；回放后等待被控对象收敛，再交接给 OSCBF 控制器。
3. `oscbf_controller.py` 订阅状态，经 `portable_oscbf/work` 的 JAX facade 计算并
   发布 `/oscbf_command`；被控对象积分后继续发布 `/mujoco_joint_states`。
4. `mujoco_viewer_with_cylinder.py` 订阅状态驱动显示；它不拥有控制状态 publisher。
5. `tracking_evaluator.py` 可订阅状态与命令计算跟踪指标；查看器、过渡服务器和
   控制器共用 `oscbf_trajectory.py` 的轨迹变换。

**当前 QoS**：canonical [ros_conventions.py](../src/robot_safecontrol_moveit/ros_conventions.py)
的 `state_stream_qos()` 是 KEEP_LAST / depth 20 / BEST_EFFORT / VOLATILE。
被控对象的状态发布和命令订阅、控制器的状态订阅及 shadow bridge 的命令订阅使用它。
viewer、过渡服务器的状态订阅、过渡回放发布和控制器的命令发布仍使用
`qos_profile_sensor_data`（depth 5 / BEST_EFFORT / VOLATILE）。深度差异仍存在，
这里记录当前实体，不代表全面 QoS 已统一；不存在 viewer 发布 transient-local 状态的链路。

## Conventions Detected

- **命名**：模块小写下划线命名（`oscbf_controller.py`、`transition_planning_server.py`）；
  类用 PascalCase（`JaxControlLoop`、`SCurveDriverSimulator`）；测试 `test_*.py`。
- **节点风格**：每个节点一个模块，`main()` 入口注册为 console_scripts；
  模块级 docstring 说明职责与话题约定。
- **共享约定**：话题名/QoS/关节名等统一在 `ros_conventions.py` 与 `robot_spec.py`，
  节点间禁止互相 import 私有符号。
- **轨迹加载与拟合**：`oscbf_trajectory.py` 的两个公开加载入口共享全量标定、
  圆柱投影和抽样流程；仅位置入口不要求时间字段。`CylinderFit.axis_point`
  统一拟合轴心坐标解释，查看器自行决定地面延伸等显示范围。
- **控制内核隔离**：`portable_oscbf/work/` 零 ROS 依赖、零裸名同级 import
  （统一 `from work.X`），节点经 `oscbf_trajectory.bootstrap_portable` 引导。
- **错误处理**：launch 文件对必需配置文件做启动时 `FileNotFoundError` 校验；
  内核侧有 QP 健康检查（`qp_solver_health.py`、`safety_snapshot.py`）；
  硬件合同网关可生成拒绝/保持结果并锁存原因；当前 bridge 无 CAN 发送能力，不能据此推断物理停车。
- **测试**：pytest（`testpaths = tests`），主包含 launch 集成测试
  （launch_testing）；`portable_oscbf/tests` 有独立 conftest；
  C++ 包有自测可执行文件（test_aeb_full.cpp 等）。全量入口：`bash run_all_tests.sh`。
- **Git**：单 main 分支；提交信息中英混合、多为 feat:/fix:/perf: 前缀或中文摘要。

## Common Tasks

```bash
# 构建（必须用脚本，普通 colcon build 发现不了嵌套 C++ 插件）
bash build_aeb_moveit.sh
source install/setup.bash

# 全自动演示：随机起始位姿 → 无碰撞过渡 → OSCBF 跟踪蝴蝶轨迹（无需键盘）
bash run_demo.sh

# shadow 记录模式：无 CAN I/O，不提供真实硬件反馈
ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py \
    hardware_mode:=shadow start_oscbf_plant:=false

# 测试：结果以当前 checkout 实际执行为准
bash scripts/agent_check.sh              # 快速启发式检查
bash run_all_tests.sh                    # 完整主包 + 内核回归

# 独立脚本（无 ROS）
python3 src/aeb_rrtstar/single_run.py    # 查看 aeb_rrtstar 用法
```

注意：修改 AEB C++ 代码后必须重新 `bash build_aeb_moveit.sh` 并重启整个
launch——运行中的 move_group 不会自动加载新编译的 `.so` 插件。

## Key Entry Points

- `src/robot_safecontrol_moveit/oscbf_controller.py` — OSCBF 安全控制节点
- `src/robot_safecontrol_moveit/oscbf_plant.py` — jerk 限幅执行器仿真节点
- `src/robot_safecontrol_moveit/transition_planning_server.py` — 过渡规划服务器（薄 ROS 壳）
- `src/robot_safecontrol_moveit/transition_executor.py` — 过渡管线相位机（纯逻辑，无 ROS）
- `src/robot_safecontrol_moveit/mujoco_viewer_with_cylinder.py` — MuJoCo 查看器/仿真
- `src/robot_safecontrol_moveit/oscbf_trajectory.py` — 统一轨迹变换（三端共享）
- `src/robot_safecontrol_moveit/hardware_bridge.py` — containment 入口（sim inert / shadow 记录 / live disabled）
- `src/robot_safecontrol_moveit/perception_bridge.py` — 感知桥接节点
- `src/robot_safecontrol_moveit/obstacle_extractor.py` — 点云→几何障碍物提取
- `src/robot_safecontrol_moveit/tracking_evaluator.py` — 跟踪评价指标
- `src/robot_safecontrol_moveit/drempower_can.py` — DrEmpower CAN 协议编解码
- `src/robot_safecontrol_moveit/socketcan_backend.py` — SocketCAN 抽象与替身测试入口，当前 bridge 不接入
- `portable_oscbf/work/jax_control_facade.py` — JAX 控制内核主机端入口
- `src/aeb_rrtstar_ompl/src/AEBRRTstar.cpp` — C++ AEB-RRT* 插件实现
- `launch/mujoco_transition_final.launch.py` — 完整闭环 launch
- `run_demo.sh` / `build_aeb_moveit.sh` — 一键运行/构建

## Key Documents

- `README.md` — 项目概览、运行方式、关节配置表、真机模式
- `CONTEXT.md` — 领域词汇表（系统结构、轨迹几何、真机部署术语）
- `LESSONS_LEARNED.md` — 踩坑经验（现象→根因→修复→教训）
- `OSCBF_PORTING_GUIDE.md` — OSCBF 从零移植指南（架构 + 步骤 + 验收门）
- `OSCBF_EXECUTION_PLAN.md` — 执行计划（M6-M12 里程碑）
- `docs/real_robot_runbook.md` — 真机操作手册
- `docs/real_robot_execution_plan.md` — 真机落地执行计划
- `docs/specs/real_robot_landing_spec.md` — 真机落地规格
- `docs/adr/0001-server-authoritative-transition-pipeline.md` — ADR：过渡管线架构决策
- `docs/planning/aeb_rrtstar/benchmark_report.md` — AEB-RRT* 基准报告

## Skills to Load

无官方栈技能匹配（非 Web/移动项目）；适合 `codebase-design`、
`code-review`、`tdd` 等通用技能。
