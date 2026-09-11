# robot_safecontrol

9 自由度（1P8R）机械臂安全控制项目，基于 MuJoCo、ROS 2 Humble、MoveIt2 与 JAX OSCBF 控制内核。任务采用 5D 工具轴路径跟随：控制末端位置和工具轴方向，绕工具轴的 roll 自由。

## 项目状态与文档入口

当前仓库包含仿真闭环、感知与硬件接口、独立 MID-360S 驱动，以及开源复用和渐进迁移的研究交接。研究结论、代码实现和实机验收分别记录；具体状态以各票交接及 [GitHub 地图](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/1)为准。

- [项目入门与架构](docs/ONBOARDING.md)、[领域术语](CONTEXT.md)
- [复用研究与决策索引](docs/planning/oscbf-reuse/README.md)、[逐票推进顺序](docs/planning/oscbf-reuse/DECISION-WINDOW-ORDER.md)
- [OSCBF 移植指南](OSCBF_PORTING_GUIDE.md)、[真机运行手册](docs/real_robot_runbook.md)
- [独立 MID-360 / MID-360S 驱动](xy-mid-360-s/README.md)：安装、设备接口、来源许可及已验证范围

测试入口与 roll-only 测试修订已实施；当前验证方法与历史记录见下方“测试与已知问题”。

## 目录结构

```
robot_safecontrol/
├── src/
│   ├── aeb_rrtstar_ompl/               # C++ AEB-RRT* MoveIt 插件
│   └── robot_safecontrol_moveit/       # ROS2 Python 包
│       ├── transition_planning_server.py #   持久化规划服务器（薄 ROS 壳）
│       ├── transition_executor.py      #   过渡管线相位机（纯逻辑，无 ROS）
│       ├── continuous_ik.py            #   连续 IK 求解
│       ├── motion_planning.py          #   MoveIt 运动规划
│       ├── cylinder_geometry.py        #   圆柱拟合/表面法向（单一实现）
│       ├── robot_spec.py               #   共享机器人常量（关节名等）
│       ├── mujoco_viewer_with_cylinder.py  # MuJoCo 可视化节点
│       └── trajectory_execution.py     #   轨迹执行
├── portable_oscbf/                    # JAX 控制内核与独立测试
├── xy-mid-360-s/                      # 独立 Python 雷达驱动
├── docs/                              # 架构、运行手册、研究与实施交接
├── models/
│   ├── ninezzhou/                      # 9 轴机械臂 URDF 包
│   │   ├── urdf/                       #   URDF 定义
│   │   ├── meshes/                     #   STL 网格 (Link1~9 + base_link)
│   │   └── config/                     #   关节名称
│   └── ninezzhou_moveit_config/        # MoveIt2 配置包
│       ├── config/                     #   SRDF、控制器、运动学配置
│       └── launch/                     #   demo.launch.py
├── config/
│   └── mujoco_transition_runtime.yaml  # 过渡服务器运行时参数
├── launch/
│   ├── mujoco_transition_final.launch.py  # 完整闭环 launch
│   └── mujoco_viewer.launch.py         # ROS2 launch: MuJoCo 可视化
├── data/
│   └── nurbs/                          # NURBS 轨迹数据
│       └── ik_input.mat                #   逆运动学输入 (末端轨迹)
├── output/                             # 生成文件 (已 gitignore)
│   ├── ninezzhou_env.xml               #   MuJoCo 环境文件
│   └── transition_path.npy             #   过渡路径
├── package.xml                         # ROS2 包清单
├── setup.py / setup.cfg                # Python 包配置
├── .gitignore
└── README.md
```

## 运行

### ROS2 节点

```bash
# 编译 Python 闭环包和 C++ AEB-RRT* MoveIt 插件。
# 注意：这里不能只运行普通的 `colcon build`，因为插件位于嵌套包。
bash build_aeb_moveit.sh
source install/setup.bash

# 启动全自动闭环：机械臂每次从随机工作位姿出发，自动规划无碰撞过渡到
# 轨迹起点，回放结束后 OSCBF 控制器接管 /oscbf_command 并跟踪蝴蝶轨迹到
# 终点。全程无需键盘。
bash run_demo.sh
```

`config/oscbf_controller.yaml` 是 OSCBF 控制器的必需生产配置。launch 将其
作为独立的 `production_config_yaml` 来源加载；文件缺失、字段不全、值非法、
资源不可读或启动快照无法落盘时，控制器会在建立命令 publisher 前退出。直接
启动节点使用同一契约，例如：

```bash
ros2 run robot_safecontrol_moveit oscbf_controller --ros-args \
  -p production_config_yaml:=$(ros2 pkg prefix robot_safecontrol_moveit)/share/robot_safecontrol_moveit/config/oscbf_controller.yaml
```

显式 `-p` 覆盖优先于已验证的 YAML 基础值；所有生产参数在启动后不可修改。
每次成功启动的最终值、来源链、资源路径、配置/软件哈希及拟合几何会原子写入
性能报告同目录下的 `runtime_snapshots/`。

依赖包含 ROS 2 Humble、MoveIt2、pymoveit2、MuJoCo，以及 JAX/CBFpy/qpax 等控制内核依赖。环境与构建说明见 [项目入门](docs/ONBOARDING.md)和 [控制内核 README](portable_oscbf/README.md)。

最终闭环默认使用 `AEBRRTstarFaithfulConfigDefault`。AEB-RRT* 由 MoveIt
PlanningScene/FCL 做状态与路径碰撞检查。

## 测试与已知问题

`run_all_tests.sh` 已修复 ROS setup 与 `set -u` 的兼容问题；两个套件分别执行并汇总退出码，首套失败仍会运行第二套，只有两套均通过才返回成功：

```bash
bash run_all_tests.sh
```

已实施的测试工具修订（链接保留各自历史验收记录）：

| 项目 | 结论与状态 |
|---|---|
| [测试入口退出](docs/planning/oscbf-reuse/handoffs/10-test-entrypoint.md) | 已实施局部关闭 nounset 与双套件退出码汇总；验收记录见链接。 |
| [roll-only 路径起点测试](docs/planning/oscbf-reuse/handoffs/11-roll-only-tolerance.md) | 已修订精度、纯 roll / 倾斜对照与执行后报告断言；修订已归档，历史验收环境见链接。 |

最近一次 containment 产品代码完整验收基于 `e4d9a268`，对应证据见[验证归档](docs/planning/oscbf-reuse/validation/2026-09-11-containment/REPORT.md)。当前 checkout 的测试真值仍以实际运行 `bash scripts/agent_check.sh`（快速反馈）与 `bash run_all_tests.sh`（完整回归）为准；验证记录应注明 commit、日期、环境和退出码。[可复现基线](docs/planning/oscbf-reuse/handoffs/30-baseline.md)仅保留历史结果，不表示当前仍有同样失败。

最小纯软件 CI 入口：[Pure software checks](.github/workflows/pure-checks.yml)。

## 硬件模式（当前 containment）

当前处于 fail-closed containment：

- `sim`：hardware bridge 不创建真实硬件控制 I/O。
- `shadow`：仅记录命令请求与安全拒绝，不收发 CAN、不发布真实硬件状态。
- `live`：final launch 在启动任何 Node/TimerAction 前拒绝 `hardware_mode=live`；`hardware_bridge` 自身也保留独立拒绝，用于保护直接启动节点的入口。

`hardware_mode` 仅接受 `sim` / `shadow` / `live`；非法值与 `live` 都会在 final launch 启动节点前失败，其中 `live` 是 containment 禁用，`sim`/`shadow` 为允许模式。

真实 backend、配置、标定、真实 feedback freshness/watchdog 和 real-hardware acceptance 完成前，live 仍不可用。反馈不可用时不会报告 healthy，人工确认不能解除这一条件。禁止发送不等于物理制动，物理急停仍是独立链路。以下 live 命令仅用于说明拒绝行为；后续准入见[真机运行手册](docs/real_robot_runbook.md)。

```bash
# shadow 模式：记录请求及拒绝结果；真实反馈 unavailable；不收发 CAN
ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py \
    hardware_mode:=shadow start_oscbf_plant:=false

# live 参数形式：final launch 在启动任何节点前报错退出
ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py \
    hardware_mode:=live start_oscbf_plant:=false

# 即使启用感知，final launch 仍在启动任何节点前拒绝 live
ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py \
    hardware_mode:=live start_oscbf_plant:=false start_perception:=true
```

标定与实机操作属于后续独立准入流程，参见[真机运行手册](docs/real_robot_runbook.md)，不是当前 containment 的运行步骤。

话题约定：OSCBF 控制器订阅植物状态 `/mujoco_joint_states`，把安全命令发布到
`/oscbf_command`；`oscbf_plant` 节点把命令经 jerk 限幅积分后作为植物状态发回
`/mujoco_joint_states`。查看器、过渡服务器、控制器共用同一套校准轨迹变换
（`oscbf_trajectory.py`），保证 tool0 与显示的蝴蝶曲线重合。

每次修改 AEB 的 C++ 代码后，都要重新执行 `bash build_aeb_moveit.sh`，再重启
整个 launch（直接运行 `bash run_demo.sh` 即可）。运行中的 `move_group` 不会自动
加载新编译的 `.so` 插件。

## 坐标系

- 规范世界坐标系为固定的 `base_link`，使用 Y-up；J1 沿该系 Z 轴移动，基座坐标系不随 J1 移动。
- MuJoCo 显示使用 Z-up，通过 `display_frame` body 的 euler 旋转转换。
- 坐标系决议见 [base_link 规范世界坐标系](docs/adr/0002-base-link-canonical-world-frame.md)。

## 关节配置

| 关节 | 类型 | 范围 | 说明 |
|------|------|------|------|
| J1 | prismatic | [0, 0.585] m | 沿 base_link Z 轴的水平直线滑台 |
| J2-J4 | revolute | [-π/2, π/2] | 肩/肘 |
| J5 | revolute | [-π, π] | 腕旋转 |
| J6-J9 | revolute | [-1.48, 1.48] | 末端 |

## 传感器安装方案

深度相机与雷达共用支架的[布局与标定方案](docs/sensor_layout_and_calibration.md)及 [ADR 0007](docs/adr/0007-shared-rear-sensor-stand.md)已纳入仓库，包含安装坐标、MuJoCo 图示、几何验证结果和实机待办；文档存在不代表实机标定完成。驱动核验见[双传感器驱动交接](docs/planning/oscbf-reuse/handoffs/31-official-sensor-drivers.md)。
