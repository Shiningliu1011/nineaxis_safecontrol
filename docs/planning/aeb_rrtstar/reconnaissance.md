# 阶段 A：仓库侦察报告

## A.1 项目概览

| 属性 | 值 |
|------|-----|
| 语言 | Python 3.10 |
| 构建系统 | colcon (ROS 2) + setuptools |
| 包管理 | apt (ROS 2 Humble), pip (Python deps) |
| CI | 未检测到 CI 配置 |
| 许可证 | BSD-3-Clause |
| Git commit (基线) | `fc36d3f` |
| ROS 2 版本 | Humble |
| MoveIt 2 版本 | 2.5.9-1jammy |

## A.2 OMPL 版本与环境

系统存在 **三套** OMPL：

| 来源 | 版本 | 路径 |
|------|------|------|
| 系统 apt (libompl-dev) | 1.5.2 | `/usr/lib/x86_64-linux-gnu/libompl.so.1.5.2` |
| ROS 2 Humble (ros-humble-ompl) | 1.7.0 | `/opt/ros/humble/lib/x86_64-linux-gnu/libompl.so.1.7.0` |
| Python pip (ompl) | 2.0.1 | `~/.local/lib/python3.10/site-packages/ompl/libompl.so` |

MoveIt 2 的 `move_group` 节点内部使用 OMPL 1.7.0（ROS 2 Humble 版本）。
Python `ompl` 包（2.0.1）可在独立脚本中使用，与系统库链接。

## A.3 规划调用链

### ROS 2 路径（生产路径）

```
TransitionPipelineNode (plan_transition.py)
  └─ MotionPlanner.plan_transition()
       └─ pymoveit2.MoveIt2.plan()           # ROS 2 action client
            └─ /plan_kinematic_path (MoveIt 2 move_group)
                 └─ ompl_interface/OMPLPlanner
                      └─ geometric::RRTConnect
```

### 独立脚本路径（无 ROS）

```
src/plan_transition.py → 直接委托到 robot_safecontrol_moveit.plan_transition:main
（同样依赖 ROS 2 / MoveIt 2 运行时）
```

**关键发现：项目中不存在直接调用 OMPL API 的代码。** 所有规划都通过 MoveIt 2 的 `pymoveit2` 客户端转发。

## A.4 当前规划器配置

**文件**: `models/ninezzhou_moveit_config/config/ompl_planning.yaml`

```yaml
planning_plugin: ompl_interface/OMPLPlanner
planner_configs:
  RRTConnectkConfigDefault:
    type: geometric::RRTConnect
    range: 0.0          # 使用 OMPL 自动计算的范围
```

**文件**: `config/plan_transition.yaml`

```yaml
planning_pipeline: ompl
planner_id: RRTConnectkConfigDefault
planning_time_s: 10.0
planning_attempts: 5
velocity_scale: 0.2
acceleration_scale: 0.2
```

**生效参数** (MotionPlanner 中设置):
- `pipeline_id`: "ompl"
- `planner_id`: "RRTConnectkConfigDefault"
- `planning_time_s`: 10.0
- `planning_attempts`: 5
- `velocity_scale`: 0.2
- `acceleration_scale`: 0.2
- `goal_joint_tolerance`: 0.001

MoveIt 的 request_adapters 包括：
- `AddTimeOptimalParameterization`
- `ResolveConstraintFrames`
- `FixWorkspaceBounds`
- `FixStartStateBounds`
- `FixStartStateCollision`
- `FixStartStatePathConstraints`

## A.5 状态空间

**9 维 RealVectorStateSpace**（关节空间）：

| 关节 | 类型 | 下界 | 上界 | 单位 |
|------|------|------|------|------|
| J1 | prismatic | 0.0 | 0.585 | m |
| J2 | revolute | -π/2 | +π/2 | rad |
| J3 | revolute | -π/2 | +π/2 | rad |
| J4 | revolute | -π/2 | +π/2 | rad |
| J5 | revolute | -π | +π | rad |
| J6 | revolute | -1.48353 | +1.48353 | rad |
| J7 | revolute | -1.48353 | +1.48353 | rad |
| J8 | revolute | -1.48353 | +1.48353 | rad |
| J9 | revolute | -1.48353 | +1.48353 | rad |

**注意**：J5 是完整圆周关节（±π），需要考虑周期性。其余旋转关节范围均 < 2π，无需周期性处理。

## A.6 碰撞检测与运动验证

当前项目的碰撞检测完全在 MoveIt 2 内部进行：
- **PlanningScene** 维护机器人几何体（STL 网格）和障碍物（box/sphere/cylinder 基元）
- **StateValidityChecker**：MoveIt 使用 FCL（Flexible Collision Library）进行网格碰撞检测
- **MotionValidator**：MoveIt 对关节空间路径进行离散插值并逐点检查

障碍物（`config/obstacles.yaml`）：
1. `obs_box1`: box, pos=[0.25, 0.243, 0.4], dim=[0.08, 0.08, 0.16]
2. `obs_sphere1`: sphere, pos=[-0.25, 0.343, 0.6], dim=[0.05]
3. `obs_cyl1`: cylinder, pos=[0.22, 0.30, 0.9], dim=[0.16, 0.03]
4. `obs_box2`: box, pos=[-0.1, 0.15, 0.9], dim=[0.10, 0.10, 0.10]

## A.7 终止条件

MoveIt 2 中的 OMPL 规划使用以下终止条件：
- `planning_time_s`: 10.0 秒（通过 `PlannerTerminationCondition` 的时间限制）
- `planning_attempts`: 5 次尝试（MoveIt 层的重试逻辑）
- RRTConnect 在找到首条路径后立即返回（首解停止模式）

## A.8 路径后处理

MoveIt 2 的 `default_planner_request_adapters` 包括：
1. **AddTimeOptimalParameterization**：时间最优参数化（速度/加速度约束）
2. **FixStartStateBounds/Collision**：修正起始状态
3. **ResolveConstraintFrames**：解析约束框架

路径简化/平滑：由 MoveIt 的 `PathSimplifier` 在内部进行。

## A.9 代表性场景

项目包含一个主要规划场景：
- **过渡路径规划**：从零位（全零 joint positions）到轨迹起始点（IK 求解的第一个 waypoint）
- 4 个静态障碍物
- 9-DOF 关节空间

IK 轨迹来自 `data/nurbs/ik_input.mat`，包含笛卡尔末端轨迹（圆柱面）。

## A.10 现有测试/基准

**无。** 项目中不存在任何自动化测试、benchmark 或性能回归测试基础设施。

## A.11 AEB-RRT* 接入点分析

### 推荐方案：独立 Python OMPL Benchmark + 后续 C++ 插件

由于项目当前无直接 OMPL API 调用，AEB-RRT* 接入有两个层级：

#### 层级 1：Python 独立 Benchmark（本次实现）
- 使用 `ompl` Python 绑定（v2.0.1）
- 继承 `ompl.base.Planner` 实现 AEB-RRT*
- 创建独立的 benchmark 框架，与项目现有 ROS 2 路径并行
- 复用相同的状态空间、关节限制和障碍物配置
- 碰撞检测使用简化模型（关节范围 + 基本几何近似）
- **优点**：快速验证、不触及生产代码、可回退

#### 层级 2：C++ OMPL 插件 + MoveIt 2 集成（后续工作）
- 用 C++ 实现 `ompl::base::Planner` 子类
- 注册为 `ompl_interface` 插件
- 在 MoveIt 2 的 `ompl_planning.yaml` 中配置
- **优点**：使用 MoveIt 原生碰撞检测、完整生产集成

## A.12 预计改动文件清单

### 新增文件
1. `src/aeb_rrtstar/__init__.py`
2. `src/aeb_rrtstar/aeb_rrtstar_planner.py` — AEB-RRT* `ompl.base.Planner` 子类
3. `src/aeb_rrtstar/collision_checker.py` — 简化碰撞检测
4. `src/aeb_rrtstar/benchmark_runner.py` — 基准运行器
5. `src/aeb_rrtstar/scenarios.py` — 测试场景定义
6. `src/aeb_rrtstar/robot_model.py` — 机器人运动学模型
7. `benchmarks/aeb_rrtstar/` — 基准输出目录
8. `docs/planning/aeb_rrtstar/reconnaissance.md` — 本报告
9. `docs/planning/aeb_rrtstar/status.md` — 状态追踪
10. `docs/planning/aeb_rrtstar/benchmark_report.md` — 基准报告

### 不修改的文件
- 所有现有 ROS 2 节点、配置和启动文件
- MoveIt 2 配置
- URDF 模型

## A.13 风险与限制

1. **碰撞检测精度**：Python benchmark 使用简化几何模型，无法完全复现 MoveIt 2 + FCL 的碰撞检测精度。这是最主要的限制。
2. **Python 性能**：Python 实现的规划器速度可能低于 C++ 原生 OMPL 规划器，但相对比较仍然有效。
3. **无 MoveIt 集成**：Python benchmark 的结果不能直接推广到 MoveIt 2 生产环境。
4. **自碰撞**：简化模型难以精确检测自碰撞，建议在关键场景中验证。
5. **J5 周期性**：完整圆周关节需要特殊距离处理。
