# portable_oscbf — JAX OSCBF 可移植控制器

9-DOF 冗余机械臂（1 棱柱关节 J1 + 8 旋转关节 J2-J9）的 **JAX OSCBF 轨迹跟踪控制器**独立包。

基于 Morton & Pavone, *Safe, Task-Consistent Manipulation with OSCBF* (IROS 2025)。

## 目录结构

```
portable_oscbf/
├── work/                          # 核心计算模块 (Python package)
│   ├── __init__.py
│   │
│   │  ── JAX 编译内核层 (热路径) ──
│   ├── jax_kernel_factory.py      # OSC+CBF+QP+积分 编译入口
│   ├── jax_barrier_terms.py       # 固定 shape 障碍物几何/CBF RHS
│   ├── jax_path_following.py      # JAX 弧长路径跟踪状态机
│   ├── jax_posture_reference.py   # JAX 姿态参考插值
│   ├── jax_control_facade.py      # 主机端 facade, 输入归一化, JIT warmup
│   ├── jax_control_loop.py        # 兼容性 re-export
│   │
│   │  ── JAX 运动学层 ──
│   ├── nineaxis_manipulator_jax.py # 9-DOF POE FK/Jacobian (JAX)
│   ├── collision_envelope.py       # 17-球碰撞模型数据
│   │
│   │  ── JAX QP 求解器层 ──
│   ├── qpax_solver.py             # qpax 弹性 QP 求解器
│   ├── qpax_warmstart.py          # PDIP warm-start 适配器
│   ├── qp_solver_health.py        # QP 健康检查
│   │
│   │  ── OSCBF 配置层 (cbfpy 框架) ──
│   ├── oscbf_velocity_config.py   # 速度级 OSCBF 配置
│   ├── oscbf_torque_config.py     # 力矩级 OSCBF 配置
│   ├── oscbf_collision_config.py  # 碰撞约束配置
│   ├── tool_axis_task.py          # 5D/6D 任务误差/Jacobian
│   ├── task_mode_contract.py      # 任务模式常量/验证
│   ├── safety_snapshot.py         # 固定 shape 距离场快照
│   │
│   │  ── NumPy 运动学 + FCL 碰撞 ──
│   ├── nineaxis_kinematics.py     # POE FK/Jacobian/IK (NumPy+SciPy)
│   ├── fcl_collision.py           # FCL 基元自碰撞检测
│   ├── fcl_collision_mesh.py      # FCL BVHModel 网格碰撞
│   ├── point_cloud_obstacles.py   # FCL 点云环境碰撞
│   ├── point_cloud_obstacles_dynamic.py # 动态障碍物类型
│   ├── dynamic_obstacles.py       # DynamicObstacleManager
│   ├── dynamic_obstacle_placement.py    # FCL 辅助障碍物放置
│   ├── controller_step_cache.py   # 每步 FK 缓存
│   ├── cbf_types.py               # CbfConstraint 数据类
│   ├── oscbf_qp_solver.py         # OSQP QP 求解器 (legacy)
│   ├── collision_geometry_backends.py # 碰撞后端接口
│   ├── joint_limit_contract.py    # 关节限位 CBF 边距
│   ├── safe_posture.py            # 离线安全姿态验证
│   ├── online_ik_des.py           # 在线 IK q_des 计算
│   │
│   │  ── 路径/轨迹 ──
│   ├── ik_data_loader.py          # .mat 轨迹加载, Frenet-Serret 帧
│   ├── path_following.py          # 弧长路径几何/状态机 (NumPy)
│   ├── path_posture_reference.py  # 姿态参考 (NumPy)
│   ├── fixed_path_trajectory.py   # 固定路径轨迹适配器
│   ├── traj_to_base_transform.py  # 轨迹→基座坐标变换
│   ├── driver_simulator.py        # S 曲线驱动模拟器
│   │
│   │  ── 工具 ──
│   ├── actuator_limits.py         # YAML 执行器限位
│   ├── perf_metrics.py            # 性能指标
│   ├── metric_row_spool.py        # 磁盘指标缓存
│   ├── joint_trajectory_logging.py # 关节时序记录
│   └── joint_velocity_limits.py   # 速度/加速度边界
│
├── config/                        # 机器人配置 (YAML)
│   ├── nineaxis.yaml              # 机器人参数
│   ├── actuator_modules.yaml      # 执行器限位
│   ├── controller_params.yaml     # 控制器增益
│   ├── fcl_params.yaml            # FCL 碰撞参数
│   ├── obstacle_params.yaml       # 障碍物参数
│   ├── ompl_params.yaml           # OMPL 规划参数
│   └── robot_params.yaml          # 机器人参数
│
├── data/                          # 轨迹数据 (.mat)
│   ├── ik_input.mat               # 主轨迹数据
│   ├── workspace_ik_input.mat     # 工作空间轨迹
│   ├── nurbs_blocks.mat           # NURBS 块数据
│   ├── workspace_nurbs_blocks.mat # 工作空间 NURBS 块
│   ├── realtime_interpolation_results.mat
│   └── workspace_realtime_results.mat
│
├── urdf/                          # 机器人模型
│   ├── ninezzhou.urdf             # URDF 描述文件
│   └── meshes/                    # STL 网格文件 (10 个)
│
├── tests/                         # 单元测试 (25 个)
│   ├── test_jax_tracking_step.py
│   ├── test_jax_path_following.py
│   ├── test_jax_qp_health.py
│   ├── test_cbfpy_migration.py
│   ├── test_safety_snapshot.py
│   └── ...
│
└── README.md                      # 本文件
```

## 控制管线架构

```
轨迹参考 (弧长参数化)
  │
  ▼
6-DOF 速度级 OSC (P-only 名义控制, kp=50)
  │  P = N^T @ W_joint² @ N + J^T @ W_task² @ J
  │  W_task >> W_joint → QP 自动在零空间修正
  ▼
CBF-QP 安全滤波器 (qpax 弹性 QP)
  │  约束: 关节限位 CBF + 自碰撞 CBF + 障碍物 CBF
  ▼
安全关节速度 → Euler 积分 → 下一步 q
```

### 关键设计决策

| 决策 | 说明 |
|------|------|
| 纯 P 控制, 无 kd | 显式 Euler 离散化下 kd≥1 不稳定 |
| 固定 shape 障碍物槽位 | MAX_JAX_OBSTACLES=8, 未用槽位 h=1e6 填充, 避免 JAX 重编译 |
| 障碍物通过 h_args 传入 | 不烘焙到闭包, 避免障碍物更新触发 JAX 重编译 |
| qpax 弹性 QP | 支持自动微分 + JIT 编译 + 批量求解 |
| OSCBF task-consistent P | 让 QP 自动在零空间修正, 不干扰末端执行器任务 |

## 外部依赖

| 依赖 | 用途 | 必需? |
|------|------|-------|
| `jax` | 自动微分, JIT 编译, GPU 加速 | ✅ 核心 |
| `qpax` | JAX 可微分 QP 求解器 | ✅ 核心 |
| `cbfpy` | CBF 框架 (CBFConfig, CBF 类) | ✅ 核心 |
| `numpy` | 数组操作 | ✅ 核心 |
| `scipy` | 旋转, 稀疏矩阵, .mat 加载 | ✅ 核心 |
| `python-fcl` | FCL 距离/碰撞查询 | ✅ 碰撞检测 |
| `trimesh` | STL 网格加载 | ✅ 碰撞检测 |
| `osqp` | OSQP QP 求解器 (legacy 路径) | ⚠️ 可选 |
| `PyYAML` | YAML 配置加载 | ✅ 配置 |

## 快速开始

### 安装依赖

```bash
pip install jax jaxlib qpax cbfpy numpy scipy python-fcl trimesh osqp pyyaml
```

### 基本使用

```python
import sys
sys.path.insert(0, '.')  # 使 'work' 包可导入

import numpy as np
from work.jax_control_facade import JaxControlLoop

# 1. 创建控制循环
ctrl = JaxControlLoop(
    dt=0.002,              # 控制周期 2ms
    w_pos=20.0,            # 位置权重
    w_orient=10.0,         # 姿态权重
    w_joint=0.1,           # 零空间关节权重
    enable_x64=True,       # JAX float64
)

# 2. 准备路径参考 (从 .mat 加载)
from work.ik_data_loader import load_trajectory
traj = load_trajectory('data/ik_input.mat')

# 3. 设置路径跟踪
ctrl.setup_path_tracking(
    path_geometry=traj.path_geometry,
    path_config=traj.path_config,
)

# 4. 运行控制循环
q = np.zeros(9)  # 初始关节角
for step in range(num_steps):
    # 准备障碍物数据 (固定 shape)
    obs_pos = ...      # (MAX_OBS, 3)
    obs_radii = ...    # (MAX_OBS,)
    obs_enabled = ...  # (MAX_OBS,) 1=启用, 0=禁用

    # 执行一步
    result = ctrl.path_tracking_step(
        q=q,
        obs_pos=obs_pos,
        obs_radii=obs_radii,
        obs_enabled=obs_enabled,
    )

    q = result.q_next  # 下一步关节角
    # result.u_safe    → 安全关节速度
    # result.err_6d    → 6D 任务误差
    # result.qp_ok     → QP 是否成功
```

### 运行测试

```bash
cd portable_oscbf
python -m pytest tests/ -v
```

## 移植到新项目

### 步骤 1: 复制整个 `portable_oscbf/` 目录

```bash
cp -r portable_oscbf/ /path/to/new_project/oscbf_controller/
```

### 步骤 2: 修改 URDF (如使用不同机器人)

编辑 `work/nineaxis_manipulator_jax.py` 和 `work/nineaxis_kinematics.py` 中的 `JOINT_CHAIN` 常量, 匹配新机器人的 DH 参数/关节链。

### 步骤 3: 修改碰撞模型

编辑 `work/collision_envelope.py` 中的球体模型:
- `NUM_ENVIRONMENT_COLLISION_SPHERES` — 碰撞球数量
- `ENVIRONMENT_SPHERE_LINK_INDICES` — 每个球附着的连杆
- `ENVIRONMENT_SPHERE_LOCAL_CENTERS_M` — 球心在连杆坐标系中的位置
- `ENVIRONMENT_SPHERE_RADII_M` — 球半径

### 步骤 4: 替换轨迹数据

将新机器人的轨迹 `.mat` 文件放入 `data/`, 或修改 `ik_data_loader.py` 适配新的数据格式。

### 步骤 5: 调整配置

编辑 `config/` 下的 YAML 文件:
- `nineaxis.yaml` — 关节限位, 速度限制
- `controller_params.yaml` — 控制增益
- `fcl_params.yaml` — 碰撞检测参数

### 步骤 6: 集成到新控制框架

`JaxControlLoop` 是纯计算类, 无 ROS 依赖。只需:

```python
from work.jax_control_facade import JaxControlLoop

# 在你的控制框架中初始化
ctrl = JaxControlLoop(dt=0.002, ...)

# 每个控制周期调用
result = ctrl.path_tracking_step(q, obs_pos, obs_radii, obs_enabled)
q_next = result.q_next
```

## 已排除的文件 (ROS 依赖)

以下文件**未包含**在本移植包中, 因为它们依赖 ROS 2 (`rclpy`, `sensor_msgs`, `geometry_msgs` 等):

| 文件 | 原因 |
|------|------|
| `newaxis/run_oscbf_rviz_newaxis.py` | ROS 2 主节点 |
| `newaxis/rviz_publisher.py` | RViz 可视化 |
| `newaxis/transition_executor.py` | ROS 消息类型 |
| `newaxis/obstacle_scene_builder.py` | ROS 节点状态 |
| `newaxis/oscbf_controller.py` | ROS 节点引用 |
| `newaxis/tracking_execution.py` | ROS 节点引用 |
| `newaxis/cbf_constraint_builder.py` | ROS 节点引用 |
| `newaxis/avoidance_state_machine.py` | 为 ROS runner 设计 |

## 参考文献

1. Morton, W., & Pavone, M. (2025). *Safe, Task-Consistent Manipulation with OSCBF*. IROS 2025.
2. Ames, A. D., et al. (2019). *Control Barrier Functions: Theory and Applications*. ECC.
3. qpax: JAX-based differentiable QP solver. https://github.com/kevin-tracy/qpax
4. cbfpy: CBF framework for Python. https://github.com/hmcm-lab/cbfpy
