## 4. 核心模块与接口

> 移植 = 复制这些模块并替换机器人模型相关常量。全部在 `portable_oscbf/work/` 下。

### 4.1 机器人模型层

| 文件 | 职责 | 移植时修改 |
|------|------|-----------|
| `nineaxis_manipulator_jax.py` | JAX POE FK / 6D Jacobian / OBB 变换 | `JOINT_CHAIN`（关节链）、`home_pose`、`joint_limits`、`joint_max_velocities` |
| `obb_collision_model.py` | OBB 包络盒数据（每关节系一个，对齐 xyz 轴，贴合 STL） | `OBB_LINK_INDICES`、`OBB_LOCAL_CENTERS_M`、`OBB_HALF_EXTENTS_M`、`OBB_LOCAL_ROTATIONS`（按 STL 实测标定） |
| `actuator_limits.py` | 执行器限位（YAML） | 电机规格 |
| `joint_velocity_limits.py` | 速度/加速度边界 | 电机规格 |

**关键接口**（`nineaxis_manipulator_jax.py`）:
```python
robot = NineaxisManipulatorJAX()
T_ee  = robot.forward_kinematics(q)      # (4,4) or batched   [现状已实现]
J     = robot.ee_jacobian(q)             # (6,9)  [J_pos; J_rot] [现状已实现]
# obb_T = robot.obb_transforms(q)        # (N_obb, 4,4) 目标接口; 现状只有球模型
#                                          self_collision_data / environment_collision_data
```

### 4.2 路径/参考层

| 文件 | 职责 |
|------|------|
| `ik_data_loader.py` | `.mat` 轨迹加载 → 弧长路径几何（位置/切线/姿态/omega_per_m/进给率） |
| `jax_path_following.py` | JAX 弧长路径跟踪状态机（投影、前进、终点制动） |
| `jax_posture_reference.py` | JAX 姿态参考插值（`q_posture_ref(ell)` 固定 shape 闭包） |

### 4.3 控制内核层（模块化 JIT — 目标）

> **现状**：`jax_kernel_factory.py` 是功能级整块编译（`path_tracking` 一个函数串完参考→OSC→CBF→QP→积分，约 7 个整块内核）。**目标**：每个子函数各自 `@jax.jit`（仿 OSCBF 源码 `controllers.py` 风格），首次编译快、逐步执行效率与 OSCBF 一致。

| 文件 | 职责 | 状态 |
|------|------|------|
| `oscbf_velocity_config.py` | cbfpy `CBFConfig`：`P/q`（任务一致性 P 矩阵）、`h_2`（全部 CBF）、`alpha`；目标 `relax_cbf=True` | 🟢（relax 目标） |
| `jax_barrier_terms.py` | 固定 shape 障碍物几何 + CBF RHS（`MAX_JAX_OBSTACLES=8`） | 🟢 |
| `dpax_collision.py` | **DCOL 可微碰撞内核**（`DpaxSelfCollisionChecker`，Box/Sphere/Capsule + `proximity`/`polytope_proximity`，`jax.grad` 提供梯度） | 🟡（现状在 `DCOLuse/` 且仅 CLI 后端，未进热路径） |
| `dcol_alpha_calibration.py` | DCOL alpha 逐对离线校准（V2 DPAX alpha 校准流程） | 🟡（**不存在，需新建**） |
| `qp_solver_health.py` | QP 健康检查（收敛、KKT 残差、slack 用量） | 🟢 |
| `jax_kernel_factory.py` | 控制内核编译入口；目标改为模块化 JIT | 🟡（目标） |

> ⚠️ **`qpax_solver.py`（`solve_qp_elastic` 包装类）现状无调用者（legacy）**。热路径 QP 由 cbfpy `CBF.qp_solver` 承担：`relax_cbf=True` 时自动选 `qpax.solve_qp_elastic`，`False` 时选硬 `qpax.solve_qp`。

### 4.4 零空间策略层（可插拔 — 目标）

> 参考 `docs/OSCBF_在线可操作度零空间目标_架构参考.md`。零空间策略只生成名义零空间速度，不负责安全裁决，QP 不感知零空间来源。
> ⚠️ 本层全部文件**现状不存在**（现状是关节中点 `q_des=(q_min+q_max)/2` 硬编码），是目标实现任务。

| 文件 | 职责 | 状态 |
|------|------|------|
| `nullspace_policy.py` | `NullspacePolicy` 接口 + `ManipulabilityGradientPolicy` | 🟡（不存在，需新建） |
| `manipulability_metric.py` | 正则化对数可操作度 φ=½logdet(JₛJₛᵀ+εI) 及其梯度（自动微分） | 🟡（不存在，需新建） |
| `online_ik_des.py` | 在线可操作度零空间目标辅助（若需） | 🟢（存在，辅助用） |

**核心公式**（可操作度零空间速度，§5.2）:
```
qdot_N = s_v · k_m · ρ(q) · N · W_q⁻¹ · Nᵀ · g_m
  g_m = ∇_q φ,  φ = ½ logdet(J_s J_sᵀ + ε I)
  s_v = 整体速度缩放 (min(1, v_N,max/‖k_m·ρ·d_m‖))  ← 不得逐关节裁剪
  ρ   = 平滑激活系数 (φ≤φ_low 激活, φ≥φ_target 关闭)
  W_q = diag(1/ẋ_max²) 关节速度归一化度量
```

### 4.5 主机 facade 层

| 文件 | 职责 |
|------|------|
| `jax_control_facade.py` | `JaxControlLoop`：输入归一化、JIT warmup、`path_tracking_step()` / `tracking_step()` |
| `jax_control_loop.py` | 兼容性 re-export（历史名称） |

**核心用法**（移植目标中嵌入控制循环的方式）:
```python
from work.jax_control_facade import JaxControlLoop

ctrl = JaxControlLoop(dt=0.002, w_pos=20.0, w_orient=10.0, w_joint=0.1, enable_x64=True)
ctrl.configure_path(geometry=traj.path_geometry, path_config=traj.path_config)
ctrl.init_cbf()

for step in range(N):
    result = ctrl.path_tracking_step(
        q=q, path_state=path_state,
        obs_pos=..., obs_radii=..., obs_enabled=..., obs_d_safe=..., obs_vel=...,
    )
    q = result.q_next
```

### 4.6 环境碰撞与感知层（DCOL 内核 + 原始点云感知）

| 文件 | 职责 |
|------|------|
| `dpax_collision.py` | DCOL 可微障碍物距离（与自碰撞同一内核） |
| `point_cloud_obstacles.py` | FCL 点云距离（**仅原始点云感知**，DCOL 无点云原语） |
| `safety_snapshot.py` | 固定 shape ESDF 距离场快照（感知 → `sdf_*` 输入） |
| `fcl_collision.py` | FCL 基元碰撞（仅用于离线/过渡验证基准） |

### 4.7 明确排除（不移植）

| 排除项 | 原因 |
|--------|------|
| `oscbf_qp_solver.py`（OSQP legacy） | 非 JAX，无法进编译图；qpax 已提供弹性等价 |
| FCL 基元作为控制热路径碰撞源 | 不可微；控制热路径碰撞统一走 DCOL（可微、JAX） |
| 全部 `newaxis/` ROS 文件 | 依赖 rclpy/sensor_msgs，控制核是纯计算类 |
| `oscbf_torque_config.py`（力矩级） | 本路线是速度级控制；如需力矩级另起路线，不在本文档范围 |

---
