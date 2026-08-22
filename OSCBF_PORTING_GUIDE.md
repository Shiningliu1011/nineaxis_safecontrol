# OSCBF 框架移植实现指南（从零实现版）

> **读者对象**: 正在从零实现 OSCBF 实时安全控制的新项目（当前相关内容一点未实现）。
> **本文档用途**: 给出**目标架构 + 逐步实现顺序**，每步有验收门。按 §9 的 Step 1→9 顺序实现即可。
> **目标架构**: JAX OSCBF 便携控制器——模块化 JIT 控制内核 + OBB/DCOL 可微碰撞 + 弹性 QP + 可操作度零空间。
> **参考实现（两个）**:
> - `portable_oscbf/`（本仓库）——可复制的参考代码。🟢 标记的模块它已有，可复制改造；🟡 标记的模块它**也没有**，需按本文档规格从零实现。
> - OSCBF 源码（`/home/lsn/oscbf/`，StanfordASL）——控制器数学的权威参考（`PoseTaskVelocityController`、`OSCBFVelocityConfig`）。
> **机器人**: 9-DOF 冗余机械臂（1 棱柱 J1 + 8 旋转 J2-J9），基于 Morton & Pavone, *Safe, Task-Consistent Manipulation with OSCBF* (IROS 2025)。
>
> **生成日期**: 2026-08-06

---

## 目录

1. [路线总览](#1-路线总览)
2. [技术选型决策表（每模块唯一选择）](#2-技术选型决策表每模块唯一选择)
3. [控制管线（单条链路）](#3-控制管线单条链路)
4. [核心模块与接口](#4-核心模块与接口)
5. [数学模型与公式](#5-数学模型与公式)
6. [关键参数（目标值）](#6-关键参数目标值)
7. [碰撞模型](#7-碰撞模型)
8. [配置体系](#8-配置体系)
9. [实现步骤（从零开始，Step 1→9）](#9-实现步骤从零开始-step-19)
10. [验证标准](#10-验证标准)
11. [踩坑教训（必读）](#11-踩坑教训必读)
12. [附录 A（文件清单）](#附录-a-目标架构文件清单含复制新建标注)
13. [附录 B（依赖）](#附录-b-依赖清单)

---

## 1. 路线总览

**一句话**: 实现 **JAX OSCBF 便携控制器**——控制数值图按**模块分别 JIT 编译**（仿 OSCBF 源码的编译粒度，编译快、逐步执行效率一致），主机端用 `JaxControlLoop` facade 调用，自碰撞用**贴合 STL 的 OBB 包络盒 + DCOL 可微碰撞内核**，原始点云感知用 FCL/ESDF 在 JAX 内核之外。

### 1.1 参考实现可复制性（先读，决定每步"复制 or 从零实现"）

> **🟢** = `portable_oscbf/` 有参考代码，**复制改造**（换机器人模型/参数即可）；**🟡** = `portable_oscbf/` 没有，**按本文档规格从零实现**。

| 模块 | 参考实现状态 | 目标架构（你要实现的） | 实现方式 |
|------|------------|----------------------|---------|
| CBF 松弛 | `relax_cbf=False`（硬 QP + 受控停车） | `relax_cbf=True` 弹性 QP，取消停车 | 🟡 改配置 + 重新验证 |
| JAX 编译粒度 | 功能级整块编译（`jax_kernel_factory.py` 约 7 个整块内核） | 模块化 JIT，各子函数独立 `@jax.jit` | 🟡 重构 |
| 速度限幅 | QP box 约束承担（`u_nom` 不 clip） | 控制器内 clip（OSCBF 式）+ QP box 固有约束 | 🟡 新增 clip |
| 自碰撞模型 | 17 球 + 14 对 | 每连杆 OBB（贴合 STL）+ DCOL 可微距离 | 🟡 新建 |
| 碰撞距离内核 | JAX 球模型；DCOL 仅 CLI 后端 | DCOL 为控制热路径碰撞内核 | 🟡 迁入+接线 |
| 环境碰撞 | FCL 点云 | DCOL 基元障碍物 + FCL/ESDF 原始点云感知 | 🟡 部分新建 |
| 零空间目标 | 关节中点 `q_des=(q_min+q_max)/2` | 在线可操作度梯度（`ManipulabilityGradientPolicy`） | 🟡 新建 |
| QP 求解 | cbfpy `CBF.qp_solver`（硬 `solve_qp`） | cbfpy `CBF.qp_solver`（弹性 `solve_qp_elastic`） | 🟡 改 relax_cbf |
| **JAX POE 运动学** | **已实现** | 同左 | **🟢 复制** |
| **cbfpy CBFConfig 框架** | **已实现** | 同左 | **🟢 复制** |
| **固定 8 槽位障碍物接口** | **已实现** | 同左 | **🟢 复制** |
| **弧长路径跟踪** | **已实现** | 同左 | **🟢 复制** |
| **qpax/cbfpy 求解链路** | **已实现** | 同左 | **🟢 复制** |
| **x64 / solver_tol=1e-3 / 权重 20/10/0.1** | **已实现** | 同左 | **🟢 沿用** |

**为什么选这条架构**:
1. **JAX 控制核有验证基线**：参考实现已完成 12s/30s 严格验证（`max_ee=0.088mm`、`qp_fail=0`、`dyn_min>4mm`），控制数学可信。⚠️ 该基线与本架构的 🟡 改动（OBB/DCOL/弹性QP/可操作度）无直接关系，新架构需按 §10 重新验证。
2. **可移植**：参考实现已剥离全部 ROS 依赖，控制核是纯计算类，可嵌入任意控制框架。
3. **性能与编译兼顾**（🟡 目标）：模块化 JIT——单函数图小、首次编译快，逐步执行效率与 OSCBF 源码一致（L94 教训避免单一巨型内核）。
4. **安全语义**（🟡 目标）：CBF QP 用弹性松弛（`relax_cbf=True`），不可行时由 slack 弹性缓解，取消受控停车。

> **JAX 边界（L67 调整后）**：控制数值图 + **碰撞距离计算（DCOL）** 走 JAX；ROS/传感器 I/O/点云解码/ESDF 构建留在 JAX 外（原始感知不追求可微）。DCOL 当前无点云原语，故原始点云感知保留 FCL/ESDF。实现时遵守同一边界。

---

## 2. 技术选型决策表（每模块唯一选择）

> 每模块只有一个实现，没有可选项。选型理由写清"为什么选它"和"为什么不选其它"，供移植决策存档。**状态列**：🟢=现状已实现（沿用）；🟡=目标改动（未实现，见 §1.1）。

| # | 模块 | **选定方案** | 选型理由 | 状态 |
|---|------|------------|---------|------|
| 1 | 控制内核 | **模块化 JIT 编译**（仿 OSCBF 源码：控制器、FK/Jacobian、`safety_filter` 各自独立 `@jax.jit`） | 单函数图小、**首次编译快**（避免单一巨型内核的长 JIT 预热，L94）；逐步执行效率与 OSCBF 源码一致。不选单一巨型编译内核（首编慢、吃满 CPU 核）。⚠️ 现状 `jax_kernel_factory.py` 是功能级整块编译，本行是目标 | 🟡 |
| 2 | QP 求解器 | **qpax**（经 cbfpy `CBF.qp_solver` 调用 `solve_qp_elastic` 弹性 QP） | JAX 原生、可微、可 JIT；`relax_cbf=True` 时 cbfpy 自动选 `solve_qp_elastic`。⚠️ `qpax_solver.py` 包装类当前无调用者（legacy），热路径求解器是 cbfpy 的 `CBF.qp_solver`。不选 OSQP（非 JAX） | 🟡 |
| 3 | CBF 框架 | **cbfpy**（`CBFConfig` / `CBF`） | 作者官方框架，自动 Lie 导数，固定 `num_cbf` 模式与固定 shape 接口天然契合 JIT。不选手写 CBF 约束列表（不可微、无框架校验） | 🟢 |
| 4 | 运动学 | **JAX POE**（`nineaxis_manipulator_jax.py`） | FK/Jacobian 在编译图内，无主机往返。不选 NumPy 运动学（仅用于 FCL 碰撞的宿主侧 FK 和离线 IK） | 🟢 |
| 5 | 自碰撞 | **OBB 包络盒（几何）+ DCOL 可微距离内核**（OBB 对齐关节系 xyz 轴、贴合 STL；`dpax.endpoints.proximity` / `dpax.polytopes.polytope_proximity` 计算可微距离与梯度，逐对标定 α） | OBB 紧贴机械臂几何、减少包络空气；DCOL 提供 JAX 可微距离+梯度，与控制内核同语言（JAX）；alpha 按 V2 DPAX 离线校准流程标定。⚠️ 现状自碰撞是 17 球+14 对（非 OBB），DCOL 未进热路径，本行是目标 | 🟡 |
| 6 | 环境碰撞 | **DCOL 障碍物碰撞 + FCL/ESDF 原始点云感知**（障碍物基元距离走 DCOL；原始点云经体素化/ESDF 后进入，DCOL 无点云原语） | 障碍物碰撞与自碰撞同内核（DCOL 可微）；原始点云感知仍用 FCL/ESDF（L67 感知 I/O 边界——DCOL 当前无点云原语）。现状环境碰撞即 FCL，DCOL 障碍物部分为目标 | 🟡 |
| 7 | 障碍物接口 | **固定 shape 8 槽位 + enabled mask**（`jax_barrier_terms.py` `MAX_JAX_OBSTACLES=8`） | 未用槽位 `h=1e6` 填充、QP 自动忽略；障碍物经 `h_args` 传参而非闭包捕获，更新不触发重编译。⚠️ 8 槽位为球参数（pos/radius）；DCOL 支持球/胶囊/盒，需扩展槽位 schema 或用球近似 | 🟢 |
| 8 | 参考模式 | **弧长路径跟踪**（`jax_path_following.py`） | 项目默认且最成熟模式；切向由进给率控制、误差反馈仅横向，与 CBF/限速/终点制动天然耦合。**配合固定姿态（fixed）使用时 `max_oe≈0.004-0.05°`**。非固定工具轴子模式存在 ~0.3° 已知残差（L76），需求 <0.1° 姿态精度时用 fixed 姿态，不要切回固定时间模式 | 🟢 |
| 9 | 积分 | **显式 Euler** `q += u_safe * dt` | 与纯 P 控制匹配；速度级控制下简单稳定 | 🟢 |
| 10 | 任务 | **6-DOF pose 任务**（位置 + 姿态） | 覆盖完整操作空间需求。`tool_axis_task.py` 提供 5D 任务（保持工具轴）作为同一内核内的任务模式切换，非独立分支 | 🟢 |
| 11 | 权重 | `w_pos=20, w_orient=10, w_joint=0.1` | OSCBF 任务一致性 P 矩阵下已验证的最优组合。权重平方后**位置/零空间比 = 20²/0.1² = 40,000:1，姿态/零空间比 = 10²/0.1² = 10,000:1** | 🟢 |
| 12 | 控制增益 | 纯 P，无 kd；`kp_task=diag(kp_pos×3, kp_orient×3)`，`kp_pos=60/120`，`kp_orient=10` | 显式 Euler 下 kd≥1 必不稳定（L1）；增益为固定值，不做运行时动态调节。零空间不再用 `kp_joint` 回中（见 #18） | 🟢 |
| 13 | 数值精度 | **JAX x64**（`enable_x64=True`） | float32 曾触发编译 ABI 问题（L77），x64 保证精度且 fix shape | 🟢 |
| 14 | QP 容差 | `solver_tol=1e-3` | 已验证的精度/速度平衡点 | 🟢 |
| 15 | CBF 松弛 | **弹性 QP**（`relax_cbf=True`，cbfpy 经 qpax `solve_qp_elastic`，CBF slack δ） | 与 cbfpy 默认一致（`relax_cbf` 默认 True）；约束冲突不可行时由 slack 弹性缓解，**取消受控停车**（⚠️ 目标；现状是 `relax_cbf=False` 硬约束+停车）。`cbf_relaxation_penalty` 取大值保证正常工况 slack≈0 | 🟡 |
| 16 | 配置 | **YAML**（`config/nineaxis.yaml` 等）+ cbfpy config | 参数与代码分离，一次加载，控制热路径不读文件 | 🟢 |
| 17 | 速度限幅位置 | **控制器内 clip**（OSCBF 源码形式：`u_nom = clip(u_nom, qdot_min, qdot_max)`） | 与 OSCBF 源码 `PoseTaskVelocityController` 一致（[controllers.py:286](file:///home/lsn/oscbf/oscbf/core/controllers.py#L286)）。⚠️ 现状 `u_nom` 不 clip、由 QP box 承担，本行是目标。QP 的 `u_min/u_max` 是 cbfpy QP 约束集的一部分（弹性模式下 box 行同样带 slack，见 §5.5） | 🟡 |
| 18 | 零空间目标 | **在线可操作度梯度**（`ManipulabilityGradientPolicy`：`qdot_N = s_v·k_m·ρ·N·W_q⁻¹·Nᵀ·∇_qφ`，φ=½logdet(JₛJₛᵀ+εI)） | 不再跟踪固定关节中点；每周期按当前构型在线生成内部自运动，改善可操作度。⚠️ 现状是关节中点 `q_des=(q_min+q_max)/2`，本行是目标。架构参考 `docs/OSCBF_在线可操作度零空间目标_架构参考.md`。安全职责由独立奇异性 CBF 承担 | 🟡 |

---

## 3. 控制管线（单条链路）

```
  .mat 轨迹 → ik_data_loader → 弧长路径几何 (PathGeometry)
                                   │
  主机端每步 (JaxControlLoop) ──────┤
  ┌─────────────────────────────────┴────────────────────────────┐
  │  path_tracking_step(q, path_state, obs_*, sdf_*)             │
  │                                                              │
  │  ┌──────── 模块化 JIT (仿 OSCBF 源码, 各自 @jax.jit) ────────┐ │
  │  │  ① 参考采样    PathReference(progress_m, pos, rot, omega) │ │
  │  │  ② 标称控制    6-DOF 速度级 OSC (P-only)                 │ │
  │  │     task_vel = [v_des;ω_des] - K_task·err_6d             │ │
  │  │     u_nom = J_hash@task_vel                              │ │
  │  │           + s_v·k_m·ρ·N·W_q⁻¹·Nᵀ·∇φ  (可操作度零空间)    │ │
  │  │     u_nom = clip(u_nom, qdot_min, qdot_max)  ← OSCBF 源码 │ │
  │  │  ③ 碰撞+距离   DCOL 可微距离 (OBB 自碰撞 + 障碍物)        │ │
  │  │     G_row = ∇h 经 jax.grad(proximity) 获得               │ │
  │  │  ④ CBF 构建    h_2(q, obs_*, sdf_*) 全部约束 (cbfpy)      │ │
  │  │     (OBB/DCOL 自碰撞 + 限位 + 障碍物 + ESDF + 奇异性)     │ │
  │  │  ⑤ QP 求解     弹性 QP, relax_cbf=True                   │ │
  │  │     min ½uᵀPu+qᵀu+ρ‖δ‖² s.t. G·u-δ ≤ α·h + dh/dt        │ │
  │  │     u_min ≤ u ≤ u_max (QP 固有硬约束)                    │ │
  │  │  ⑥ 积分        q_next = q + u_safe·dt                    │ │
  │  └───────────────────────────────────────────────────────────┘ │
  │    输出: u_safe, q_next, err_6d, qp_ok, dyn_min, δ_slack     │
  └──────────────────────────────────────────────────────────────┘
        │
  DCOL 障碍物距离 (JAX 内, 与自碰撞同内核)
  原始点云感知 (JAX 外): FCL/ESDF → 体素化/距离场 → 经 obs_* / sdf_* 传入
```

**单步数据流**（主机端 → 内核）：

| 输入 | 形状 | 说明 |
|------|------|------|
| `q` | `(9,)` float64 | 当前关节角 |
| `path_state` | `(n,)` | 弧长路径状态（`jax_path_following` 状态机） |
| `obs_pos` | `(8,3)` | 障碍物中心，未用槽位填 0 |
| `obs_radii` | `(8,)` | 障碍物半径 |
| `obs_enabled` | `(8,)` | 1=启用 0=禁用 |
| `obs_d_safe` | `(8,)` | 每障碍物安全距离 |
| `obs_vel` | `(8,3)` | 障碍物速度（动态 CBF 的 dh/dt） |
| `obs_radius_dot` | `(8,)` | 障碍物半径变化率 |
| `obs_alpha` | `(8,)` | 每障碍物 CBF 增益 |
| `sdf_distance` | `(nx,ny,nz)` | ESDF 距离场（可选） |
| `sdf_origin` / `sdf_voxel_size` / `sdf_enabled` / `sdf_margin` | — | 距离场栅格合同 |

**输出**: `q_next`（下一步关节角）、`u_safe`（安全速度）、`err_6d`（6D 任务误差）、`qp_ok`（QP 成功标志）、`dyn_min`（最小安全裕度）、`qp_diagnostics`。

---

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

## 5. 数学模型与公式

### 5.1 运动学（POE）

```
T_ee(q) = exp([S_1]·q_1) · ... · exp([S_9]·q_9) · M
  其中 S_i 为世界系螺旋轴（零位构型），M 为 q=0 时末端 SE(3)
  转动关节: S = [ω; p×ω]
  棱柱关节: S = [0; v]
```

空间雅可比（螺旋轴）:
```
J_s(:,i) = Ad_{T_{i-1}} · S_i,    Ad = [R, 0; skew(p)R, R]
6D 雅可比: J_full = [J_pos; J_rot],  J_rot = J_s[:3,:]
```

### 5.2 标称控制（6-DOF 速度级 OSC，纯 P）

```
err_6d = [p - p_des ; e_rot(R, R_des)]          # 6D 任务误差
e_rot = -0.5 · Σ_k cross(R[:,k], R_des[:,k])     # 姿态误差 (论文方法)

# 任务速度 (P-only, 无 kd)
task_vel = [v_des ; ω_des] - K_task · err_6d      # K_task = diag(kp_pos×3, kp_orient×3)
J_hash = Jᵀ · (J·Jᵀ + λ²I)⁻¹                     # 阻尼伪逆, λ=1e-3
N = I - J_hash·J                                  # 零空间投影

# 零空间: 在线可操作度梯度 (不再用固定关节中点/回中)
φ     = ½ · log det(J_s·J_sᵀ + εI)               # J_s = S_x·J, S_x = diag(I₃, l_c·I₃)
g_m   = ∇_q φ                                    # 自动微分 (JAX)
d_m   = N·W_q⁻¹·Nᵀ·g_m                          # 投影到当前零空间
s_v   = min(1, v_N,max / ‖k_m·ρ·d_m‖)            # 整体缩放, 不逐关节裁剪
u_null = s_v · k_m · ρ(q) · d_m                  # ρ = 平滑激活系数

# 标称 + 速度限幅 (目标: OSCBF 源码形式的控制器内 clip)
u_nom = J_hash·task_vel + u_null
u_nom = clip(u_nom, qdot_min, qdot_max)   # 目标: 与 OSCBF PoseTaskVelocityController 一致
                                            # ⚠️ 现状 u_nom 不 clip, 速度限幅由 QP box 承担

# 注: QP 的 u_min ≤ u ≤ u_max 是 cbfpy QP 约束集的一部分 (与 CBF 行同类); 弹性模式下 box 行同样带 slack, 见 §5.5
```

### 5.3 OSCBF 任务一致性 P 矩阵（核心）

```
P = N_nullᵀ·W_joint²·N_null + Jᵀ·W_task²·J
  W_task = diag(w_pos·I₃, w_orient·I₃),  W_joint = w_joint·I₉
  λ_qp = 1e-3   (阻尼伪逆 J_hash = Jᵀ(J·Jᵀ + λ²I)⁻¹, 与标称共用 λ=1e-3)
```

**性质**: 权重平方后位置/零空间比 = 20²/0.1² = **40,000:1**，姿态/零空间比 = 10²/0.1² = **10,000:1**。QP 优先在零空间修正以满足 CBF，末端任务几乎不变。这是解决"避障 vs 跟踪"冲突的关键，**必须原样保留**。

> ⚠️ **与 OSCBF 源码的关键差异（务必知晓）**: OSCBF 源码 `OSCBFVelocityConfig._P()` 用**动力学一致性逆** `J_bar = M⁻¹Jᵀ(J·M⁻¹·Jᵀ)⁻¹`（需质量矩阵，零空间投影也 M 加权）；本项目用**阻尼伪逆** `J_hash = Jᵀ(J·Jᵀ + λ²I)⁻¹`（纯运动学，免动力学建模）。两者 P 矩阵形式相同，但投影子不同——本项目为移植到任意机械臂主动放弃了动力学一致性。

### 5.4 CBF 约束（相对度 2）

```
速度级:  f(q) = 0,  g(q) = I  →  ∇hᵀ·u + α(h) ≥ 0
QP 编码:  G·u ≤ α·h + dh/dt        (G_row = -∇hᵀ)
动态障碍物: dh/dt = -nᵀ·v_obs - ṙ_obs   (显式传入, cbfpy 不会自动加)
碰撞梯度: ∇h 由 DCOL 提供 —— jax.grad(dpax.endpoints.proximity)(q)
```

**固定约束拓扑**（全部在 `h_2()` 中拼接，行数恒定）:

| 约束 | 行数 | 说明 |
|------|------|------|
| 关节限位 | 2×9 | `h = q_max - q - margin` 与 `h = q - q_min - margin` |
| 自碰撞（OBB via DCOL） | N_pairs | OBB 几何 + DCOL 可微距离，`obb_collision_model.py` 标定，非相邻连杆对 |
| 障碍物（DCOL） | N_obb×8（或聚合） | DCOL 基元距离，masked，未用槽位 `h=1e6` |
| ESDF | N_obb（可选） | 距离场采样 `SDF(p_obb) - margin`（OBB 无半径，按盒表面/最近点采样；原始点云感知） |
| 奇异性 | 1 | `σ_min(J_pos) - tol` |

### 5.5 QP 问题（qpax 弹性 QP — 目标）

```
变量:  x = [u (9,) ; δ (n_cbf,)]
目标:  min  ½·uᵀ·P·u + qᵀ·u  +  ρ·‖δ‖²          # 弹性松弛, relax_cbf=True
约束:  G·u - δ ≤ h + dh/dt        (CBF 可弹性缓解, 不可行不再停车)
       u_min ≤ u ≤ u_max           (速度盒)
       δ ≥ 0
```

- 求解器: cbfpy `CBF.qp_solver` → `qpax.solve_qp_elastic`（`relax_cbf=True` 触发；现状 `relax_cbf=False` 走硬 `solve_qp`）
- `ρ` = `cbf_relaxation_penalty`（取大值，保证正常工况 δ≈0，仅在约束冲突时启用松弛）
- ⚠️ **弹性模式下 `u_min/u_max` box 行同样带 slack**（`solve_qp_elastic` 对所有不等式行统一加松弛惩罚）——不是硬约束。控制器内 clip（目标）才是保证名义命令在界内的机制
- 不活跃槽位用 `h=1e6` 填充 → `G·u ≤ 1e6` 恒满足 → QP 自动忽略，**JIT 拓扑恒定，不重编译**
- **取消受控停车**（目标）：CBF 冲突由 slack 弹性缓解；`δ_slack` 作为诊断量记录，不触发 HARD_STOP。⚠️ 现状是 `relax_cbf=False` 硬约束 + `apply_qp_health_gate` 受控停车

---

## 6. 关键参数（目标值）

> 以下为建议采用的目标值。标注"🟢"者与 `portable_oscbf` 代码一致；标注"🟡"者随目标改动（弹性 QP / OBB+DCOL / 可操作度零空间）需要在新实现中设定。**移植到新机器人必须按单位/限位重新标定**，不要当作已验证常数直接套用。

### 6.1 控制器

| 参数 | 值 | 说明 |
|------|-----|------|
| `dt` | 0.002 s | 名义 500Hz；部署 100Hz 也通过验证 |
| `kp_pos` | 60（fixed）/ 120（非 fixed） | 位置增益 |
| `kp_orient` | 10 | 姿态增益 |
| `damping` | 1e-3 | 标称伪逆阻尼 |
| `qdot_min / qdot_max` | 关节速度限位 | **控制器内 clip**（OSCBF 源码形式）🟡（目标；现状由 QP box 承担） |
| `w_pos` / `w_orient` / `w_joint` | 20 / 10 / 0.1 | 任务一致性 P 矩阵权重 |
| `enable_x64` | True | JAX float64 |
| `solver_tol` | 1e-3 | qpax 求解容差 |

### 6.2 CBF

| 参数 | 值 | 说明 |
|------|-----|------|
| `alpha_joint_limit` | **8.0** 🟢 | 关节限位 CBF 增益（`oscbf_velocity_config.py:82` 硬编码 8.0；⚠️ `config/nineaxis.yaml:135` 写 5.0，两处不一致需统一） |
| `joint_limit_cbf_margin` | **0.01** 🟢 | 限位内缩（`joint_limit_contract.py:12` 为 10mm，非 1mm） |
| `d_safe_collision` | 0.03 m 🟢 | 自碰撞安全距离（现状球模型值；OBB 路线改为 `dcol.d_safe` 近接触标定） |
| `obstacle_h_baseline_alpha` | 10.0 🟢 | 障碍物 CBF 增益基线 |
| `singularity_tol` | 0.005 🟢 | 奇异值下限 |
| `relax_cbf` | **True** 🟡 | **弹性 QP**（cbfpy→`qpax.solve_qp_elastic`），取消停车；⚠️ 现状 `False` 硬约束+受控停车 |
| `cbf_relaxation_penalty` | 1e4 ~ 1e6 🟡 | CBF slack 惩罚 ρ（取大值，正常工况 δ≈0） |
| `MAX_JAX_OBSTACLES` | 8 🟢 | 障碍物槽位数 |
| `smooth_min_temperature` | 0.01 🟢 | 障碍物 soft-min 聚合温度 |
| `dcol.alpha` | 逐对标定 🟡 | DCOL 每约束行 CBF 增益 α，V2 DPAX 离线校准（3/14 行已定、11 行需近接触数据） |
| `dcol.d_safe` | 按近接触标定 🟡 | DCOL 安全距离（近接触数据驱动，勿套用球模型值） |

### 6.3 可操作度零空间

| 参数 | 值 | 说明 |
|------|-----|------|
| `nullspace.policy` | `manipulability_gradient` | 在线可操作度策略（替换关节中点） |
| `manipulability.metric` | `regularized_logdet` | φ = ½logdet(JₛJₛᵀ + εI) |
| `manipulability.epsilon` | 1e-6 | 正则化 ε |
| `manipulability.gain` | 0.15 | 可操作度增益 k_m |
| `manipulability.max_weighted_speed` | 0.25 | 零空间速度上限 v_N,max |
| `gradient.method` | `autodiff` | 自动微分（JAX） |
| `gradient.filter_enabled` | true | 梯度低通 |
| `gradient.filter_alpha` | 0.2 | 低通系数 β |
| `activation.enabled` | false | 首版关闭激活，验证梯度后再启用 |
| `joint_metric.type` | `velocity_normalized` | W_q = diag(1/ẋ_max²) |
| `characteristic_length` | 0.4 | 特征长度 l_c（任务尺度矩阵） |

> 参数来自 `docs/OSCBF_在线可操作度零空间目标_架构参考.md` §13，具体数值需按机器人单位/限位重新标定。

### 6.4 路径跟踪

| 参数 | 值 | 说明 |
|------|-----|------|
| `projection_half_window_segments` | 96 | 弧长投影半窗 |
| `max_projection_speed_m_s` | 0.12 | 投影最大速度 |
| `reference_lead_m` | 1e-5 (0.01mm) | 参考超前量（投影后积分修正后，离散时序滞后已消除，0.01mm 足以让参考点略超前于投影并低于 0.1mm 验收门） |
| `cross_track_stop_m` | 1e-3 | 横向误差停车阈值 |
| `endpoint_braking_deceleration_m_s2` | 0.05 | 终点虚拟制动 |
| `maximum_tool_axis_speed_rad_s` | 0.15 | 工具轴角速度上限（防腕部反向点跳变） |
| `endpoint_settle_s` | 0.5 | 终点稳定时间 |

### 6.5 动态障碍物（若启用）

| 参数 | 值 |
|------|-----|
| `cbf_alpha` | 1.5 |
| `d_safe` | 0.08 m |
| `activation` | 0.20 m |

---

## 7. 碰撞模型

### 7.1 自碰撞 —— OBB 几何 + DCOL 可微距离内核（目标）

> ⚠️ **现状是 17 球 + 14 对**（`collision_envelope.py` 的 17 个球 + `oscbf_collision_config.py` 的 `SELF_COLLISION_PAIRS` 14 对），本文描述的是**目标**（每连杆 OBB + DCOL）。

**要求**（目标）: 不用球模型；每个机械臂关节系下定义**一个 OBB（定向包围盒），对齐该关节坐标系 xyz 轴**，尺寸参考 STL 实测，**紧密贴合连杆几何，不产生大范围包络空气**。

`obb_collision_model.py` 定义每个 OBB 的参数：

```python
OBB_LINK_INDICES:        每个 OBB 附着的连杆索引
OBB_LOCAL_CENTERS_M:     盒心在连杆坐标系的位置
OBB_HALF_EXTENTS_M:      半边长 (hx, hy, hz)，按 STL 包围盒标定
# 姿态: 默认对齐连杆系 xyz 轴; 若连杆系未对齐 STL 主轴, 附加局部旋转 R_obb
```

**距离内核 = DCOL**（`dpax_collision.py`）:

```python
# 每步: FK 得各 OBB 世界位姿 → DCOL 可微距离
from dpax.endpoints import proximity          # 线段/点基元可微距离
from dpax.polytopes import polytope_proximity # OBB/凸多面体可微距离
proximity_jit = jax.jit(proximity)
grad = jax.grad(proximity, argnums=(...))     # 提供 ∇h

h = d_DCOL(OBB_i, OBB_j) - d_safe
∇h = jax.grad(proximity) @ J_point            # DCOL 梯度 → CBF 行
```

**DCOL alpha 校准**: 每个自碰撞约束行的 CBF 增益 `α` 必须**离线校准**（V2 DPAX alpha 校准流程：3/14 行已定、拓扑已认证、11 行需近接触数据）。⚠️ 该"14 行"对应**现状球对拓扑**的 14 对；切换到 OBB 目标拓扑后配对数量会重新确定，需按新拓扑重跑校准。alpha 不是全局常数，是逐约束行参数。

**标定要求**:
1. 从 STL 求每连杆的 OBB：对连杆系坐标下的网格顶点求主轴方向 → 旋转使 xyz 对齐 → 求最小包围盒
2. 若连杆系与 STL 主轴有固定偏差，在 `R_obb` 中记录，随 FK 组合
3. **验证**：OBB 体积 / 连杆包围盒体积比尽量接近 1（包络空气最小）；RViz 渲染 OBB 与 STL 重合检查
4. 确定自碰撞对：仅非相邻连杆，跳过已标定拓扑豁免（机械近邻按 L6/L71 流程扫描确认）
5. 逐对校准 DCOL alpha（近接触数据驱动）

### 7.2 环境碰撞 —— DCOL 障碍物 + FCL/ESDF 原始点云感知

- **障碍物基元距离走 DCOL**（与自碰撞同一可微内核）：机器人 OBB vs 障碍物基元（球/胶囊/盒）
- 输出转为 `obs_pos / obs_radii / obs_enabled / obs_d_safe / obs_vel` 传入 JAX 内核
- **原始点云感知仍用 FCL/ESDF**：`point_cloud_obstacles.py`（FCL 距离）、`safety_snapshot.py`（ESDF 距离场）——DCOL 当前无点云原语，点云经体素化/ESDF 后进入

**边界**: 原始点云感知不追求可微（L67 感知 I/O 边界），但**碰撞距离计算全部走 DCOL**（自碰撞 + 基元障碍物），FCL 仅保留在原始点云感知与离线验证。

---

## 8. 配置体系

```
portable_oscbf/config/
├── nineaxis.yaml          # 主配置: 关节限位/速度/控制器增益/路径/动态障碍物/QP
├── actuator_modules.yaml  # 电机规格
├── controller_params.yaml # 控制器增益
├── nullspace.yaml         # 零空间策略 (manipulability_gradient + 激活/滤波参数)
├── obb_model.yaml         # OBB 包络盒标定数据 (每连杆半边长/局部旋转)
├── fcl_params.yaml        # FCL 碰撞参数 (环境)
├── obstacle_params.yaml   # 障碍物场景
├── ompl_params.yaml       # 规划参数 (预留)
└── robot_params.yaml      # 机器人物理参数
```

控制热路径**不读 YAML**；参数在 `JaxControlLoop` 初始化时一次加载到配置对象，避免逐步 I/O。零空间策略经配置切换（`joint_center` ↔ `manipulability_gradient`），切换不修改控制循环和 QP。

---

## 9. 实现步骤（从零开始，Step 1→9）

> 这是新项目的**主线实现顺序**，无分叉。🟢 步从 `portable_oscbf/` 复制参考代码改造；🟡 步按本文档规格（§4 模块 / §5 公式 / §6 参数 / §7 碰撞）从零实现。每步通过验收门再进下一步。

### Step 1: 建立机器人模型（JAX）【🟢 复制】
- 从 `portable_oscbf/work/nineaxis_manipulator_jax.py` 复制，改 `JOINT_CHAIN`/`home_pose`/`joint_limits`/`joint_max_velocities` 匹配你的 URDF
- 规格参考：§4.1、§5.1
- **验证**: FK 与 URDF 一致（关节 0 位姿、随机位姿抽查）；Jacobian 数值微分校验

### Step 2: 标定 OBB 包络盒 + DCOL alpha（自碰撞）【🟡 新建】
- 对每个连杆 STL 求对齐关节系 xyz 轴的 OBB：顶点主轴旋转 + 最小包围盒半边长（规格：§7.1、§4.1 `obb_collision_model.py`）
- 写入 `obb_collision_model.py` + `obb_model.yaml`；确定非相邻自碰撞对（含机械近邻豁免，L6/L71 流程）
- 接入 `dpax_collision.py`（DCOL 可微距离，从参考的 `DCOLuse/` 迁入热路径），逐对离线校准 DCOL alpha（V2 校准流程，§6.2）
- **验证**: OBB 与 STL 贴合（RViz 渲染重合；包络体积比接近 1）；零位无假碰撞；DCOL 距离/梯度与 FCL 基准一致

### Step 3: 接入轨迹【🟢 复制】
- 从 `portable_oscbf/work/ik_data_loader.py` 复制，适配你的轨迹数据格式（规格：§4.2）
- **验证**: 弧长路径几何生成成功，`path_geometry` 点/切线/姿态连续

### Step 4: 跑通控制内核（模块化 JIT）【🟢 复制 + 🟡 重构】
- 复制 `portable_oscbf` 的 `jax_control_facade.py` / `jax_kernel_factory.py` / `oscbf_velocity_config.py`（规格：§4.3、§5.2-§5.5）
- 🟡 把 `jax_kernel_factory.py` 从整块编译重构为各子函数独立 JIT（模块化，L94）
- **验证**: `qp_ok=True`、无 NaN、末端误差收敛；首次编译时间可接受（模块化图小）

### Step 5: 接入障碍物 + 弹性 QP【🟢 复制 + 🟡 改语义】
- 构造 `obs_*` 输入（固定 8 槽位，🟢 复制 `jax_barrier_terms.py`），障碍物距离走 DCOL，验证 JIT 不重编译（cache size 保持 1）
- 🟡 切换 `relax_cbf=True`（弹性 QP，取消受控停车——⚠️ 安全语义改动，需重新验证不可行场景行为，见 §5.5）
- **验证**: 障碍物逼近时 `dyn_min` 保持正裕度，正常工况 `δ_slack≈0`

### Step 6: 接入零空间策略【🟡 新建】
- 实现 `ManipulabilityMetric`（φ=½logdet，JAX 自动微分梯度）→ 用中心有限差分对照梯度（规格：§4.4、§5.2、参考 `OSCBF_在线可操作度零空间目标_架构参考.md`）
- `ManipulabilityGradientPolicy` 接入 `u_null`（替换关节中点）；首版 `activation.enabled=false`（§6.3）
- **验证**: 梯度有限差分相对误差 < 容差；零空间残差 `‖J·qdot_N‖` 小；可操作度上升行为可复现

### Step 7: 接入环境感知（可选）【🟢 复制】
- 复制 `point_cloud_obstacles.py` / `safety_snapshot.py` 转 `obs_*` / `sdf_*`（规格：§4.6、§7.2）
- **验证**: 感知链路端到端正常，`obs_*` 输入合法

### Step 8: 嵌入目标控制框架【🟢 复制】
- 在目标框架的控制周期内调用 `ctrl.path_tracking_step(q, path_state, obs_*, sdf_*)`（规格：§4.5 核心用法）
- **验证**: 100Hz 实测 p95 < 10ms

### Step 9: 调优与验收
- 按 §10 验收标准逐项核对；每个 🟡 改动单独验证后再组合

---

## 10. 验证标准

> ⚠️ **精度/安全实测列是改动前基线**（17球 + 硬QP + 关节中点 + 整块编译）。目标路线（OBB+DCOL + 弹性QP + 可操作度 + 模块化JIT）**尚未验证**，达到下表阈值是验收任务，不是现状。

### 精度

| 指标 | 阈值 | 改动前基线实测 |
|------|------|------|
| 最大位置误差 max_ee | < 0.1mm | 0.088-0.099mm |
| 最大姿态误差 max_oe | < 0.1° | 0.004-0.082° |

### 安全

| 指标 | 阈值 |
|------|------|
| 最小动态裕度 dyn_min | > 0mm（实测 4-23mm） |
| QP 失败 qp_fail | = 0 |
| CBF slack δ_slack（正常工况） | ≈ 0（仅约束冲突时启用弹性） |
| 速度跳变（>0.1 rad/s） | 0 |
| OBB 包络贴合 | 包络体积比接近 1，RViz 与 STL 重合 |
| JIT 缓存大小 | = 1（首帧后不重编译） |

### 可操作度零空间（新增验收）

| 指标 | 阈值 |
|------|------|
| 梯度有限差分对照 | 相对误差 < 容差 |
| 零空间残差 ‖J·qdot_N‖ | 数值精度范围（Moore-Penrose 下） |
| 可操作度上升 | 固定末端时 φ 上升后稳定，‖Nᵀg‖ 减小 |
| 末端跟踪误差劣化 | 相比 JointCenterPolicy 不明显劣化 |
| 奇异区安全指标 | 不越界（独立奇异性 CBF 约束，与可操作度性能目标分离） |

### 性能

| 指标 | 阈值 |
|------|------|
| 控制周期 p95 | < 10ms（实测 2.6-4.8ms） |
| 首次 JIT 预热 | < 30s（单线程 XLA；模块化 JIT 图更小，编译更快） |

---

## 11. 踩坑教训（必读）

> 完整版见项目 `LESSONS_LEARNED.md`（96 条）。以下为本路线（模块化 JIT + DCOL/OBB 碰撞 + 弹性 QP + 可操作度零空间）必须遵守的教训。

| # | 教训 | 后果 |
|---|------|------|
| **L1** | 纯 P，无 kd | 显式 Euler 下 kd≥1 发散 |
| **L15** | OSCBF 任务一致性 P 矩阵必须保留 | 否则避障会干扰末端任务 |
| **L67** | JAX 边界 = 控制 + **碰撞数值图**，不是 ROS/感知 I/O | 原始点云解码/ESDF 构建留在 JAX 外；碰撞距离计算走 DCOL（JAX 内） |
| **L70** | qpax warm-start 默认关闭 | 冷启动每步迭代多但整体更快、更安全 |
| **L77** | ESDF dtype 必须 float32（固定 ABI） | float64 触发首帧重编译 |
| **L79** | 求解器 slack ≠ 物理速率违例 | 弹性 QP 下用真实残差判断，不误把 δ 当物理越界 |
| **L83** | 弧长进给在腕部反向点放大 omega_per_m | 运行时硬上限 `ell_dot ≤ 0.15/‖omega_per_m‖` |
| **L94** | 单一巨型 JAX 内核首编慢、吃满 CPU | **选模块化 JIT**（各子函数独立 `@jax.jit`），并设置单线程 XLA |
| **L95** | 初始化瞬态（首帧 0.1mm 误差）不算稳态 | 验证时区分瞬态与稳态 |
| **DCOL** | DCOL alpha 是逐约束行参数，不是全局常数 | 按 V2 DPAX 校准流程逐对离线标定；未标定的近接触行不得用于控制 |
| **DCOL** | "包络相交" ≠ "实体碰撞" | 基于独立网格/STL 可复现证据配置拓扑豁免（L6/L71 流程），不静默跳过 |
| **零空间** | 逐关节裁剪会破坏 `J·qdot_N=0` | 可操作度零空间用**整体缩放** s_v，关节限速交给 QP |
| **零空间** | 阻尼伪逆下 `J(I-J⁺J)≠0` | 记录 `nullspace_leakage=‖J·qdot_N‖`，由任务 QP 项校正 |

**JIT 预热注意事项**（L94）: XLA 编译吃满所有 CPU 核，无 swap 系统会 OOM 重启。模块化 JIT 减小单图规模，仍需设置：

```bash
export XLA_FLAGS=--xla_cpu_multi_thread_eigen=false
export JAX_NUM_THREADS=1
```

---

## 附录 A: 目标架构文件清单（含复制/新建标注）

> 🟢 = `portable_oscbf/` 有参考代码，**复制改造**；🟡 = `portable_oscbf/` 没有，**按本文档规格从零实现**（或从 `DCOLuse/` 迁入）。清单描述目标架构的文件布局。先读 §1.1 对照表确定每项的实现方式。

```
portable_oscbf/work/
├── nineaxis_manipulator_jax.py   # JAX 运动学 🟢（OBB 变换方法为目标）
├── obb_collision_model.py        # OBB 包络盒数据 (贴合 STL) 🟡 不存在，需新建
├── dpax_collision.py             # DCOL 可微碰撞内核 🟡 现位于 DCOLuse/，需迁入热路径
├── dcol_alpha_calibration.py     # DCOL alpha 逐对离线校准 🟡 不存在，需新建
├── actuator_limits.py            # 限位 🟢
├── ik_data_loader.py             # 轨迹加载 🟢
├── jax_path_following.py         # 弧长路径状态机 🟢
├── jax_posture_reference.py      # 姿态参考 🟢
├── nullspace_policy.py           # 可操作度零空间策略 🟡 不存在，需新建
├── manipulability_metric.py      # φ=½logdet(JₛJₛᵀ+εI) 及其梯度 🟡 不存在，需新建
├── oscbf_velocity_config.py      # cbfpy CBFConfig 🟢（relax_cbf=True 为目标）
├── jax_barrier_terms.py          # 障碍物 CBF 🟢
├── qp_solver_health.py           # QP 健康 🟢
├── jax_kernel_factory.py         # 控制内核编译入口 🟢（模块化 JIT 为目标）
├── jax_control_facade.py         # JaxControlLoop facade 🟢
└── safety_snapshot.py            # ESDF 快照 (原始点云感知, 可选) 🟢
```

> ⚠️ `qpax_solver.py` 未列入——其 `solve_qp_elastic` 包装类现状无调用者（legacy）；目标路线由 cbfpy `CBF.qp_solver` 直接调 qpax。

**配套**: `config/`（YAML）、`data/`（轨迹）、`urdf/`（机器人模型）、`tests/`（36 个单测文件）。

## 附录 B: 依赖清单

```
pip install jax jaxlib qpax cbfpy numpy scipy python-fcl trimesh pyyaml
# DCOL 碰撞内核: 依赖 DCOLuse/dpax 本地包 (可微碰撞), 见 DCOLuse/dpax_collision.py
# osqp 仅 legacy 参考, 非必需
```

---

> **维护规则**: 移植过程中每发现一个新的坑或调优经验，追加到移植项目的 LESSONS_LEARNED.md，保持与本项目一致的模板（现象/根因/修复/教训）。不得在本文档重新引入同一模块的第二个实现选择。
