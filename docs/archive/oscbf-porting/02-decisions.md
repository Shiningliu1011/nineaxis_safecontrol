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
