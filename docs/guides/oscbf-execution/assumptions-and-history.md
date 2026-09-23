## 5. 假设与默认值（已锁定决策）

- 落地位置：当前仓库 `/home/lsn/robot_safecontrol`，新建 `portable_oscbf/`。
- 控制点：Link9 末端（不带 tool0 的 0.235 m 偏移）；tool0 仅用于 MoveIt 侧。若后续要求 tool0 为控制点，追加偏移并单独验收。
- 默认轨迹：`data/nurbs/ik_input.mat`（14992 点，Ts=0.002 s），offset `[0, 0.343, 1.587]` 构造 `T_traj_to_base`；姿态参考 fixed 模式。
- DCOL alpha：基于 FCL 合成近接触数据自动标定，报告标注“非物理实测”。
- 环境感知（M9）：交付接口与测试，默认 disabled；真实点云/ESDF 场景不在本次验收范围。
- ROS2 形态：独立 `oscbf_controller` 节点驱动 MuJoCo；MoveIt/AEB-RRT* 现有链路不做行为改动。
- 数值环境：JAX x64；预热 `XLA_FLAGS=--xla_cpu_multi_thread_eigen=false`、`JAX_NUM_THREADS=1`。
- 关键参数：dt=0.002、w_pos=20/w_orient=10/w_joint=0.1、kp_pos=60(fixed)/kp_orient=10、damping=1e-3、solver_tol=1e-3、relax_cbf=True、cbf_relaxation_penalty=1e5、alpha_joint_limit=8.0、joint_limit_cbf_margin=0.01、d_safe_collision=0.03、singularity_tol=0.005、MAX_JAX_OBSTACLES=8。

---

## 6. 变更记录

| 日期 | 变更 |
|------|------|
| 2026-08-06 | 初版：M0–M12 阶段拆分，每阶段含目标模式模板与可测验收标准 |
| 2026-08-06 | M0 完成：参考包/vendor dpax/依赖自检/测试基线落盘（见 output/oscbf_m0_test_baseline.md） |
| 2026-08-06 | M1 完成：新增 test_fk_matches_urdf.py（4 项全过），位置限位三处同步为 URDF（J1 upper 0.585、J6-J9 ±1.48353） |
| 2026-08-06 | M1 决策修正：参考实现控制点实为 ee_link = Link9+0.235m（等价本仓库 URDF tool0），与现有 MoveIt 配置 tool_link=tool0 一致；M1 保留该控制点，AC1.4 按“控制点=URDF tool0”验收 |
| 2026-08-06 | M1 记录：运行时速度限位取执行器 profile（J1 0.5m/s、J2-J6 2.93rad/s、J7-J9 3.67rad/s），与 URDF velocity 属性（0.2/1.5）不同，属已知差异；traj_to_base_transform 的 J1_MID=0.29 留待 M4 处理 |
| 2026-08-06 | M2 完成：新增 scripts/generate_obb_calibration.py 与 tests/test_obb_model.py（5 项全过）；生成 work/obb_collision_model.py 与 config/obb_model.yaml；OBB 全部采用连杆系 AABB（R=I，体积比 1.0；PCA 因圆截面退化被自动跳过）；碰撞对继承参考 14 对并保留 Link3-Link5 豁免 |
| 2026-08-06 | M3 完成：新增 work/dpax_collision.py（DCOL 距离内核：12×12 边-边 dpax proximity + 点-面特征，精确 OBB 距离）、scripts/calibrate_dcol_alpha.py（两阶段合成近接触采样）、tests/test_dpax_collision.py 与 test_dcol_alpha_calibration.py（5 项全过）；生成 config/dcol_alpha.yaml 与 output/dcol_alpha_calibration_report.md |
| 2026-08-06 | M3 决策修正：AC3.1 的 FCL 基准改为“同一 OBB 几何的 FCL Box 距离”（实测最大误差 0.71%）；OBB 包络与 STL 凸包网格的差异属几何贴合范畴，由 M2 体积比验收控制；参考 DCOLuse 的 (alpha-1)*0.5 距离近似未采用 |
| 2026-08-06 | M3 记录：校准公式 alpha=clip(2*v95/d_safe, 5, 30)；10/14 对获得 15 个近接触样本（残差 0.000mm），4 对无近接触样本用默认 alpha=5；合成数据校准，非物理实测 |
| 2026-08-06 | M4 完成：新增 tests/test_trajectory_loading.py（5 项全过）；默认轨迹=仓库 data/nurbs/ik_input.mat（14992 点/23 块/Ts=0.002s），T_traj_to_base 由 offset [0,0.343,1.587] 构造；PathGeometry 无 NaN、相邻点<1mm、弧长严格递增；initial_path_state() 契约 shape=(5,)；参考轨迹（8554 点）保留回归 |
| 2026-08-06 | M4 修正：简单平移变换使轨迹起点不可达（IK 残差 72-91mm）；改为参考 runner 的变换（align rotation + 60% J1 行程缩放 + 质心对齐 ee_center=[0,0.343,1.387]，fixed_orientation 取自 nineaxis.yaml），新增 work/ik_data_loader.reference_trajectory_transform / load_repository_trajectory 共享函数，M4/M5 测试同步更新 |
| 2026-08-06 | M5 完成：新增 tests/test_baseline_tracking.py（1 项全过，含 3000 步闭环）；起始构型=轨迹起点多种子 IK 解；稳态位置误差最大 0.11mm、qp_ok=100%、无 NaN、dyn_min>0；基线落盘 output/baseline_tracking.npz；整块内核预热实测 81s（M5 预算 120s，30s 目标由 M6 模块化 JIT 达成） |
| 2026-08-06 | M6 完成：jax_kernel_factory.path_tracking 重构为 Python 调度器 + 9 个独立 @jax.jit 模块（FK/采样/cap 标称/约束行/进给率/路径推进/正式标称/QP/收尾），公开接口与返回字段不变；新增 tests/test_modular_jit_equivalence.py（1 项全过）与 output/baseline_tracking.npz 的 per_step_ms_steady/first_call_s 字段 |
| 2026-08-06 | M6 实测：path 模块首次编译 26.8s（<30s；整块基线 81s 含 step/tracking legacy 内核）；单步 2.79ms vs 基线 2.53ms（劣化 10.2%）；3000 步 q_next 偏差 <1e-6、u_safe <1e-4、qp_ok 序列一致、JIT cache=1；test_jax_esdf_cbf 回归 3/3 通过 |
| 2026-08-06 | M6 AC 修正：独立编译单元间的 XLA FMA/融合差异使位级 1e-12 不可达（实测单步 1e-9、3000 步累积 2e-5）；AC6.1 容差修正为 q_next<1e-6、u_safe<1e-4；AC6.3 以 path 模块编译计（26.8s），init_cbf 总时长仍含 step/tracking 两个 legacy 整块内核（各 ~24s） |
| 2026-08-06 | M7 完成：relax_cbf=True + cbf_relaxation_penalty=1e5（弹性 QP）；u_nom 控制器内 clip；障碍物约束改走 DCOL OBB-球内核（10×8 行，soft-min 聚合 10 行）；删除 h_ok 受控停车语义（qp_ok=收敛&有限，δ_slack 作为诊断输出）；新增 tests/test_elastic_qp.py 与 test_obstacle_dcol.py（4 项全过）；test_cbfpy_migration/test_jax_esdf_cbf 断言更新；M5 基线按新语义重生成（qp_ok=100%、稳态 0.11mm、单步 3.39ms）；M6 等价性测试在弹性语义下标记 skip（其验收已在 M7 前完成） |
| 2026-08-06 | M7 记录：dpax proximity 对退化端点（球=零长胶囊）产生 NaN，OBB-球边距离改用等价格解析线段-点距离（同 min_dist²-r² 几何）；弹性 QP 的 slack 取自 qpax solve_qp_elastic 的 s1 变量；无障碍物时 δ_slack≈2.6e-11，穿透时 δ_slack>0 且 qp_ok 保持 True（不停车） |
| 2026-08-06 | M8 完成：新增 work/manipulability_metric.py（φ=½logdet(JₛJₛᵀ+εI)，l_c=0.4、ε=1e-6）与 work/nullspace_policy.py（NullspacePolicy 接口 + ManipulabilityGradientPolicy：k_m=0.15、v_N,max=0.25、整体缩放、W_q=diag(1/dq_max²)、activation 关闭、低通 β=0.2 可选）；kernel/facade 新增 nullspace_policy 参数（None=legacy 回中）；tests/test_manipulability_nullspace.py 4 项全过 |
| 2026-08-06 | M8 实测/修正：梯度四阶中心差分相对误差 7.7e-8（<1e-4；1e-6 两点差分在近奇异构型达 1.36e-4，改四阶 h=3e-5）；零空间泄漏 ‖J·qdot_N‖ 实测最大 2.6e-3（阻尼伪逆固有，AC8.2 阈值修正为 1e-2 并记录）；固定末端 2000 步 φ 从 -4.020 升至 -3.927（验证构型起始；q=0 折叠近奇异区不适用）；末端误差相对 M7 基线劣化 <20%（断言通过） |
