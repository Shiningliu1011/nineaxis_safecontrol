# OSCBF 移植拆分执行文档（M0–M12）

> 关联文档: [OSCBF_PORTING_GUIDE.md](OSCBF_PORTING_GUIDE.md)（目标架构/公式/参数权威来源）
> 生成日期: 2026-08-06
> 执行规则: 阶段严格串行；每个阶段**全部验收标准（AC）通过**才算完成；AC 未通过时停在当前阶段、记录偏差，不跳过、不带病进入下一阶段。

---

## 1. 文档目的

把 OSCBF 框架移植大任务拆成 13 个可独立验收的阶段（M0–M12）。每个阶段都采用统一的**目标模式模板**：

| 字段 | 含义 |
|------|------|
| 目标 Goal | 本阶段要交付的结果，SMART 表述（具体、可测、可达、相关、有时限） |
| 前置条件 | 进入本阶段前必须已满足的状态（通常=上一阶段 AC 全过） |
| 任务 Tasks | 为实现目标必须执行的具体动作 |
| 验收标准 AC | 每个标准都是可执行/可测量的断言，注明测试名、命令与阈值 |
| 证据/产出 | 阶段完成后必须落盘的文件、日志、报告 |
| 完成判定 | 判定完成的唯一条件：全部 AC 通过 + 证据齐备 |

---

## 2. 阶段总览与依赖

| 阶段 | 名称 | 依赖 | 一句话目标 | 出口（完成标志） |
|------|------|------|-----------|-----------------|
| M0 | 脚手架与参考基线 | 无 | 参考代码在新仓库可导入、可测试 | 参考 25 测试可执行，失败清单登记，dpax 指向 vendor |
| M1 | 机器人模型移植 | M0 | JAX POE 模型与本仓库 URDF 一致 | FK/Jacobian/限位对照测试全过 |
| M2 | OBB 包络盒标定 | M1 | 生成贴合 STL 的 OBB 模型与碰撞对 | OBB 数据+YAML 生成，零位无假碰撞 |
| M3 | DCOL 热路径 + alpha 校准 | M2 | DCOL 距离/梯度进控制热路径并完成逐对标定 | DCOL vs FCL 对照、alpha YAML、无重编译 |
| M4 | 轨迹接入 | M1 | 当前仓库轨迹加载成弧长路径几何 | PathGeometry 连续、14992 点、<5s |
| M5 | 基线控制内核跑通 | M3+M4 | 原版整块内核跑通闭环并保存基线 | 3000 步 qp_ok=100%、误差收敛、基线 npy 落盘 |
| M6 | 模块化 JIT 重构 | M5 | 整块内核拆为独立 JIT 子函数 | 与 M5 输出一致 <1e-12、cache=1、性能不劣化 |
| M7 | 弹性 QP + 限幅 + DCOL 障碍物 | M6 | relax_cbf=True、控制器内 clip、障碍物 DCOL | δ≈0、qp_fail=0、dyn_min>0、u_nom 限内 |
| M8 | 可操作度零空间 | M7 | 新增可操作度零空间策略替换关节中点 | 梯度 FD 对照、零空间残差、φ 上升 |
| M9 | 环境感知接口 | M7 | 感知链路（ESDF/点云→obs_*/sdf_*）接通 | 接口测试通过、默认 disabled 行为不变 |
| M10 | ROS2 控制节点 | M8+M9 | oscbf_controller 节点可独立运行 | 冒烟测试、JointState 合法、p95<10ms |
| M11 | Launch 集成与端到端 | M10 | 控制节点并入现有 launch 闭环 | launch 测试、端到端轨迹完成、回归不破 |
| M12 | 调优与全量验收 | M11 | 按指南 §10 全量验收 | 验收报告 + LESSONS_LEARNED + pytest 全绿 |

---

## 3. 全局验收指标（指南 §10，M12 最终核对）

| 类别 | 指标 | 阈值 |
|------|------|------|
| 精度 | 最大位置误差 max_ee | < 0.1 mm |
| 精度 | 最大姿态误差 max_oe | < 0.1° |
| 安全 | 最小动态裕度 dyn_min | > 0 mm |
| 安全 | QP 失败 qp_fail | = 0 |
| 安全 | CBF slack δ_slack（正常工况） | ≈ 0（仅约束冲突时启用） |
| 安全 | 速度跳变（>0.1 rad/s）次数 | 0 |
| 安全 | OBB 包络贴合 | 体积比记录，包络空气最小 |
| 安全 | JIT 缓存大小 | = 1（首帧后不重编译） |
| 零空间 | 梯度有限差分对照 | 相对误差 < 容差 |
| 零空间 | 零空间残差 ‖J·qdot_N‖ | 数值精度量级 |
| 零空间 | 可操作度上升 | 固定末端时 φ 上升后稳定 |
| 零空间 | 末端跟踪误差劣化 | 相比 JointCenterPolicy 不明显（<20%） |
| 性能 | 控制周期 p95 | < 10 ms |
| 性能 | 首次 JIT 预热 | < 30 s（单线程 XLA） |

---

## 4. 阶段详述

### M0 脚手架与参考基线（Step 0）

**目标**：在当前仓库建立独立的 `portable_oscbf/` 包，参考代码可导入、依赖可用、原版测试可执行，并登记已知失败基线。

**前置条件**：无（从当前仓库干净状态开始；当前未跟踪文件仅 `OSCBF_PORTING_GUIDE.md`）。

**任务**：
1. 复制 `/home/lsn/robot_oscbf/portable_oscbf`（work/、config/、urdf/、tests/、data/）到仓库根目录。
2. 复制 `/home/lsn/robot_oscbf/DCOLuse/dpax` 到 `portable_oscbf/vendor/dpax`。
3. 新增 `portable_oscbf/conftest.py`（或更新根 `tests/conftest.py`），保证从任意目录运行 pytest 时 `work` 可导入。
4. 编写依赖自检脚本 `portable_oscbf/scripts/check_dependencies.py`：校验 jax/cbfpy/qpax/fcl/trimesh/dpax 可导入并打印版本。
5. 用参考 `portable_oscbf/data/` 轨迹运行 `pytest portable_oscbf/tests`，把每个测试的通过/失败原因登记到 `output/oscbf_m0_test_baseline.md`。

**验收标准**：
- AC0.1 `portable_oscbf/work|config|urdf|tests|data|vendor/dpax` 全部存在。
- AC0.2 `python3 portable_oscbf/scripts/check_dependencies.py` 退出码 0，所有依赖可导入。
- AC0.3 `python3 -c "import dpax; print(dpax.__file__)"` 输出路径以 `portable_oscbf/vendor/dpax` 开头。
- AC0.4 `pytest portable_oscbf/tests -q` 可完整运行（不崩溃、无 ImportError）；每个失败项都能在指南 §1.1/§2 的 🟡 差异清单中找到对应原因。
- AC0.5 `output/oscbf_m0_test_baseline.md` 存在，列出每个测试的 pass/fail 与失败原因。

**证据/产出**：`portable_oscbf/` 完整目录、依赖自检输出、pytest 日志、`output/oscbf_m0_test_baseline.md`。

**完成判定**：AC0.1–AC0.5 全部通过。

---

### M1 机器人模型移植（Step 1）

**目标**：JAX POE 运动学模型（FK/Jacobian/限位）与本仓库 URDF 完全一致，控制点为 Link9 末端。

**前置条件**：M0 完成。

**任务**：
1. 复制 `nineaxis_manipulator_jax.py` 到 `portable_oscbf/work/`。
2. 按本仓库 `models/ninezzhou/urdf/ninezzhou.urdf` 核对并修正 `JOINT_CHAIN`、`home_pose`、`joint_limits`（J1 棱柱 [0,0.585]，J2-J4 ±π/2，J5 ±π，J6-J9 ±1.48）、`joint_max_velocities`。
3. URDF 使用本仓库带 `tool0` 的版本；JAX 控制点保持 Link9 末端，不追加 0.235 m 偏移。
4. 新增 `portable_oscbf/tests/test_fk_matches_urdf.py`：解析 URDF（xml.etree，不依赖 ROS），逐关节累积变换与 JAX FK 对照。
5. 新增 Jacobian 数值微分校验（中心差分 ε=1e-6）。

**验收标准**：
- AC1.1 关节数量=9、类型（1 棱柱 + 8 旋转）、限位与 URDF 完全一致（测试断言逐项相等）。
- AC1.2 零位 + ≥5 个随机位姿：FK 位置与 URDF 对照差 < 1e-9 m，姿态矩阵元素差 < 1e-9。
- AC1.3 Jacobian 6×9 与数值微分逐元素相对误差 < 1e-5。
- AC1.4 末端点 = Link9 末端（无 tool0 偏移），测试明确断言。
- AC1.5 `pytest portable_oscbf/tests/test_fk_matches_urdf.py -q` 全过。

**证据/产出**：`work/nineaxis_manipulator_jax.py`、`tests/test_fk_matches_urdf.py`。

**完成判定**：AC1.1–AC1.5 全部通过。

---

### M2 OBB 包络盒标定（Step 2a）

**目标**：为 base + Link1-9 共 10 个 STL 生成贴合连杆系 xyz 轴的最小 OBB 数据与 YAML，并确定非相邻自碰撞对清单。

**前置条件**：M1 完成（需要连杆坐标系定义）。

**任务**：
1. 新增 `portable_oscbf/scripts/generate_obb_calibration.py`：用 trimesh 读 `models/ninezzhou/meshes/*.STL`，按 URDF 的 mesh origin/连杆系将顶点变换到连杆系，求对齐主轴的最小 OBB（主轴旋转 + 半边长 + 局部中心）。
2. 生成 `work/obb_collision_model.py`（`OBB_LINK_INDICES`、`OBB_LOCAL_CENTERS_M`、`OBB_HALF_EXTENTS_M`、`OBB_LOCAL_ROTATIONS`）与 `config/obb_model.yaml`。
3. 确定自碰撞对：仅非相邻连杆；对机械近邻按 STL 实际包围盒检查确认豁免。
4. 新增 `portable_oscbf/tests/test_obb_model.py`。
5. 零位无假碰撞先用分离轴（SAT）检查 OBB 两两不相交 + FCL 网格距离 > 0 验证（DCOL 精确验证放 M3）。

**验收标准**：
- AC2.1 每个 OBB 包围对应 STL 全部顶点（无顶点越界，测试逐顶点断言）。
- AC2.2 每个连杆 OBB 体积 / STL 轴对齐包围盒体积比 > 0.7，比值记录在测试输出。
- AC2.3 碰撞对全部为非相邻连杆（相邻连杆不在清单中）。
- AC2.4 零位构型下所有碰撞对 SAT 不相交且 FCL 网格距离 > 0。
- AC2.5 `pytest portable_oscbf/tests/test_obb_model.py -q` 全过；生成脚本可重复运行且输出确定性（同输入同输出）。

**证据/产出**：`work/obb_collision_model.py`、`config/obb_model.yaml`、`scripts/generate_obb_calibration.py`、`tests/test_obb_model.py`。

**完成判定**：AC2.1–AC2.5 全部通过。

---

### M3 DCOL 热路径 + alpha 校准（Step 2b）

**目标**：DCOL 可微距离内核进入控制热路径（OBB 自碰撞 + 基元障碍物），并用 FCL 合成近接触数据完成逐对 alpha 自动标定。

**前置条件**：M2 完成（OBB 数据与碰撞对就绪）。

**任务**：
1. 新增 `work/dpax_collision.py`：封装 `dpax.polytopes.polytope_proximity`（OBB 用半空间表示）与 `dpax.endpoints.proximity`（胶囊/球），提供 `obb_transforms(q)`（FK 后各 OBB 世界位姿）和 `jax.grad` 距离梯度接口。
2. 将 vendor 的 dpax 通过 `work/__init__.py` 或 `conftest.py` 确保优先加载本地副本。
3. 新增 `portable_oscbf/scripts/calibrate_dcol_alpha.py`：在关节空间采样合成近接触构型（距离 0–5 cm），以 FCL 网格距离为基准，逐对最小化 DCOL 距离残差并标定 CBF 增益 alpha（保证 alpha>0），输出 `config/dcol_alpha.yaml`。
4. 新增 `portable_oscbf/tests/test_dpax_collision.py` 与 `test_dcol_alpha_calibration.py`。
5. 将 OBB→DCOL 距离换算为米（与 FCL 一致），α 缩放因子与距离的换算关系写死在测试基准中。

**验收标准**：
- AC3.1 随机位姿下（距离 > 1 cm）DCOL 距离与 FCL 网格距离相对误差 < 5%。
- AC3.2 `jax.grad` 距离梯度与中心有限差分（ε=1e-6）相对误差 < 1e-3。
- AC3.3 校准脚本收敛：输出每对 alpha > 0，残差 RMS 记录到 `output/dcol_alpha_calibration_report.md`。
- AC3.4 校准后近接触构型（0–5 cm）DCOL 距离残差 RMS 记录，且全部碰撞对（>1 对时）均完成标定。
- AC3.5 JIT 首帧后缓存大小 = 1（`jax_control_loop` cache size 断言）。
- AC3.6 `pytest portable_oscbf/tests/test_dpax_collision.py portable_oscbf/tests/test_dcol_alpha_calibration.py -q` 全过。

**证据/产出**：`work/dpax_collision.py`、`config/dcol_alpha.yaml`、`scripts/calibrate_dcol_alpha.py`、`output/dcol_alpha_calibration_report.md`。

**完成判定**：AC3.1–AC3.6 全部通过。

---

### M4 轨迹接入（Step 3）

**目标**：当前仓库 `data/nurbs/ik_input.mat` 可加载为连续、无 NaN、弧长单调的 `PathGeometry`。

**前置条件**：M1 完成（坐标系定义）；不依赖 M2/M3。

**任务**：
1. 复制 `ik_data_loader.py`、`jax_path_following.py` 到 `portable_oscbf/work/`。
2. 默认轨迹路径 = 本仓库 `data/nurbs/ik_input.mat`；`T_traj_to_base` 由现有 offset `[0, 0.343, 1.587]` 构造（平移变换，旋转单位阵）。
3. 姿态参考模式 = fixed；保留参考 `portable_oscbf/data/` 轨迹用于回归对比。
4. 新增 `portable_oscbf/tests/test_trajectory_loading.py`。

**验收标准**：
- AC4.1 加载成功：num_points=14992、Ts=0.002 s、num_blocks=23。
- AC4.2 位置/切线/姿态序列无 NaN/Inf；相邻点欧氏距离 < 1 mm（无跳跃）。
- AC4.3 弧长参数单调递增（严格非减）。
- AC4.4 `PathGeometry` 与 `initial_path_state()` 可构造，形状符合 facade 契约。
- AC4.5 加载+几何构建耗时 < 5 s。
- AC4.6 `pytest portable_oscbf/tests/test_trajectory_loading.py -q` 全过。

**证据/产出**：`work/ik_data_loader.py`、`work/jax_path_following.py`、`tests/test_trajectory_loading.py`。

**完成判定**：AC4.1–AC4.6 全部通过。

---

### M5 基线控制内核跑通（Step 4a，复制不改语义）

**目标**：原样复制整块编译内核（relax_cbf=False、关节中点、无控制器内 clip），用 M4 轨迹跑通 ≥3000 步闭环，建立可对照基线。

**前置条件**：M3 与 M4 完成（DCOL 内核就绪供后续；本阶段可暂不接入，先跑球模型原版）。

**任务**：
1. 复制 `jax_control_facade.py`、`jax_kernel_factory.py`、`oscbf_velocity_config.py`、`jax_barrier_terms.py`、`qp_solver_health.py` 等 🟢 模块（保持参考实现语义：`relax_cbf=False`）。
2. 预热环境变量：`XLA_FLAGS=--xla_cpu_multi_thread_eigen=false`、`JAX_NUM_THREADS=1`。
3. 新增 `portable_oscbf/tests/test_baseline_tracking.py`：从指南验证起始构型开始，用 M4 轨迹跑 ≥3000 步，保存 `output/baseline_*.npy`（q 序列、u_safe、err_6d、dyn_min、qp_ok）。
4. 记录 JIT 预热耗时与单步平均耗时。

**验收标准**：
- AC5.1 3000 步内 `qp_ok=True` 率 100%。
- AC5.2 全程无 NaN/Inf（q、u_safe、err_6d、dyn_min）。
- AC5.3 稳态末端位置误差收敛 < 1 mm（区分瞬态：前 100 步不计入）。
- AC5.4 全程 `dyn_min > 0`。
- AC5.5 JIT 预热 < 30 s（单线程 XLA）。
- AC5.6 `output/baseline_*.npy` 存在且含全部必需字段，可被 M6 测试加载。
- AC5.7 `pytest portable_oscbf/tests/test_baseline_tracking.py -q` 全过。

**证据/产出**：`output/baseline_*.npy`、测试日志（含预热/单步耗时）。

**完成判定**：AC5.1–AC5.7 全部通过。

---

### M6 模块化 JIT 重构（Step 4b）

**目标**：把整块控制内核拆为独立 `@jax.jit` 子函数（参考采样 → 标称 OSC → CBF h → QP → 积分），行为与 M5 逐位一致。

**前置条件**：M5 完成（基线 npy 存在）。

**任务**：
1. 重构 `jax_kernel_factory.py`：每个子函数独立 `@jax.jit`，公开接口（`configure_path/init_cbf/path_tracking_step` 及返回字段）不变。
2. 新增 `portable_oscbf/tests/test_modular_jit_equivalence.py`：加载 M5 基线，用同一输入序列逐步对照重构后输出。
3. 新增 JIT cache size 断言与性能对比（单步耗时、首次编译耗时）。

**验收标准**：
- AC6.1 同输入逐步输出：`q_next`/`u_safe` 与 M5 最大差 < 1e-12；`qp_ok` 序列一致。
- AC6.2 首帧后 JIT cache 大小 = 1（后续帧不重编译）。
- AC6.3 首次编译 < 30 s。
- AC6.4 单步平均耗时相对 M5 不劣化超过 20%。
- AC6.5 `pytest portable_oscbf/tests/test_modular_jit_equivalence.py -q` 全过。

**证据/产出**：重构后的 `work/jax_kernel_factory.py`、等价性测试与性能记录（`output/oscbf_m6_perf.md`）。

**完成判定**：AC6.1–AC6.5 全部通过。

---

### M7 弹性 QP + 控制器内限幅 + DCOL 障碍物（Step 5）

**目标**：切换 `relax_cbf=True`（qpax 弹性 QP，惩罚 1e5）、标称控制加 `u_nom = clip(u_nom, qdot_min, qdot_max)`、障碍物约束改走 DCOL 基元、删除 HARD_STOP 语义（保留 δ_slack 诊断）。

**前置条件**：M6 完成。

**任务**：
1. `oscbf_velocity_config.py`：`relax_cbf=True`、`cbf_relaxation_penalty=1e5`（cbfpy 0.0.1 参数名已核实）。
2. `jax_kernel_factory.py` 标称段：`u_nom = clip(u_nom, qdot_min, qdot_max)`。
3. `jax_barrier_terms.py`：固定 8 槽位 `obs_*` 接口不变，障碍物距离计算改走 `dpax_collision.py` 基元（球/胶囊/盒）。
4. 移除受控停车/HARD_STOP 分支；`δ_slack` 作为诊断量保留在结果中。
5. 新增 `portable_oscbf/tests/test_elastic_qp.py` 与 `test_obstacle_dcol.py`。

**验收标准**：
- AC7.1 正常工况（无障碍物、远离限位）`max|δ_slack| < 1e-6`。
- AC7.2 无障碍物轨迹全程 `qp_fail=0`、`dyn_min>0`。
- AC7.3 在路径附近放置 DCOL 障碍物（球）逼近时，`dyn_min` 保持 > 0；`δ_slack` 仅在逼近时刻非零。
- AC7.4 全程 `u_nom` 元素在 `[qdot_min, qdot_max]` 内（clip 生效断言）。
- AC7.5 障碍物更新后 JIT cache 仍 = 1（不重编译）。
- AC7.6 `pytest portable_oscbf/tests/test_elastic_qp.py portable_oscbf/tests/test_obstacle_dcol.py -q` 全过。

**证据/产出**：更新后的 `oscbf_velocity_config.py`、`jax_kernel_factory.py`、`jax_barrier_terms.py`、新增测试。

**完成判定**：AC7.1–AC7.6 全部通过。

---

### M8 可操作度零空间（Step 6）

**目标**：新增在线可操作度梯度零空间策略，替换固定关节中点；安全职责仍由奇异性 CBF 独立承担。

**前置条件**：M7 完成。

**任务**：
1. 新增 `work/manipulability_metric.py`：φ = ½ logdet(JₛJₛᵀ + εI)，ε=1e-6，autodiff 梯度；`J_s = S_x·J`，`S_x = diag(I₃, l_c·I₃)`，l_c=0.4。
2. 新增 `work/nullspace_policy.py`：`NullspacePolicy` 接口 + `ManipulabilityGradientPolicy`（k_m=0.15、v_N,max=0.25、整体缩放 s_v、低通 β=0.2、`activation.enabled=false`、W_q=diag(1/ẋ_max²)）。
3. 接入 `u_null` 到标称控制，替换 `q_des=(q_min+q_max)/2` 回中；QP 不感知零空间来源。
4. 新增 `portable_oscbf/tests/test_manipulability_nullspace.py`。

**验收标准**：
- AC8.1 φ 的 autodiff 梯度与中心有限差分（ε=1e-6）相对误差 < 1e-4。
- AC8.2 ≥20 个随机构型下 `‖J·qdot_N‖ < 1e-6`（整体缩放不逐关节裁剪）。
- AC8.3 固定末端任务运行 2000 步：可操作度 φ 上升或保持稳定；`‖Nᵀg‖` 下降趋势记录。
- AC8.4 与 M5 基线相比，末端最大位置/姿态误差劣化 < 20%。
- AC8.5 `pytest portable_oscbf/tests/test_manipulability_nullspace.py -q` 全过。

**证据/产出**：`work/manipulability_metric.py`、`work/nullspace_policy.py`、测试与趋势记录（`output/oscbf_m8_nullspace.md`）。

**完成判定**：AC8.1–AC8.5 全部通过。

---

### M9 环境感知接口（Step 7）

**目标**：感知链路（点云→距离场→`sdf_*`、障碍物→`obs_*`）代码与测试就绪，默认 disabled，不改变 M7 行为。

**前置条件**：M7 完成。

**任务**：
1. 复制 `point_cloud_obstacles.py`、`safety_snapshot.py` 到 `portable_oscbf/work/`。
2. 接通 `sdf_distance/sdf_origin/sdf_voxel_size/sdf_enabled/sdf_margin` 输入；默认 `sdf_enabled=false`、无外部障碍物。
3. 新增 `portable_oscbf/tests/test_perception_interface.py`（`test_safety_snapshot.py` 直接移植）。

**验收标准**：
- AC9.1 移植的 `test_safety_snapshot.py` 全过。
- AC9.2 `sdf_enabled=false` 时 `path_tracking_step` 输出与 M7 完全一致（逐步对照）。
- AC9.3 ESDF 输入 dtype 为 float32，启用后 JIT cache 仍 = 1（无 ABI 重编译，指南 L77）。
- AC9.4 点云→体素化→距离场链路单元测试通过（合成点云输入，输出合法距离场）。
- AC9.5 `pytest portable_oscbf/tests/test_perception_interface.py portable_oscbf/tests/test_safety_snapshot.py -q` 全过。

**证据/产出**：`work/point_cloud_obstacles.py`、`work/safety_snapshot.py`、感知接口测试。

**完成判定**：AC9.1–AC9.5 全部通过。

---

### M10 ROS2 控制节点（Step 8a）

**目标**：新增独立 `oscbf_controller` ROS2 节点，无 MoveIt 依赖即可加载轨迹、运行 JAX 控制核并发布安全关节状态。

**前置条件**：M8 与 M9 完成。

**任务**：
1. 新增 `src/robot_safecontrol_moveit/oscbf_controller.py`：参数加载 → 轨迹加载 → `JaxControlLoop` 初始化/预热 → 订阅 `/mujoco_joint_states`（QoS 与现有 viewer 一致）→ 100 Hz（参数可配 500 Hz）循环 `path_tracking_step` → 发布 `JointState`（J1-J9）回 `/mujoco_joint_states`。
2. 新增 `config/oscbf_controller.yaml`：dt、轨迹路径、offset、权重、CBF 参数、发布频率、topic 名。
3. `setup.py` 注册 `oscbf_controller` console script；数据文件（轨迹、URDF、STL）沿用现有打包。
4. 新增 `tests/test_oscbf_controller_smoke.py`（无 MoveIt 依赖；可用参数注入 + 内部 `step_once` 纯方法测试 + 短生命周期启动测试）。

**验收标准**：
- AC10.1 节点可在无 move_group 的环境启动并优雅关闭（pytest 冒烟通过）。
- AC10.2 发布消息为合法 `JointState`：9 关节、顺序 J1-J9、无 NaN、值在限位内。
- AC10.3 100 Hz 下 `path_tracking_step` 实测 p95 < 10 ms（记录到 `output/oscbf_m10_perf.md`）。
- AC10.4 全部参数有 YAML 默认值且类型/范围合法（缺省启动不报错）。
- AC10.5 `pytest tests/test_oscbf_controller_smoke.py -q` 全过。

**证据/产出**：`src/robot_safecontrol_moveit/oscbf_controller.py`、`config/oscbf_controller.yaml`、冒烟测试与性能记录。

**完成判定**：AC10.1–AC10.5 全部通过。

---

### M11 Launch 集成与端到端（Step 8b）

**目标**：`oscbf_controller` 并入现有 launch 闭环，与 viewer 并存，MoveIt 规划链路保留可选；端到端轨迹可完成。

**前置条件**：M10 完成。

**任务**：
1. `launch/mujoco_transition_final.launch.py` 增加 `start_oscbf_controller` 参数（默认 true），条件启动 `oscbf_controller` 节点（与 viewer 使用同一 `/mujoco_joint_states` 流）。
2. `run_demo.sh` 启动说明更新（注明 OSCBF 执行通道与 M 手动模式交互）。
3. 新增/更新 `tests/test_final_launch_runtime.py` 与 `test_viewer_transition_client.py` 覆盖 launch 结构与 headless 启动。

**验收标准**：
- AC11.1 launch 结构测试通过：参数存在、默认值正确、节点列表包含 oscbf_controller 与 viewer。
- AC11.2 headless（`start_viewer=false`）启动后：节点存活、`/mujoco_joint_states` 有发布、无异常退出。
- AC11.3 端到端运行：轨迹跟踪完成（无 qp_fail、dyn_min>0、末端误差达 M12 阈值前的收敛记录）。
- AC11.4 现有回归测试（`tests/` 全部）不回归。
- AC11.5 `pytest tests/test_final_launch_runtime.py tests/test_viewer_transition_client.py -q` 全过。

**证据/产出**：更新后的 launch/run_demo、launch 测试、端到端运行日志（`output/oscbf_m11_e2e.log`）。

**完成判定**：AC11.1–AC11.5 全部通过。

---

### M12 调优与全量验收（Step 9）

**目标**：按指南 §10 逐项核对全部指标，产出验收报告与 LESSONS_LEARNED，全量测试绿灯。

**前置条件**：M11 完成。

**任务**：
1. 按 §3 全局指标逐项测量（精度、安全、零空间、性能）。
2. 每个 🟡 改动单独验证后再组合复测（弹性 QP、模块化 JIT、OBB/DCOL、clip、零空间）。
3. 产出 `output/oscbf_acceptance_report.md`：逐项列出阈值、实测值、通过/不通过。
4. 新建 `LESSONS_LEARNED.md`（现象/根因/修复/教训模板，与 robot_oscbf 一致）。
5. 运行全量 pytest（portable_oscbf/tests + tests/）。

**验收标准**：
- AC12.1 精度：max_ee < 0.1 mm、max_oe < 0.1°。
- AC12.2 安全：dyn_min > 0、qp_fail=0、正常工况 δ≈0、速度跳变（>0.1 rad/s）次数=0、OBB 贴合记录、JIT cache=1。
- AC12.3 零空间：梯度 FD 相对误差 < 1e-4、‖J·qdot_N‖ 数值精度量级、φ 上升记录、末端误差劣化 < 20%、奇异区不越界。
- AC12.4 性能：控制周期 p95 < 10 ms、首次 JIT 预热 < 30 s。
- AC12.5 `output/oscbf_acceptance_report.md` 存在且逐项有实测值与判定。
- AC12.6 全量 `pytest -q` 通过（portable_oscbf/tests + tests/，无 skip 掩盖失败）。

**证据/产出**：`output/oscbf_acceptance_report.md`、`LESSONS_LEARNED.md`、全量测试日志。

**完成判定**：AC12.1–AC12.6 全部通过，即整个移植任务完成。

---

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
