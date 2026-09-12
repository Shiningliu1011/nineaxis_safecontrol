# 弹性 QP 的实机准入检查：一个 tick 实际解什么、失败意味着什么、容差从哪来

日期：2026-09-12。服务于 GitHub 票 #18「[08B] 选定不可行/裕度/降级策略」中的准入检查决策。本文件把「实机准入检查放在哪里、容差多少」这个问题拆成可核查的事实：控制环每 tick 解的数学问题、可行/不可行的确切含义、硬 QP 与弹性 QP 各自返回什么、"安全被花掉了多少"这个信息今天存在于哪里、以及一个容差为什么不可为 0。**本文件不选方案**；三种候选各自意味着什么写在 §9，未决项写在 §10。

本文件只读不改：除本目录（`docs/planning/oscbf-reuse/research/18-elastic-qp-admission/`）与本文外，仓库内没有改动任何生产代码、配置、阈值或测试；没有安装任何包；没有运行 ROS 或硬件。

## 0. 引用约定、固定版本，以及一处路径更正

引用标记分三类，全文严格区分：

- **【代码】**＝本仓库文件或本机已安装库的确定行号，我逐行读过。
- **【文献】**＝公开论文/其官方页面，给了 DOI 或 arXiv 号。
- **（本次实测）**＝我在本机用仓库既有内核跑出来的数字，脚本与输出见 §8。

| 组件 | 本机版本/来源 |
|---|---|
| JAX / jaxlib | 0.6.2（`python3 -c "import jax; print(jax.__version__)"`） |
| cbfpy | 0.0.1（`/home/lsn/.local/lib/python3.10/site-packages/cbfpy-0.0.1.dist-info/METADATA`；作者 Daniel Morton，仓库 github.com/danielpmorton/cbfpy） |
| qpax | 0.1.4（`.../qpax-0.1.4.dist-info/METADATA`；作者 Jon Arrizabalaga；本版本带 `explicit`/`implicit` 两套后端与顶层 dispatcher） |
| 生产控制器 | `src/robot_safecontrol_moveit/oscbf_controller.py` |
| 内核 | `portable_oscbf/work/{jax_kernel_factory.py, jax_barrier_terms.py, oscbf_velocity_config.py, jax_control_facade.py}` |

**路径更正**【代码】：题目中的 `portable_oscbf/config/oscbf_controller.yaml` **不存在**。生产 profile 是 `config/oscbf_controller.yaml`（`find . -name oscbf_controller.yaml` 只返回它和 `.scratch/ticket13-review/config/oscbf_controller.yaml` 这个评审副本）。`solver_tol: 0.001` 在 `config/oscbf_controller.yaml:39`。`portable_oscbf/config/` 下放的是内核侧参数（`nineaxis.yaml`、`controller_params.yaml` 等）。

## 1. 控制环每个 tick 到底解哪个数学问题

### 1.1 决策变量与目标

系统是**速度级**模型：状态 `z = q`（9 个关节角），决策变量 `u = q̇`（9 个关节角速度）。因为 `f(z) ≡ 0`、`g(z) ≡ I`【代码：`portable_oscbf/work/oscbf_velocity_config.py:167-173`】，`ḣ = (∂h/∂q)·q̇ = L_g h · u`，于是每个安全条件都变成一条**关于 u 的线性不等式**。

每个 tick 解的是一个二次规划（QP，quadratic program：目标函数是二次的、约束是线性的优化问题）：

```
min_u  0.5 · uᵀ P u + qᵀ u           （目标：跟参考 + 很小的关节正则/时序近端项）
s.t.   G u ≤ h_row                  （43 条 CBF 行）
       u ≤ u_max,  -u ≤ -u_min      （18 条速度盒行，由 cbfpy 追加）
```

`P` 是 OSCBF 的"任务一致性"二次项、`q = -P·u_des`，都随状态变化【代码：`oscbf_velocity_config.py:175-231`，默认实现见 cbfpy `cbfs/cbf.py:265-293`】；`u_des` 是标称速度（位置/姿态反馈 + 零空间项，见 `jax_kernel_factory.py:400-425`）。`u_max = joint_max_velocities`，本机为 `[0.5, 2.932, 2.932, 2.932, 2.932, 2.932, 3.665, 3.665, 3.665]` rad/s（本次实测打印，来源 `NineaxisManipulatorJAX`）。

### 1.2 43 条安全行分别是什么

`h_2()` 返回**原始屏障值 h**（下文 h 的含义见表）；cbfpy 把它变成行上界 `h_row = α(h) + L_f h`，`G = -L_g h`，即 `ḣ ≥ -α(h)` 写成 `ḣ ≤ α(h)`（`L_f h = 0`）【代码：cbfpy `cbfs/cbf.py:313`（`G = -self.Lgh`）、`:336`（`h = self.alpha(hz) + lfh`）】。本仓库把 `α` 覆盖成 `10·h`【代码：`oscbf_velocity_config.py:341-343`】。

| # | 行区间 | 含义 | `h` 的定义 | `h` 的量纲 | 行的 `α` | 关键常量与出处 |
|---|---|---|---|---|---|---|
| 1 | 0–17 | 关节限位上/下（每关节两条） | `h = upper - q - margin`，`h = q - lower - margin` | rad | 10 | `margin = 0.01`【代码：`oscbf_velocity_config.py:260-262`；`work/joint_limit_contract.py:12`】 |
| 2 | 18–31 | 自碰撞（14 个非相邻连杆对） | `h = d_self - d_safe_collision` | m | 10 | `d_safe_collision = 0.03`（30 mm）【代码：`oscbf_velocity_config.py:265`、`:84`】 |
| 3 | 32–41 | 障碍物（10 个连杆 OBB，每杆一条聚合行） | `h = d_obb_sphere - obs_d_safe`（8 个槽软最小聚合） | m | 每槽 `obs_alpha`，生产默认 1.5 | `obs_d_safe = 0.08`（80 mm）【代码：`portable_oscbf/config/nineaxis.yaml:199`】、`cbf_alpha: 1.5`（`:198`）；行数由 `num_obb_links=10` + `aggregate_dynamic_obstacles=True` 决定【代码：`oscbf_velocity_config.py:100-110`】 |
| 4 | 42 | 奇异性 | `h = Π σ(J_pos) - 0.005` | `(m/rad)³` 量级（奇异值乘积） | 10 | 【代码：`oscbf_velocity_config.py:287-289`，`singularity_tol = 0.005`（`:86`）】 |

行数常量：`obstacle_h_start = 2·9 + 14 = 32`、`num_obstacle_constraints = 10`、`obstacle_h_stop = 42`、`esdf_h_stop = 42`（未启用 ESDF）【代码：`oscbf_velocity_config.py:101-118`】。（本次实测）内核回报的 `num_cbf = 43`、行切片 `{joint:(0,18), self_collision:(18,32), obstacle:(32,42), singularity:(42,43)}`，与代码一致。

**障碍行的 `d_safe` 与 `α` 从哪里来（三处不同来源，别混）**【代码】：

- 生产路径上它们来自感知轨道的槽位：`perception_bridge.py:68` 定义槽布局 `px,py,pz, r, vx,vy,vz, enabled, d_safe, alpha`；`:492` 取本节点的 `safety_margin` 参数、`:499-500` 把它写进槽位 8 并把槽位 9 的 `alpha` **硬编码成 1.5**（该参数的配置文件里 `safety_margin` 常见值为 `0.08`：`config/perception_runtime.yaml:75`、`portable_oscbf/config/controller_params.yaml:31`，但**具体生效值取决于运行时参数文件**，本文件未逐一核对每个 profile）；控制器在 `oscbf_controller.py:482-490` 把槽位解出 `obs_d_safe = slots[:,8]`、`obs_alpha = slots[:,9]`。`nineaxis.yaml:197-199`（`cbf_alpha: 1.5`、`d_safe: 0.08`）是与之一致的参数来源。
- 若调用方完全不提供障碍输入，`jax_control_facade.py:800-809` 的默认值是 `obs_d_safe = d_safe_collision = 0.03`、`obs_alpha = obstacle_h_baseline_alpha = 10.0`——**与感知路径的 0.08 / 1.5 不同**。
- 生产 profile 里 `enable_perception_obstacles: false`（`config/oscbf_controller.yaml:60`），此时控制器传 `obs_kwargs = None`（`oscbf_controller.py:561`），8 个槽全部 disabled，障碍行不激活（`h` 被掩码成 1e3 再软最小），因此这两组常量在当前部署下不起作用。

本文件的隔离实验取 `obs_d_safe = 0.08`、`obs_alpha = 1.5`（感知路径的口径），并在 §6.3 说明这直接影响 `slack → h` 的换算系数。

`h` 的三种量纲（rad / m / 奇异值乘积）混在同一个向量里，这一点在 §6 会重新出现。

### 1.3 内核回报的 `h_vals` 就是 `h`

`h_vals = h_qp[:num_cbf] / obstacle_h_baseline_alpha`【代码：`jax_kernel_factory.py:200`】。因为 `f ≡ 0`，`u=0` 时 `h_qp = α(h) = 10·h`，所以除以 10 之后 `h_vals` 恰好等于原始 `h`（本次实测：同一状态下 kernel 回报的 `h_vals` 与直接调用 `CBF.h_2(...)` 得到的分组最小值一致到打印精度——自碰撞组 `0.162999`、障碍组 `999.979206`、奇异组 `1.1042`）。这就是 §5 里 `h_ok = all(h_vals >= -1e-3)` 的含义：**"每条屏障值都不低于 -1 mm / -0.001 rad"**。

（本次实测）生产风格的起点 `q = 0` 上，四组行的最小值：

```
joint = -0.010000   self_collision = +0.162999   obstacle = +999.979206   singularity = +1.1042
```

`joint = -0.010000` 不是噪声：J1 的机械下界是 0、`margin = 0.01`，所以 `q=0` 时 J1 正好压在关节限位 CBF 的 10 mm 边界上，`h = 0 - 0 - 0.01 = -0.01`。障碍行的 `999.979` 是"未启用槽被掩码成 1e3 再软最小"的产物（`aggregate_dynamic_obstacle_terms`，`jax_barrier_terms.py:84-93`），不参与 min。

## 2. "可行"与"不可行"到底指什么

**可行**＝存在**至少一个**关节速度 `u`，同时满足全部 43 条安全行和速度盒。**不可行**＝这样的 `u` 不存在。注意可行性与"轨迹好不好"无关：可行只保证这一 tick 的安全下降条件能被满足。

### 2.1 一个必须先纠正的直觉：`h < 0` 本身不是不可行

题面把"测到的状态已经在裕度以内（`h<0`，一个 tick 内没有速度能修好）"列为不可行的来源。本仓库的实际公式不支持这个说法，必须说清楚：

CBF 条件是 `ḣ ≥ -α·h`。当 `h = -0.01`、`α = 10` 时，行要求的是 `ḣ ≥ +0.1`（单位：rad/s 或 m/s），也就是**"朝安全侧退"**，而不是"瞬时把 h 抬回 0"。所以 `h<0` 只说明这一 tick 的下降条件要求的运动方向是朝安全侧，并不意味着无解——只要"退"这个方向的速度在执行器与其它行允许范围内，QP 依然可以**零松弛**地解出来。恢复所需时间也被这条不等式锁住：`h` 的回升速率不能快于 `α|h|`，量级上是 `1/α` 秒（`α=10` 时约 0.1 s）。

（本次实测）场景 A 正好把这条不等式逐 tick 验了一遍：J1 的行在 `q=0` 时 `h=-0.010`，其后该行**恰好保持紧约束**（`ḣ = α|h|`），于是 `h_{k+1} = h_k·(1 - α·dt) = 0.9·h_k`——实测序列 -0.0100 → -0.0090 → -0.0081 → -0.0073 → -0.0066 → -0.0059 …… 严格符合 0.9 的几何衰减，到 A+018 转正（约 0.18 s）。这既是"负 h 可以零松弛恢复"的证据，也顺带证明整条 `h → α(h) → 行上界 → QP` 链路是自洽的。

（本次实测，两组独立证据）

- 场景 A（无任何障碍物，60 tick，生产参数）：`qp_ok` 全为 True，`delta_slack` 最大值 **1.87e-9**，而 `min h` 全程 **-0.01**（即 J1 的负 h 行一直存在）。`primal_residual` 全程 0.0。
- 场景 B：把 1 mm 半径的球放在机械臂碰撞球外侧，沿射线滑动到实测障碍裕度 **-5.015 mm**（也就是已经花掉 80 mm 障碍裕度中的 5 mm）。连续 5 tick 的 `delta_slack` ≤ 4.7e-11，且障碍行 `h` 单调回升：-5.015 → -4.704 → -4.423 → -4.061 → -3.318 mm。

**结论（实测 + 代码）**：在本系统里"某条 h 为负"是**常态**（`q=0` 就是），不是异常。真正需要区分的是"退得动的负 h"和"退不动的负 h"。这道区分恰恰是票 #18 的核心。

### 2.2 本系统中不可行的四条真实来源（逐条给机制）

**(a) 某行的控制权威为零 —— 结构性不可行。** 行的可达变化率是 `ḣ = -G u`；如果 `G` 的第 i 行整行为 0（记 `|L_g h|₁ = 0`），那么第 i 行的 `ḣ` **完全不受任何 u 影响**。此时只要它的行上界 `α·h + L_f h` 为负，该行就无解，与"执行器有多大、参考是什么"无关。
（本次实测，场景 G）把障碍球放在末端中心：10 条障碍行里有 **2 行** `|L_g h|₁ = 0`（`rows_with_zero_lgh = 2`，`obstacle_rows_lgh_l1 = [0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0]`）。这两行对应"球心落在 OBB 内部、最近特征退化"的连杆。**这条是本次实验里唯一能让弹性 QP 花掉松弛的机制。**

**(b) 单行与速度盒冲突。** 行要求 `ḣ ≥ α·|h|`，但把这个方向的速度投影到速度盒（以及它与其它行的夹角）后达不到。
（本次实测，场景 E）把 J1 放到低于下界的位置，`h_j1 = -(overshoot + 0.01)`，需求退速 `= 10·|h|`：

| 下界以下过冲 | J1 的 `h` | 需求退速 | J1 上限 | 实际 `u_J1` | `delta_slack` | `qp_ok` |
|---|---|---|---|---|---|---|
| 0.000 rad | -0.010 | 0.10 rad/s | 0.5 | 0.10000017 | 1.98e-10 | True |
| 0.005 rad | -0.015 | 0.15 | 0.5 | 0.15000016 | 2.05e-10 | True |
| 0.020 rad | -0.030 | 0.30 | 0.5 | 0.30000667 | 2.93e-09 | True |
| 0.050 rad | -0.060 | 0.60 | 0.5 | 0.50090216 | **0.09910** | True |
| 0.200 rad | -0.210 | 2.10 | 0.5 | 0.50105106 | **1.59895** | True |

松弛正好出现在"需求 ≥ 可达"的临界点之后，且 `delta_slack ≈ 需求 - 可达`（0.6-0.5=0.1；2.1-0.5=1.6），误差来自 10 条障碍行与其它行同时参与。注意 `u_J1` 被速度盒逼到 0.5009（略超 0.5 的项由 solver_tol 内的松弛承担），`q_next_J1 = 0.0`（被机械限位 clip）。

**(c) 两行互相要求相反的位形运动。** 本仓库的 43 行里没有内建的反向对，但**障碍物行之间**可以：两个障碍分居两侧时，两条行会要求相反的 `ḣ` 方向。本次**未构造**这一场景（见 §11），不把推论当结论。

**(d) 参考把 `u` 拉向屏障。** 这项只改变"花多少控制努力"，不直接造成不可行：参考与屏障的冲突表现为屏障行变成 active（`active_count`）以及退速被拉满，而不是无解。要变成不可行仍要走 (a)(b)(c) 的机制。

**(e) 传感器噪声让 h 跳变。** 噪声落到 h 上就是行上界的抖动，会提高 (a)–(d) 的发生概率。本次**未注入噪声**，不给数字（见 §11）。

### 2.3 一条与安全政策直接相关的代码事实：h 在 0 处饱和

两个距离核都在 0 处截断，因此**"负 h 的绝对值"并不等于穿透深度**：

- 障碍：`_obb_sphere_pair_distance` 返回 `max(min(edge_surface, face_surface), 0.0)`【代码：`portable_oscbf/work/dpax_collision.py:314`】，因此 `h_obs ≥ -obs_d_safe = -0.08`。本次实测：把半径 0.001 / 0.05 / 0.30 / 0.80 m 的球心放在末端中心，`h_obs` 一律 **-0.080000**。
- 自碰撞：`_obb_pair_distance_impl` 的文档串写明是 "Exact distance between two disjoint OBBs"【代码：`dpax_collision.py:168-170`】。本次实测（核函数层面）：取同一连杆的变换，把第二个盒沿 x 平移 0.0 / 0.001 / 0.01 / 0.05 m（该盒 x 半长 0.075 m，故这些偏移都是**深重叠**）→ 返回 **0.000000**；偏移 0.2 m（已分离 0.05 m）→ 返回 0.049997。也就是说重叠态下它给出 0 而不是负穿透深度。

**推论**：`min h` 与 `slack` 都**不能**回答"实际进去了多少"。任何以 `min h` 或 `max slack` 为输入的判定门槛，能分辨的是"有没有被顶住"，分辨不了"顶得多深"。这与既有几何研究对 FCL 符号距离的告警方向一致（`docs/planning/oscbf-reuse/research/obb-environment-geometry.md` 关于不相交态以外不得把返回值解释为穿透深度一节）。

## 3. 硬 QP（无 slack）：它做什么，返回什么，为什么在实践里别扭

**调用路径**【代码】：`relax_cbf=False` 时 cbfpy 把求解器选成 `jax.jit(qpax.solve_qp)`【`cbfpy/cbfs/cbf.py:105-108`】，签名 `(P, q, A, b, G, h, solver_tol=...)`（本仓库在 rate-limit 分支里直接调用 `qpax.solve_qp`，见 `jax_kernel_factory.py:325-327`）。

**它返回什么**【代码：`qpax/explicit/pdip.py:164-196`】：`(x, s, z, y, converged, pdip_iter)`，其中 `x` 是最优（或最后一个）迭代点、`s` 是行松弛 `Gx + s = h`、`z` 是行对偶、`y` 是等式对偶、`converged` 是整数标志。

**`converged` 的确切含义**【代码】：

```
converged = jnp.where(jnp.linalg.norm(kkt_res, ord=jnp.inf) < nonlocal_tol, 1, 0)
```
（`qpax/explicit/pdip.py:249-250`），循环条件为 `pdip_iter < max_iter 且 converged == 0`（`:295-296`），`max_iter` 默认 **30**、`solver_tol` 默认 1e-5（`SolverParams`，`:48-50`）。

所以 `converged = 0` 的含义是**"在 max_iter 步内 KKT 残差的 ∞-范数没有降到 solver_tol 以下"**，而不是"可行集是空的"。内点法（interior-point method：沿对数障碍把迭代点推入可行域内部、逐步降低障碍权重的方法）在这里**不输出任何不可行证明**（既没有 Farkas/对偶不可行证书，也没有可行集为空的判定）；它只在容差意义上报告"没收敛"。这是内点法停止准则的一般性质（背景参考：Nocedal & Wright, *Numerical Optimization*, 2nd ed., 第 17 章；本文件不对该书做逐句引用）。

（本次实测，场景 D）构造一个**可证明不可行**的一维 QP：`min 0.5u²` s.t. `u ≥ 1` 且 `u ≤ -1`，即 `G = [[1],[-1]]`、`h = [-1,-1]`。硬 QP 的结果：

```
converged = 0        pdip_iter = 30（= max_iter）
x = [0.0]            G·x - h = [1.0, 1.0]      （违反两条行各 1.0）
s（行松弛）= [1.49e-10, 1.49e-10]              （数值上无意义）
z（对偶）  = [5.94e+215, 5.94e+215]            （已发散/溢出）
```

也就是说：**硬 QP 失败时给出的 `x` 完全不能下发**，返回的 `s`、`z` 也不携带可解释的物理信息，调用者唯一能用的信息是 `converged` 标志与迭代计数。

**调用者剩下要决定的**：cbfpy 自己把这件事写在文档串里（见 §4 引文）——"若要严格强制 CBF，应由更高层控制器处理 QP 不可行的情况"。本仓库今天的选择是"不用硬 QP"：`relax_cbf=True`【代码：`oscbf_velocity_config.py:161`】。

## 4. 弹性/松弛 QP（加 slack）：它做了什么

**调用路径**【代码】：`relax_cbf=True` → `qp_solver = jax.jit(qpax.solve_qp_elastic)`【`cbfpy/cbfs/cbf.py:105-108`】；本仓库在无 rate-limit 时走这条分支并取 8 元组返回【`jax_kernel_factory.py:312-323`】。罚系数是 `cbf_relaxation_penalty = 1e5`【`oscbf_velocity_config.py:162`】。

**它解的到底是什么问题**（从 qpax 的 KKT 残差反读，这是最直接的证据）【代码：`qpax/explicit/elastic_qp.py:36-53`（`ElasticQPData` 与初始化）、`:166-171`（`_step` 里的残差）】：

```
min_{u,t}  0.5 uᵀQu + qᵀu + penalty · Σ_i t_i
s.t.       G u - t ≤ h,   t ≥ 0
```

残差 `r2 = -z1 - z2 + penalty·1`（`:167`）、`r5 = -t + s1`（`:170`）、`r6 = Gx - t + s2 - h`（`:171`）说明：`t` 是**逐行**的弹性变量，`s1 = t`（互补配对），`s2 = h - Gx + t` 是该行自己的松弛。所以 `delta_slack = max(s1)`【`jax_kernel_factory.py:317`】就是 **`max_i t_i`**。本次实测（场景 D 的弹性版）校验了这一点：`t = 1.0`，`s1 = 1.00000000062`（两者相等到 solver_tol 内），`s2 ≈ 6.2e-10`。

**它为什么"永远有解"**：对任意 `u`，取 `t_i = max(0, (G u - h)_i)` 就能让所有行成立，而 `t ≥ 0` 只有下界，所以可行集非空。这就是 cbfpy 文档串的原话（**引文逐字**，`cbfpy/config/cbf_config.py:28-36`）：

> 「Depending on the construction of the barrier functions and if control limits are provided, the CBF QP may not always be feasible. If allowing for relaxation in the CBFConfig, a slack variable will be introduced to ensure that the problem is always feasible, with a high penalty on any infeasibility. This is generally useful for controller robustness, but means that safety is not guaranteed.」
> 「If strict enforcement of the CBF is desired, your higest-level controller should handle the case where the QP is infeasible.」

【文献】这条机制在文献里有标准的同型表述，两处可核验的原文：

- Ames, Coogan, Egerstedt, Notomista, Sreenath, Tabuada, *Control Barrier Functions: Theory and Applications*（ECC 2019, pp. 3420–3431；DOI [10.23919/ECC.2019.8796030](https://doi.org/10.23919/ECC.2019.8796030)；arXiv [1903.11199](https://arxiv.org/abs/1903.11199)）。在该文 §II-C（CLF-CBF-QP）中逐字出现：「**δ is a relaxation variable that ensures solvability of the QP**」；§V-D 用「introducing the relaxation variable δ」描述同一手法。请注意这句话的两个含义：加松弛变量是**为了保住可解性**，而不是为了证明安全性。
- *Safety-critical Control with Control Barrier Functions: A Hierarchical Optimization Framework*, arXiv [2410.15877](https://arxiv.org/abs/2410.15877)：把 CLF 约束松弛为 `L_f V + L_g V u + λV ≤ δ`，并逐字写道「**the safety of the system is no longer strictly guaranteed** after (11c) is relaxed into (12c)」。
- 关于"`h` 为负 / 状态在裕度内**不等于**不可行"这一反向事实，还可对照 Wang, Ames, Egerstedt, *Safety Barrier Certificates for Collisions-Free Multirobot Systems*（IEEE T-RO 33(3), 2017, pp. 661–674；DOI [10.1109/TRO.2017.2659727](https://doi.org/10.1109/TRO.2017.2659727)）：该文把 QP 的可行性问题交给**较高层的紧急制动动作**来保证，即"安全证书 QP 的可行性不是自动的，需应用层安排"。

**代价（机制层）**：`penalty · Σt` 是**线性**罚，且 `t` 是连续量，所以"买多少松弛"是 QP 在 `t` 的价格与偏离 `u_des` 的代价之间做的比较；`penalty` 越大越倾向于不买。这是"松弛是一种被定价的放弃"，不是"松弛只反映数值误差"。

（本次实测，场景 G）但**调大罚因子不能解除结构性不可行**：把 `penalty` 从 1e3、1e5、1e7 改到 1e9（每次显式清 JAX 编译缓存以强制重跟踪），同一状态的 `delta_slack` 全部为 **0.12000000614759472**、`primal_residual` 全部 0.12、`|u|` 与 `q_next` 逐位相同。原因是 §2.2(a) 的两条行 `|L_g h|₁ = 0`：松弛量被结构锁死在 `-α·h = -1.5 × (-0.08) = 0.12`，与罚系数无关。这条对"用更高罚系数换取安全"的思路是直接的反例。

（本次实测，场景 D）弹性版在同一不可行 QP 上：7 次迭代、`converged = 1`、`x = 0`、`t = s1 = 1.0`（正好是所需的最小松弛）、`G·x - h = [1,1]`。

## 5. 陷阱：同一个机制把两件不同的事报成一件事

对偶来看，"弹性 QP 永远返回一个解且 `converged = 1`"这个良好性质，恰好抹掉了两种情况之间的区别：

1. **安全约束真冲突**——机器人在当前位形/参考下没法既安全又跟路径；
2. **纯数值不顺**——内点法在容差内的正常抖动。

两者在 `qp_ok` 上表现相同（都是 `True`），区别只剩在 `delta_slack` 的数值上。而今天的生产路径只对 `qp_ok` 做门控。逐条核验：

### 5.1 生产模式的门到底是什么

`jax_kernel_factory.py:341-351`（**引文逐字**，注释原文在 `:343-345`）：

```python
        u_finite = jnp.all(jnp.isfinite(u_candidate))
        if cbf.relax_cbf and not enable_rate_limit:
            # Elastic QP (M7): a negative barrier is softened by the slack
            # instead of triggering a controlled stop; delta_slack is the
            # diagnostic.  Only convergence and finiteness gate the command.
            qp_ok = u_finite & solver_accepted
        else:
            # Rate-limited mode keeps the documented hard-CBF gate: the two
            # shared slacks relax only u - u_safe_prev, never the barrier.
            h_ok = jnp.all(h_vals >= -1e-3)
            qp_ok = u_finite & h_ok & solver_accepted
```

注意 `h_ok = all(h_vals >= -1e-3)` 这一行**只在 `enable_rate_limit=True` 时可及**。

（本次实测 + 代码核对）生产节点**从不**打开 rate limit：

- `oscbf_controller.py:406-417` 构造 `JaxControlLoop` 时传的参数是 `dt, dt_path, w_pos, w_orient, w_joint, temporal_lambda, enable_x64, solver_tol, task_mode, nullspace_policy`——**没有 `rate_limit_du_max`**。
- `grep -rn "rate_limit" src/` 只命中 `feedrate_rate_limit_m_s`（一个诊断字段）与 `hardware_contract.py` 的命令速率监察，没有任何 CBF 速率约束的注入。
- `jax_control_facade.py:145-147`：`enable_rate_limit = (rate_limit_du_max is not None and any(>0))` → 生产恒为 `False`。
- 因此生产永远走 `:342-346` 分支，`qp_ok = isfinite(u) & converged`。

**结论**：今天用于放行的"健康门"里**不包含任何关于屏障值或松弛量的判据**。`h_vals` 那条 -1e-3 的检查是仓库里既有的、可用的同类判据先例，但它在生产模式下不可达。

### 5.2 `delta_slack` 今天去了哪里

- 内核：`delta_slack = jnp.max(s1_qp)`【代码：`jax_kernel_factory.py:317`】。
- facade：写进 `JaxPathTrackingResult.delta_slack`【`jax_control_facade.py:687`】与 `last_delta_slack`（`:454`/`:570`）、`last_path_metrics['path_delta_slack']`（`:657`）。
- 评价器：`tracking_evaluator.py:33` 定义字段、`:50`/`:64` 收集、`:109-110` 聚合成 `max_delta_slack` / `mean_delta_slack`、`:250`/`:299-300` 只做 `np.max`/`np.mean`、`:162` 在 Markdown 报告里打一行「`- CBF 最大松弛: {self.max_delta_slack:.2e}`」。
- 控制器遥测：`oscbf_controller.py:787` 每秒打一行 `slack={snapshot['delta_slack']:.2e}`。

**除"统计与打印"之外没有任何消费者**：没有阈值、没有冻结、没有告警、没有进入 `qp_ok`。

### 5.3 `qp_ok` 为假时会发生什么

【代码】`oscbf_controller.py:675-682`：`qp_ok` 为假时 `_qp_fail_count += 1` 并打一条 `warn`，文案是 "QP failed at step N; holding current state"，**但紧接着仍然调用 `_publish_positions(q_next)`**。而 `q_next` 来自 `apply_qp_health_gate`【`jax_barrier_terms.py:111-115`】：

```python
u_applied = jnp.where(qp_ok, u_candidate, jnp.zeros_like(u_candidate))
q_next = jnp.clip(q + u_applied * dt, q_min, q_max)
```

即 `qp_ok=False` 时这一 tick 的 `u = 0`、`q_next = clip(q, q_min, q_max)`（位形本就在限位内时即 `q` 本身）。但：

- 这是**单 tick** 的置零，不是锁存状态；下一 tick 若 `converged` 又为真就继续动。
- 所有下发路径（`oscbf_controller.py:551` hold、`:626` stall、`:654` stalled、`:664` endpoint、`:682` 正常）都经过 `_publish_positions`，而它在 `:684-700` 施加一个**硬编码 `τ = 0.02 s` 的一阶低通**（`alpha = dt / (dt + 0.02)`）。因此被"冻结"的那一 tick 也只是把低通目标设为当前实测位姿。

**含义**：现状的"冻结"是"把一个 tick 的目标速度置零再被低通平滑一次"，与选项 A 里说的 `freezes/latches`（持续冻结或锁存到人工复位）**不是同一件事**。如果票 #18 要的是后者，那是新增行为，不是既有行为。

### 5.4 用今天的代码就把陷阱显出来

（本次实测，场景 C）把半径 0.02 m 的球放在末端中心（使障碍行 `h = -0.080`，即 80 mm 障碍裕度被整条花掉），得到的 8 个 tick：

| tick | `min h`（障碍组） | `delta_slack`（行单位） | `primal_residual` | `qp_ok` | `|u|` (rad/s) | `|Δq|` (rad/tick) |
|---|---|---|---|---|---|---|
| C+000 | -0.080 | 0.120000 | 0.120 | **True** | 0.106 | 0.001059 |
| C+003 | -0.080 | 0.120000 | 0.120 | **True** | 0.282 | 0.002824 |
| C+005 | -0.080 | 0.120000 | 0.120 | **True** | 2.495 | 0.024946 |
| C+007 | -0.080 | 0.120000 | 0.120 | **True** | 5.686 | 0.056856 |

最后一 tick 的 `u_safe`（rad/s，前 6 个关节）`[0.0478, **-2.9321**, 2.5261, **2.9320**, -0.0007, -2.9314]`——关节 2、4 正好顶在自己的速度上限 `2.932`，关节 6 顶在 `2.931`；`q_next`（前 6 项）`[0.00570, -0.07871, 0.06546, 0.07815, -0.00002, -0.06582]`。

也就是说：**一条安全行被顶在 -80 mm、QP 顶掉 0.12 的松弛、速度盒多轴饱和，而 `qp_ok` 报 True，控制器照常下发、机械臂照常以最高 5.69 rad/s 的速度运动。** `delta_slack = 0.12` 与 `primal_residual = 0.12` 在数值上相等并非巧合：`primal_residual = max(max(G·u* - h_row), 0)`【代码：`jax_kernel_factory.py:354-356`】，它就是"求解出来的 `u` 实际违反行的量"，也等于该行付出的松弛。

同时（本次实测，场景 A）在干净的 60 个 tick 上，`delta_slack` 的量级是 **1e-10 ~ 1.9e-9**。两种情形相差 8 个数量级，但 `qp_ok` 都是 `True`。

### 5.5 一个已经存在但没被用于判定的量

`primal_residual`（`:356`）今天被写进诊断（`jax_control_facade.py:705` → `oscbf_controller.py:519-520` 的 `qp_primal_residual`），**没有参与任何门控**。它在物理语义上就是"这条路要求的准入度量"，且不需要额外求解就能得到。这一点在 §9 的三种方案里都对得上，但本文件不据此推荐任何方案。

## 6. "准入检查"具体是什么，以及为什么容差不可避免

### 6.1 定义

按选项 A 的字面写法，准入检查（admission check）＝求解之后，取该 tick 的松弛统计量并与阈值比较：

```
max_i t_i  ≤  eps        （等价地：重建的 min_i h_i  ≥  -eps_h ）
不满足 → 该 tick 的命令不被采信（冻结/降级/锁存，具体动作属政策）
```

题面把它写成 `max(slack) <= eps` 与 `min h >= -eps` 是同一件事。**在本仓库里这两者不是同一件事**，见 §6.3。

### 6.2 为什么 `eps` 不能是 0

三条独立原因：

1. **内点法不会给出精确 0。** `t ≥ 0` 是通过互补条件以容差收敛的；行未激活时 `t` 仍可能残留一点点正值。（本次实测）场景 A 的 60 个干净 tick 上 `delta_slack` 在 **9.0e-12 ~ 1.87e-9** 之间；场景 B（`h` 为负但可行）在 **2.2e-12 ~ 4.7e-11**。因此 `eps = 0` 会让几乎每个 tick 都被判为不可准入。
2. **h 的重建本身有浮点误差。** 判定所用的 `h_vals` 是 `h_qp/10`，`h_qp` 由 `h_2` 的 JVP 构造【`jax_kernel_factory.py:198-200`】，距离核内部还有 `max(..., 0)` 与软最小（`jax_barrier_terms.py:88-93`），都是舍入误差的来源。
3. **`solver_tol` 不能直接当这个 `eps`。** `solver_tol = 0.001` 的定义是"KKT 残差的 ∞-范数阈值"【代码：`qpax/explicit/elastic_qp.py:174-176`、`qpax/explicit/pdip.py:249-250`，`SolverParams(tol, max_iter)` 在 `:48-50`】。这个残差向量把**梯度行（`Q x + q + Gᵀz`，量纲是"目标函数的梯度"）、互补行（`s·z`，量纲是"松弛×对偶"）、原始残差（`Gx - t + s2 - h`，量纲是行上界）**拼在一起取 ∞-范数，是一个无量纲化的混合量，**不是**对 `max t` 的上界，也不是对 `min h` 的上界。所以"因为 solver_tol 是 1e-3，所以 eps 也取 1e-3"是**没有依据的**。

`solver_tol` 的来源链（题面要求核验）：`config/oscbf_controller.yaml:39` `solver_tol: 0.001` → 被 `production_config.py:38` 列为必填数值项、并在 `:274-275` 校验 `0 < solver_tol < 1` → `oscbf_controller.py:414` 传给 `JaxControlLoop(solver_tol=...)` → `jax_control_facade.py:133`、`:318` → `oscbf_velocity_config.py:163` → cbfpy `cbfs/cbf.py:104` → 每次求解作为 `solver_tol=` 传入 `qpax.solve_qp_elastic`。

### 6.3 `eps` 的量纲：slack 不是"米"

这是本文件最需要传达的一条技术事实。

行约束的形式是 `ḣ_i ≤ h_row_i`，其中 `h_row_i = α_i·h_i + (L_f h_i + 时间项)`【代码：`cbfpy/cbfs/cbf.py:336`；动态障碍的 α 与时间项在此之上打补丁，`portable_oscbf/work/jax_barrier_terms.py:70-108`；内核在 `jax_kernel_factory.py:200-223` 应用】。弹性松弛 `t_i` 是**加到行上界上**的：

```
ḣ_i ≤ h_row_i + t_i        ⟹  t_i 与 h_row_i 同量纲 = ḣ_i 的量纲
```

所以：

- 关节限位行（`α = 10`）：`t` 的单位是 **rad/s**；
- 自碰撞/障碍行（`α = 10` / `1.5`）：`t` 的单位是 **m/s**；
- 奇异行：单位随 `h` 的乘子量纲。

把 `t_i` 换算成"这条行容许多花多少 h"要**除以 `α_i`**：`Δh_i = t_i / α_i`。（本次实测校验）场景 E：`t = 1.59895` rad/s、`α = 10` → `Δh = 0.1599` rad，而 `h - 可达速度/α = 0.21 - 0.05 = 0.16` ✓。场景 C：`t = 0.12` m/s、`α = 1.5` → `Δh = 0.08` m = 恰好 80 mm 障碍裕度 ✓。

因此：

- **"`eps` = 1 mm ⇒ 静默允许花掉 1 mm 安全裕度"这句话只有在 `eps` 作用在 `t_i/α_i`（或等价地作用在 h 上）时才成立。** 直接比较 `max_i t_i` 与一个以米为直觉的 `eps`，会把 m/s 与 rad/s 混为一谈，并且被 `α ∈ {10, 1.5}` 缩放（两者相差 6.7 倍）。
- 1 mm 相对于本仓库当前的裕度：自碰撞 `d_safe_collision = 30 mm` 的 **3.3%**，障碍 `d_safe = 80 mm` 的 **1.25%**，关节限位 `margin = 10 mm` 的 **10%**。
- 若改用 `min h` 直接判定，`eps_h` 的三种行同时存在 rad / m / 奇异值乘积三种量纲，"一个 eps 管所有行"本身就需要额外定义（例如按行组各给一个）。

### 6.4 容差的两侧代价（含可用的量级锚点）

- **太松**：真实的裕度支出被放行——包括"有障碍物但控制器决定放弃 80 mm 中的一部分"这种情况。
- **太紧**：冻结被数值噪声触发。（本次实测）以 `max(slack) ≤ eps` 作为后验门，在场景 A 的 60 个干净 tick 上：`eps = 1e-9` → **冻 1 个 tick**；`eps ≥ 1e-6` → 0 个；而场景 C 的 8 个 tick 在 `eps ∈ {1e-9, 1e-6, 1e-3, 1e-2}` 下**全部**被判冻结。也就是噪声地板的最大值（1.87e-9）落在 1e-9 与 1e-6 之间，`eps` 必须显著大于 ~2e-9 才不会在干净工况误触发；1e-6 在本次样本里已经足够。

**但必须标注**：这个"噪声地板"是**一台机器、一次 60 tick 采样**（x64、CPU、`OPENBLAS_NUM_THREADS=1`、`OMP_NUM_THREADS=1`、无感知噪声注入）的最大值样本，**不是上界**。样本最大值不能当硬上界用。

### 6.5 一条与"能分辨多少"有关的结构限制

由 §2.3，`min h` 与 `t` 都不携带穿透深度信息（两个距离核都在 0 处饱和）。因此任何以它们为输入的准入检查，其能力上限是"发现被顶住了"，而不是"发现顶进去多少"。如果政策上需要"最多允许 N mm 越界"，仅凭今天的几何内核无法直接度量——这一点在 §9 的"需要额外测什么"里再次出现。

## 7. 选项 C 多做了什么

门（A/B）看到的对象是**求解出来的速度 `u*`**（可能被 `primal_residual` 复检）；而机器人**实际执行**的是积分并 clip 之后的下一个位形：

```python
q_next = clip(q + u_applied · dt, q_min, q_max)     # jax_barrier_terms.py:111-115
```

两处之间至少有三段缝隙（都【代码】可核验）：

1. **积分**：行条件保证的是 `ḣ ≥ -αh`（连续时间的下降率），而实际前进一步是 `q + u·dt`；一个 tick 内 `ḣ` 会被 `dt` 放大/缩小（本生产 profile `dt = dt_path = 0.01`，`config/oscbf_controller.yaml:9-12`）。
2. **clip**：`q_next` 被夹在 `q_min/q_max`（生产未传 `joint_limit_lower/upper`，故等于机械限位，`jax_control_facade.py:156-178`）。clip 可能让实际到达的位形**偏离**行条件所假设的那一步——也就是说，被 clip 之后的行值不再等于门检查时假设的行值。
3. **控制器外侧还有两级**：`τ = 0.02 s` 的一阶低通（`oscbf_controller.py:684-700`）与下游位置环。低通意味着下发的位置序列不等于 `q_next` 序列。

因此"在 `q_next` 处重算所有行"检查的是**另一个物理命题**：不是"这一 tick 我是否违反了 CBF 的下降条件"，而是"积分并 clip 之后实际到达的状态，其安全裕度是多少"。这与 A/B 是**互补**关系，不是替代关系：A/B 检查的是求解结果的自洽性，C 检查的是一步动力学之后的结果。

（本次实测）这条缝在本例中很小但同时可见：场景 C 最后一个 tick `|u| = 5.686 rad/s`、`|Δq| = 0.056856 rad`，`u·dt` 与 `Δq` 一致（`dt=0.01`），`q_next` 未触机械限位；场景 E 的 0.2 rad 过冲那一步 `q_next_J1 = 0.0` 是**被 clip 到机械限位**的结果（`q + u·dt = -0.2 + 0.005 = -0.195`，被夹到 0.0），也就是这一步的实际到达位形与积分预测不一致。这正说明"看 `u*`"与"看待执行位形"是两件不同的事。

（未验证）选项 C 还需要重算 43 行的几何。本文件**没有测**一次全行重算的耗时（CPU 端 `h_2` + `L_g h` 的代价），因此不对"能否在 100 Hz 内做完"给结论（见 §11）。

## 8. 隔离实验

### 8.1 产物与运行方式

| 文件 | 说明 |
|---|---|
| `docs/planning/oscbf-reuse/research/18-elastic-qp-admission/probe_elastic_qp_admission.py` | 实验脚本。只用仓库既有的生产内核 `work.jax_control_facade.JaxControlLoop`（→ `work.jax_kernel_factory` → cbfpy → qpax），外加一次对 `qpax.solve_qp` / `solve_qp_elastic` 的直接调用。除本目录外不写任何文件。 |
| `docs/planning/oscbf-reuse/research/18-elastic-qp-admission/results.json` | 全部逐 tick 的机器可读结果（含每组场景的完整 `u_safe`、`q_next`、`h` 分组最小值）。 |
| `docs/planning/oscbf-reuse/research/18-elastic-qp-admission/probe-output.txt` | 上述脚本的完整 stdout（本文件所有"（本次实测）"数字都来自它或 `results.json`）。 |

复现命令（在仓库根目录执行，与仓库内其它研究脚本同一套环境变量约定）：

```bash
PYTHONPATH=portable_oscbf/vendor/dpax JAX_PLATFORMS=cpu \
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python3 docs/planning/oscbf-reuse/research/18-elastic-qp-admission/probe_elastic_qp_admission.py
```

环境：（本次实测打印）`jax=0.6.2 backend=cpu x64=True threads(OPENBLAS=1, OMP=1)`；内核配置 `relax_cbf=True cbf_relaxation_penalty=100000.0 solver_tol=0.001 alpha=(10.0*h) enable_rate_limit=False num_cbf=43`；`n_obb_links=10 num_obstacle_constraints=10 d_safe_collision=0.03`；JIT 预热 7.6–8.3 s。控制参数取生产 profile（`dt=dt_path=0.01`、`kp_pos=160`、`kp_orient=10`、`kp_joint=0.45`、`w_pos=40`、`w_orient=10`、`w_joint=0.1`、`temporal_lambda=0.2`、`solver_tol=1e-3`、`task_mode=tool_axis_5d`、`damping=0.05`、`nullspace_speed_limit=0.18`、`reference_lead_m=0.01`），参考轨迹用仓库自带 `data/nurbs/ik_input.mat`。

### 8.2 场景与关键读数

| 场景 | 构造 | 关键读数（本次实测） |
|---|---|---|
| **A** 干净工况 | 无任何障碍物，从 `q=0` 跟踪轨迹 60 tick | 全部 `qp_ok=True`；`max delta_slack = 1.87e-9`；`min h = -0.01`（J1 关节限位行）；`primal_residual` 恒 0.0；`min_h_by_group joint=-0.0100 self=+0.1630 sing=+1.1042` |
| **B** 已在裕度内 | 1 mm 球沿射线滑到障碍裕度 `h=-5.015 mm`（80 mm 中的 5 mm），5 tick | `delta_slack ≤ 4.7e-11`（**零松弛**）；`obs h` 单调回升 -5.015→-3.318 mm；`qp_ok=True`；`|u|` 0.117→4.445 |
| **C** 真冲突 | 半径 0.02 m 的球放在末端中心（障碍行饱和在 `h=-0.080 m`），8 tick | `delta_slack = 0.12` 每 tick；`primal_residual = 0.12`；`qp_ok=**True**`；`|u|` 长到 **5.686 rad/s**，多个关节顶在 2.932/3.665 上限；`|Δq|` 到 0.0569 rad/tick |
| **D** 可证明不可行的 QP | `min 0.5u²` s.t. `u≥1 ∧ u≤-1` | 硬 QP：`converged=0`、30 次（=max_iter）、`x=0`、`Gx-h=[1,1]`、`z=5.94e215`、`s≈1.5e-10`。弹性：`converged=1`、7 次、`t=s1=1.0` |
| **E** 关节在包络之外 | J1 低于下界 0 / 0.005 / 0.02 / 0.05 / 0.20 rad | 见 §2.2(b) 表：`delta_slack` 在需求退速超过 `vmax=0.5` 后才出现，且 ≈ 需求-可达（0.0991、1.59895），`qp_ok` 仍全为 True |
| **F** 距离核饱和 | 球心在末端中心，半径 0.001/0.05/0.30/0.80 m；同连杆 OBB 自重叠 | `h_obs` 一律 `-0.080000`；自重叠盒（偏移 0.05 m，半长 0.075 m）返回 **0.000000**，偏移 0.2 m（分离 0.05 m）返回 0.049997 |
| **G** 罚因子敏感性 | 场景 C 的状态，`penalty ∈ {1e3,1e5,1e7,1e9}`，每次显式清 JIT 缓存 | 四者 `delta_slack = 0.12000000614759472`、`primal_residual=0.12`、`|u|`、`q_next` **逐位相同**；同状态 10 条障碍行中 2 条 `|L_g h|₁ = 0` |
| **Gate 表** | 对 A/B/C 的逐 tick 结果事后套 `max(slack) ≤ eps` | `eps=1e-9`：A 冻 1/60、B 冻 0/5、C 冻 8/8；`eps ∈ {1e-6,1e-3,1e-2}`：A、B 冻 0，C 冻 8/8 |

### 8.3 实验过程中的两点方法学记录（避免后人重犯）

1. **不要在 eager 模式下反复调用 `CBF.h_2`。** 首版脚本把它放在 401 次参数扫描里，每次都会重新编译 `compute_dcol_obstacle_clearance` 内部的 `jax.lax.cond` 两个分支，进程因 LLVM `Cannot allocate memory` / `JITDylib ... is defunct` 段错误（exit 139）崩溃且**可复现**。改成"一次性 `jax.jit` 包装 + `vmap` 扫参数"后消失。脚本里的 `raw_h()` 因此是 jit 化的。
2. **改 `cbf_relaxation_penalty` 必须显式清 JAX 编译缓存。** 该系数在跟踪期被读取，而 JAX 的编译缓存键只看输入 avals，不清缓存会静默复用旧程序。首版场景 G 就是这样得到"四个罚因子完全相同"的**无效**结果；加上 `loop._path_tracking_fn.clear_cache()` 强制重跟踪后结果仍然相同，但此时"相同"才是真实结论（原因见 §4 的 `|L_g h|₁=0`）。场景 G 只改了进程内的属性并在 `finally` 中还原，**没有**修改任何仓库文件、配置或阈值。

## 9. 这个问题的三种答案分别意味着什么

**看"可观察行为"这一列。** 其余两列是各方案的验收前提。

| | 现场可观察到的机器人在做什么 | 要说什么"已核实"，还须补测 | 今天还不掌握的信息 |
|---|---|---|---|
| **A** 保留弹性求解器，加事后准入门（`max slack ≤ eps` 或 `min h ≥ -eps_h`），不通过就冻结/锁存 | 轨迹精度与今天相同；约束冲突时现场表现为**突然停止运动并报警**（若真做锁存，则是停到人工复位）；**在达到 `eps` 之前会静默地花掉那部分裕度**（例如 `eps_h = 1 mm` 就是自碰撞 30 mm 中默许 3.3%、障碍 80 mm 中 1.25%） | 干净长跑（分钟~小时级、含真实感知噪声）的 slack 分布，用于定 `eps` 及其单位；冻结后的恢复/复位契约；冻结期间下发给驱动器的位置序列与使能状态；"冻结"是否要锁存（今天只是单 tick `u=0` + 0.02 s 低通） | 没有长时 slack 分布；没有冻结行为的既有契约；`eps` 的数值属安全政策而非代码事实 |
| **B** 实机路径改硬 QP，不收敛即视为不可行并停车 | 不会再有"静默越界"；代价是**数值噪声直接变成停车**，且"不可行"的判定实际上是在读内点法的 `converged` 标志——**失败 ≠ 已证明不可行**（§3），因此会出现"其实可行却被判不可行"的停车 | 硬 QP 在现有工况下的 `converged` 失败率；该标志与"真的不可行"（可用 `slack > eps` 作为代理）的一致性；停车后的恢复契约 | 从未在生产路径上跑过硬 QP（生产恒为弹性），失败率与代价分布未知 |
| **C** 只用最终积分并 clip 之后的命令做后检（在 `q_next` 处重算全部行） | 能看到 A/B 看不到的一层：**积分 + clip + 低通之后实际到达的位形**的裕度；对"CBF 条件这一 tick 成立、但一步之后更糟"的情形敏感。但**仍无法分辨"越过 1 mm"与"越过 1 m"**（距离核在 0 处饱和，§2.3） | 一次全行重算的耗时（100 Hz 预算）；clip 生效时的行为（本次实测场景 E 已见 `q_next` 被 clip 到机械限位 0.0）；与 A/B 组合时的优先级 | 没有重算耗时数据；没有 clip 生效频率的统计 |

三种方案在本仓库都能找到"半成品"支撑：A 的判据形态已有先例（`h_vals >= -1e-3`，`jax_kernel_factory.py:350`，但只在 rate-limit 分支可达），A/B 需要的准入量已被算出来（`primal_residual`，`jax_kernel_factory.py:354-356`，今天只进诊断），C 需要的 `q_next` 也已经在返回值里（`jax_barrier_terms.py:111-115`）。这些是"实现位置"的事实，不构成对本票的选择建议。

## 10. 未决 / 需要用户决议

以下是本文件**不回答**、需要用户决定的政策项。它们都不是代码事实问题：

1. **选 A / B / C 还是组合**（例如 A 做常规门 + C 做补充检查），以及门不通过时的动作是"冻结当前位姿"、"锁存到人工复位"、"退出跟踪但保持位置环"，还是"按预置的应急制动轨迹退出"。
2. **`eps` 的数值与单位**。"允许花掉多少裕度"是安全政策。若采用 A，还需明确 `eps` 作用在哪个量上：`max_i t_i`（混合 m/s 与 rad/s，且被 α 缩放）、`max_i t_i/α_i`（各行 h 单位）、还是重建的 `min h`（rad / m / 奇异值乘积混合）。
3. **`eps` 是否按行分组给不同值**。自碰撞 30 mm、障碍 80 mm、关节限位 10 mm 的物理含义不同；是否统一一个值需要政策判断。
4. **是否需要先补"长时 slack 分布"测量**再做选择。本文件只提供 60 tick 的单机样本与一个"噪声地板 ~1e-9 量级"的观察，不足以支撑定阈值。
5. **是否接受"距离核在 0 处饱和"这一既有事实**成为准入检查的能力上限（即检查只能回答"是否被顶住"，不能回答"越过多少"）。

## 11. 本次未能验证的事项

- 未运行 ROS、未接硬件、未测 CAN 与下游位置环；低通之后的真实轨迹没有测量。
- 未做传感器噪声注入实验；§2.2(e) 的量化影响未测。
- 未构造"两个障碍方向相反导致行间冲突"（§2.2(c)）的场景。
- 自碰撞的饱和结论只在**核函数层面**验证（同一连杆的两个合成重叠盒），未在真实自碰撞构型下验证 `_obb_pair_distance_impl` 的行为。
- slack 噪声地板（1.87e-9）是**单机、单次 60 tick、x64、CPU、无感知噪声**的最大值样本，不是上界，也不是分布。
- 场景 C 的"包住末端"是人造的隔离场景，用于把机制显出来；它不是对现场任何工况发生概率的估计。
- 未测选项 C 所需的全行重算耗时。
- 未核验本机 `qpax 0.1.4`（其 METADATA 标注 "Copyright 2026 The qpax developers"，带 `explicit`/`implicit` 双后端，与常见上游版本形态不同）是否与某个公开发布指纹一致；本文件所有 qpax 结论以**本机 site-packages 的实际源码行**为准。
- §3 关于内点法停止准则的表述中，"不输出不可行证明"一节以本机 qpax 源码为直接证据（`converged` 的构造、返回元组内容），Nocedal & Wright 仅作背景参考，本文件未对该书做逐句引用。
