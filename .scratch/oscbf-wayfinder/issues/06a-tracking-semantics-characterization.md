# 06A — 精确刻画当前实现的跟踪语义：arc-length path following vs 时间参数化（research）

**What to build:** 当前实现是**弧长路径跟随 + 进给率调度**（joint/CBF/rate/
tool-axis/endpoint-brake 五种限速、cross-track stop、bounded lead=0.01m、
条件前馈），`source_time_s` 只用于完成度计算——不是「按参考时间复现轨迹」。
这与「trajectory tracking」的常见定义（时间参数化、误差随参考时间评估）不同，
直接影响验收指标怎么算。本票只做 research，如实刻画现状，**不做取舍**：

- 代码级行为精确描述（path_state 5 分量、进给率调度优先级、limiting_reason 语义）；
- 与时间参数化跟踪的差异矩阵（误差定义、完成度、参考姿态来源）；
- 现有 `tracking_evaluator` 的指标到上述语义的映射（哪些算数、哪些无意义）。

「按哪种语义验收、阈值怎么定」由 **06B（grilling 决策票）** 拍板。

**Blocked by:** None — 分析可独立完成。

**Type:** research（AFK）

**Queue:** wayfinder-core — P0 起点：回答「现状到底为何」，是 06B/07B 的输入。
**Tracker:** #2 (https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/2)

**Status:** resolved (2026-09-04, agent — research done; input to 06B)

- [x] 写出 path_state / 调度器行为规格（带代码引用）
- [x] 差异矩阵：path following vs time-parameterized（误差/完成度/进给率）
- [x] 映射文档：evaluator 指标 ↔ 两种语义

---

# Resolution — 精确刻画当前实现的跟踪语义（2026-09-04）

**结论（一句话）：** 当前实现的语义是 **弧长参数化路径跟随 + 进给率调度（feedback-coupled arc-length path following with feedrate scheduling）**。
参考点的推进速率由「名义进给率调度 × 物理极限」决定，且被测量投影的 bounded-lead 上限冻结；**不存在以墙钟时间为独立变量的轨迹复现**。
`source_time_s` 只做诊断/完成度显示，不参与控制。
验证方式：逐行读码 + 纯 NumPy 状态机对真实 `data/nurbs/ik_input.mat` 复现（`.scratch/oscbf-wayfinder/verify_06a.py`），JAX 内核与 NumPy 的数值等价性由既有测试 `portable_oscbf/tests/test_jax_path_following.py` 断言（atol=2e-6）。

## 1. 轨迹数据：生成、加载、预处理（事实）

| 步骤 | 行为 | 代码位置 |
|---|---|---|
| 生成（仓库外） | `data/nurbs/ik_input.mat`：NURBS 分块时间参数化采样，**14992 点 @ Ts=0.002s（29.982s），23 块**；单位 mm；含 `position/velocity/acceleration/jerk/feedrate_cmd/time_series` | 数据文件（实测） |
| 加载 | `load_repository_trajectory()` → `IKTrajectoryData` | `portable_oscbf/work/ik_data_loader.py:65,118` |
| 坐标变换 | mm→m；`T_traj_to_base` = 单位旋转对齐 × 等比缩放 **scale=0.967380**（span 60% J1 行程）+ 平移 ≈ **[−4.5e-5, 0.332, 1.367]** m（质心对齐 ee_center=[0,0.343,1.387]） | `ik_data_loader.py:18-62, 219-227` |
| NaN 修复 | 逐列线性插值 | `ik_data_loader.py:229-246, 194-200` |
| 圆柱吸附（无条件，在 `__init__` 执行） | 轴硬编码 **(0,1,0)**；最小二乘拟合圆心/半径；全部位置点径向分量钉到拟合半径；速度/加速度/向心分量去径向投影 | `ik_data_loader.py:203-207, 496-519` |
| 姿态参考（生产模式 surface_normal） | 控制器以 `orientation_mode=surface_normal, cylinder_axis_direction=[0,1,0], cylinder_center=空 → 自动拟合` 调用 `set_surface_normal_orientation`；**X=指向轴线的内向径向、Y=圆柱轴、Z=X×Y**（切向）；对 `_R_des_series` 逐点重建，并重算 `_omega_series`（相邻标架差分 ÷Ts，rad/s） | `oscbf_controller.py:281-290`；`ik_data_loader.py:391-452, 659-678` |
| 弧长参数化 | `PathGeometry.from_samples(positions, R_des, feedrate, source_time)`：剔除零长段（14992→**14984 点**，弧长累计），切向中央差分，`omega_per_m`=相邻旋转 rotvec/段长，quaternion slerp 采样 | `path_following.py:123-184` |
| 进给率数值 | `feedrate = raw_feedrate_cmd/1000 × scale × 3.5`（`reference_feedrate_scale=3.5`，控制器参数 `oscbf_controller.py:167`）→ **[0, 0.2709] m/s**（源最大 0.0774 m/s） | `ik_data_loader.py:188`（实测） |

实测几何：总弧长 **1.8168 m**；拟合圆柱：圆心 ≈ [−7.2e-5, 0.3466, 1.6373]、半径 **0.2706 m**、轴 [0,1,0]；吸附后 |r−R| 最大偏差 = **0.0 mm**（算术上精确贴面）；`source_time_s` 沿弧长严格单调（用作显示无歧义）。

## 2. path_state 5 分量 + 推进规则（规格）

`PathFollowerState`（`path_following.py:77-89`；JAX 向量布局 `jax_path_following.py:82-88`）：

| 分量 | 含义 | 更新规则 |
|---|---|---|
| `reference_progress_m` | 虚拟参考点弧长 s_ref（OSC 唯一参考来源） | 每周期：`s_ref ← max(s_ref, min(s_ref + feedrate·dt_path, s_proj + reference_lead_m))`，clip 到 [0, L]；单调不减；**与墙钟时间无关** |
| `projected_progress_m` | 实测末端位置向路径的局部投影（锚段 ±96 段窗口） | `clip(raw_proj, s_proj_old, s_proj_old + 3.0·dt)` — 单调不减、≤3.0 m/s 投影速度上限 |
| `projection_segment` | 投影所在段索引（锚窗中心） | 由投影弧长反查 |
| `endpoint_hold_s` | 完成后累计保持时长 | `completed ? hold+dt : 0`（仅日志，不进控制） |
| `completed` | s_ref ≥ L − ε | 置位后永久成立，feedrate=0 |

初始化：ROS 控制器用 `initial_path_state()`＝**全零**（`oscbf_controller.py:337,729`）——参考从弧长 0 开始，**不由实测末端姿态播种**（`initial_path_follower_state()` 的播种仅用于 NumPy/测试路径）。每周期推进发生在周期开始（输入 = 周期初实测 q），随后 QP 解一步（Ø dt=0.01s），积分得 q_next，再用 **q_next 后的位置**调 `reconcile_path_state_after_motion` 更新投影（`jax_kernel_factory.py:678-696`）。

注意：报告的 `cross_track_error_m` 是**周期初**参考（advance 前样本）相对实测位置的横偏；控制用的 task error 是周期末参考（advance 后样本）相对 q_next 的误差——**两个误差参考点不同**（`jax_path_following.py:212-219` vs `jax_kernel_factory.py:689-692`）。

## 3. 进给率调度：全部限制项与生效逻辑

`feedrate = gamma · min(nominal, joint, cbf, rate, tool_axis, endpoint_brake)`，且终点时 =0（`path_following.py:480-489`；JAX 同构 `jax_path_following.py:256-272`）：

| 限制项 | 公式 | 来源 |
|---|---|---|
| **nominal** | 弧长索引的源数据进给率（已有 ×3.5 缩放）；s=0 恰为 0 时向前探测一个 `max_projection_speed·dt` 采样（避免零速吸收态） | `path_following.py:458-482` |
| **joint** | 最大的 ŝ 使 \|u_bias + u_per_m·ŝ\| ≤ dq_max（9 关节速度盒） | `jax_kernel_factory.py:557-563`；`path_following.py:559-568` |
| **cbf** | 最大的 ŝ 使全部 43 行 CBF 约束 G·(u_bias + u_per_m·ŝ) ≤ h（α(h)+L_f h，f=0 → α(h)）仍满足（行序：18 关节限位 + 14 自碰撞 + 10 聚合障碍 + 0 ESDF + 1 奇异） | `jax_kernel_factory.py:563-564`；`oscbf_velocity_config.py:233-292` |
| **rate** | \|u − u_safe_prev\| ≤ du_max 盒 → **生产未启用**（`JaxControlLoop` 未传 `rate_limit_du_max` → `enable_rate_limit=False` → rate_cap=∞） | `jax_control_facade.py:145-147`；`jax_kernel_factory.py:565-572` |
| **tool_axis** | ŝ ≤ max_tool_axis_speed_rad_s / ‖ω_per_m‖ | `path_following.py:453-457` |
| **endpoint_brake** | ŝ ≤ sqrt(2·0.5·(L − s_ref))（**虚拟参考减速**，非制动器模型） | `path_following.py:594-608` |
| **gamma（cross-track stop）** | γ = clip(1 − ct/0.01, 0, 1)，ct=参考点处剔除切向的横偏；γ≤ε 时 feedrate=0 | `path_following.py:431-439, 487-489` |
| **endpoint** | s_ref ≥ L−ε 或 completed → feedrate=0 | `path_following.py:483-485` |
| **lead cap（隐含第 8 项）** | s_ref ≤ s_proj + 0.01（不改变 feedrate 报告值，只冻结推进） | `path_following.py:500-505` |

**「优先级」的准确表述**：所有 cap 同时生效（取 min），优先性只体现在 `limiting_reason` 标签：JAX 码按 joint(1)<cbf(2)<rate(3)<tool_axis(7)<endpoint_brake(6)<nominal(0) 顺序首个严格小于者获胜，然后 cross_track(γ≤ε) 覆盖，最后 endpoint 覆盖（`jax_path_following.py:397-421`）；数值相等时 nominal 获胜。**γ 在中值（0<γ<1）时 feedrate 已被削减但标签仍可能报 nominal**。

实测（无反馈 cap 的纯调度复现，端点跟随参考）：参考走完 1.8168 m 用 **848 步（8.48 s）**，标签 nominal 806 次 / endpoint_brake（末 0.42 s）42 次；源时长 29.98 s —— ×3.5 重缩放基本等价于时间轴整体缩放。**tool-axis cap 数值上永不绑定**：nominal 峰值 0.2709 m/s < tool_axis cap 最小值 0.5412 m/s（‖ω_per_m‖ 峰值 3.6955 rad/m → 2.0/3.6955）。若 feedrate_scale 提升到 ≈7 以上才可能绑定。

「bounded lead」实测：把末端钉在参考后方 5mm（贴路径，横偏≈3mm 级），名义进给 0.2555 m/s，300 步后参考冻结在 lead=**0.0100 m**（推进 +0.007 m 后触及 lead 上限）——推进速率 = min(进给率调度, 投影+0.01 头寸)，参考与**测量投影耦合**。

## 4. reference_lead_m 作用（语义关键）

`reference_lead_m=0.01`（控制器参数，同时覆盖了 PathFollowingConfig 默认值）把「参考点」从纯时间/调度驱动改为**测量耦合**：
- 末端进给跟得上 → 参考按调度走（误差率名义值）；
- 末端落后（投影停滞/扰动/限速）→ 参考冻结、等待末端收回——**语义安全**：参考绝不会跑到末端前面超过 10mm；
- 稳态时参考与投影差 ≈ 一个控制周期的推进量（≤ feedrate·dt ≈ 2.7mm @ 0.27 m/s）。
历史上它替换了「参考锚定在投影推进」的版本，那会引入 ~20× 的低速跟随（代码注释 `path_following.py:491-499`）。

## 5. source_time_s 实际使用点（全部为诊断/显示，零控制作用）

1. `PathGeometry.source_time_s`：每采样随弧长插值携带（`path_following.py:222-225`），推进函数 `advance_path_state` 的参数里**没有时间项**（实测断言：签名仅 geometry/config/state/ee_position/dt/caps）。
2. 控制器 `progress_snapshot` → `arc_fraction = source_time / (num_points·Ts)`（`oscbf_controller.py:614-633`）；telemetry 日志显示 `source_time=.../29.984s`（`oscbf_controller.py:668-685`）。
3. 卡死检测启发式：5s 窗口内 source_time 冻结 + feedrate<0.05 + pos_err>5mm → 冻结当前位姿（`oscbf_controller.py:527-547`）——**用源时间作参考停滞的代理**（与 lead 冻结是同一现象的两种探测）。
4. `TrackingEvaluator.completion_fraction = 最后 source_time / trajectory_duration_s`（`tracking_evaluator.py:252-254`），并入综合评分（15% 权重）。
5. e2e 测试用 `source_time_s > 0.5` 断言「参考没卡死」（`tests/test_oscbf_full_flow_e2e.py:189-197`）。

无任何路径推进/任务误差/限制项读取 source_time。

## 6. tool_axis_5d 误差定义（5D 任务 = 3D 位置 + 2D 工具 X 轴）

- 参考姿态来自 surface_normal 帧（见 §1）；任务误差 `task_error_5d = [p_cur − p_des (3D), B_des^T·(a_des × a_cur) (2D)]`，B_des = 期望旋转的后两列（轴切平面基），a = 旋转矩阵第 0 列（工具 X 轴）（`tool_axis_task.py:95-106, 146-155`）。
- **绕工具轴的 roll 完全不控制**（误差在该方向恒 0）；5D 任务 Jacobian = 位置 3 行 + 2 行轴误差（`tool_axis_task.py:126-135`）。
- 报告向 6D 填充时补一个 0（`task_error_report_6d_jax` → `err_6d = [pos3, axis2, 0]`，`tool_axis_task.py:265-268`）。
- 因 B 正交，**报告的 orient 误差 = ‖a_des × a_cur‖ = sin(θ)，θ 为工具轴夹角**（小角度 ≈ θ rad）。
- 控制：反馈速度 = task_gain·e 前置符号为 −（e = p_cur − p_des，`jax_kernel_factory.py:404-412`）；位置反馈用**横偏**（剔除切向）而非常规 3D 误差，端点处回退全 3D（`jax_kernel_factory.py:503-508, 600-601`）；大误差时反馈饱和：1×kp 半径 5mm、限速 0.25→0.8 m/s（`jax_kernel_factory.py:53-59, 515-532`）；端点保持模式把位置/姿态反馈×0.1（Cascade 振荡抑制，`jax_kernel_factory.py:533-539, 620-623`）。

## 7. 前馈的真相（文档 vs 代码差异 — 事实）

`_path_control_nominal` 计算了 `ff_scale`（ct<3mm 时 1.0，线性退到 0 @10mm，注释声称「条件前馈」），**但 ff_scale 从未被使用——死代码**（全文件仅出现于 `jax_kernel_factory.py:632`）。实际执行的 `u_nom = control_u_bias + control_u_per_m·feedrate_m_s` 为**无条件前馈**，仅由已受 gamma/caps 削弱的 `feedrate` 缩放到 0（即跨轨停止时前馈随 feedrate→0 归零）。MAP.md 里「条件前馈」的说法与可执行代码不符，应以代码为准。

同样：`PathFollowingConfig.maximum_reference_feedrate_step_m_s=0.005`（yaml 声称平滑参考进给）在推进函数中**未被使用**（仅声明+校验，`path_following.py:42,54`）——死参数，归 ticket 09 清点。

## 8. 与严格 time-parameterized trajectory tracking 的差异矩阵

| 维度 | 当前实现（弧长路径跟随+进给率调度） | 严格时间参数化跟踪 |
|---|---|---|
| 参考索引 | 弧长 s ∈ [0, L]，R(s) | 墙钟 t ∈ [0, T]，X_des(t) |
| 推进规则 | Δs = min(feedrate·dt, s_proj+lead−s)⁺，单调、**与时间解耦**；受测量投影冻结 | Δt = dt 恒定，**与测量无关** |
| 状态量 | 5 分量 path_state（无时间坐标） | 时间 t（外部时钟） |
| 误差定义 | 横偏（剔切向）+ 工具轴 2D；端点全 3D | X(t)−X_des(t) 3D + 全姿态 6D（时间对齐） |
| 扰动/卡死 | 参考冻结、末端收回（lead+cross-track）；「进度」暂停 | 参考继续前进，误差增长直至超差 |
| 完成 | s_ref ≥ L | t ≥ T |
| 进给率 | 显式调度变量（nominal×3.5 后再过 6 个物理/参考 cap） | 隐含在位置剖面 \|Ẋ_des(t)\|；无独立调度 |
| 姿态参考 | 弧长索引 surface_normal 帧（X 内向径向/Y 轴向/Z 切向） | 数据姿态时间索引（当前数据本身有此信息但未用） |
| 自由度 | 5D 任务（roll 空闲） | 通常 6D（或自定义） |
| 误差与时间的对齐 | 报告误差为「周期末参考 vs 集成后末端」，无时间对齐概念 | 误差必须对齐到同一时刻参考 |
| 完成度显示 | source_time 插值（源时间轴代理，单调但非线性于弧长） | t/T 直接 |

**本质**：当前实现是「进给率显式调度 + 参考与实测投影弱耦合（bounded lead）」的路径跟随——介于纯几何路径跟随与时间复现之间，但时间参数化特征（t 作为独立变量、误差按 t 评估）**完全不存在**。唯一残留的时间元素是数据侧 per-sample `source_time_s`（仅显示/检测）+ 时长归一化分母 `num_points·Ts=29.984s`。

## 9. tracking_evaluator 指标 → 两种语义的映射（事实）

| 指标 | 实际测量的量 | 在 PF 语义下 | 在严格 TT 语义下 |
|---|---|---|---|
| `pos_error_m`（mean/max/p95） | ‖ee_pos(q_next) − R(s_ref_end)‖ 3D，`err_6d[:3]` | 横偏 + **切向相位滞后**（≈feedrate·dt，峰值 2.7mm）+ 瞬态混合；不是纯路径偏差 | 仅当 s_ref 恰为时间进度时才成立；否则参考点不同步 |
| `orient_error_rad` | ‖B^T(a_des×a_cur)‖ = **sin(工具轴夹角)**，roll 恒 0 | 有定义（弧长索引轴参考） | 需要 6D 姿态误差才有意义；5D 报告不含 roll |
| `cross_track_m` | 周期初参考处剔除切向的横偏 | **原生指标**（也是 gamma 输入的同一量，但参考点提前一周期） | 无直接对应（需人为投影到参考切线） |
| `feedrate_m/s` 统计 | 调度后 feedrate（gamma×min caps；**不含 lead 冻结修正**——参考冻结时报告值可>0 而推进=0） | 原生（调度质量指标） | 无对应（可改为报告 \|Ẋ_des‖ 对比） |
| `qp_success_rate` / `min_obs_distance` / `delta_slack` | QP 可行性/障碍/松弛 | 语义中立（安全指标，两种语义都适用） | 同左 |
| `completion_fraction` | 最后 source_time / (N·Ts) | 弧长进度在**源时间轴**上的代理（单调但非线性）；与 arc 分数不同，与墙钟无关 | 接近严格定义，但分母 N·Ts 不是数据时长（29.984 vs 29.982），且完成由 s_ref≥L 判定 |
| `completed` | `reference_at_endpoint`（s_ref≥L−ε） | **PF 完成定义** | TT 应判 t≥T |
| `tracking_score` | 0.30·pos(均值/5mm) + 0.20·cross(均值/3mm) + 0.20·qp + 0.15·completion + 0.15·obs(20mm) | **混合语义**：pos 项混入切向相位滞后、cross 项纯路径、completion 项源时间 | 只有 qp/obs 项可移植；pos/cross/completion 需重定义 |

**要点**：a) evaluator 无「时间对齐误差」指标——现有 pos_error 是"离参考点 3D 距离"，在 PF 语义下把**设计内**的切向相位滞后计为误差；b) 完成度把 PF 的完成事件（弧长）与 TT 的完成度量（时间）混用；c) cross_track 与 gamma 是同量（但参考点时间不同），是唯一纯 PF 指标。

## 10. 已由代码确定的事实（结论性列表）

1. 语义 = 弧长路径跟随 + 进给率调度（§2-§4）；参考与测量投影弱耦合（lead=0.01m）。
2. source_time_s 无控制作用（§5）。
3. 误差定义 = 位置横偏（端点全 3D）+ 工具轴 sin(θ)（§6）；roll 自由。
4. 姿态参考 = 圆柱 surface_normal 帧（弧长索引），与圆柱吸附后的路径一致；吸附幂等且实测径向偏差 0mm。
5. 生产 QP 为 elastic（relax_cbf=True, penalty=1e5）；rate 约束未启用；temporal_proximal λ=0.2 生效（P += 0.2·I, q −= 0.2·u_safe_prev，`oscbf_velocity_config.py:216-231`）。
6. 参考初始化为弧长 0（ROS 控制器不播种实测姿态）。
7. 「条件前馈」为死代码；`maximum_reference_feedrate_step_m_s` 为死参数（§7）。
8. 完成机制：s_ref≥L → feedrate=0 + 控制器冻结命令（`oscbf_controller.py:549-557`）；卡死检测双启发式（`oscbf_controller.py:505-547`）。
9. tool-axis cap 在当前参数下数值不绑定（0.5412 vs 0.2709 m/s）。
10. 端到端附加环节（非跟踪语义核心）：发布命令 τ=0.02s 低通；plant 为几何积分+S 曲线+kp=80 位置环；QP 输出为关节速度并积分成位置命令。

## 11. 必须留给 06B（用户拍板）的决策

1. **验收语义**：按 PF（横偏+工具轴+调度）还是 TT（时间对齐误差）验收；误差阈值（现有 5mm/3mm/20mm 评分线是否保留）。
2. `pos_error` 口径：3D 距离（现状，含切向相位滞后） vs 横偏+切向容差分离。
3. `completion` 定义：弧长/源时间/墙钟。
4. roll 是否控制（5D vs 6D）→ 影响姿态误差上限与奇异段约束。
5. 名义进给率来源：×3.5 重缩放是否保留（演示用途决定）；tool-axis cap 是否收紧。
6. 死代码/死参数（ff_scale、maximum_reference_feedrate_step_m_s）的取舍——实现或删除（归 09/06B 均可，06B 只需定"是否期望该行为"）。
7. 参考起始是否改为实测播种（现状=恒从弧长 0 开始）。

（06A 不新增 ticket：上述每条要么已有归属（09 清点、06B 拍板），要么是 06B 的输入，不需要新的白盒调查票。）
