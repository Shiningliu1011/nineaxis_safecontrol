# 06B — 选定目标跟踪语义与验收标准（grilling）

**What to build:** HITL 决策票（只能由用户拍板，AFK agent 无法回答），由 06A 的
刻画结果驱动，回答：

- **最终演示/验收场景是什么？**（加工？抓取？避障演示？）——决定参考轨迹与姿态的来源/参数化；
- **路径跟随（现状）还是严格时间参数化跟踪？**误差按弧长横偏（现在）还是按参考时间评估？
- **roll 是否重要？**5D 工具轴（现状）还是 6D 位姿任务？
- **验收阈值：0.1mm 还是 1mm？**位置/姿态/横偏/进给率的通过阈值（现有 evaluator 已算指标，阈值未定）。

产出：验收口径文档（误差定义 + 指标 + 阈值），作为后续验收基准与 07B/01B 的目标对齐输入。

**Blocked by:** 06A（先有现状刻画，再谈目标）。

**Type:** grilling（HITL）

**Queue:** wayfinder-core — 决策枢纽：控制目标与验收口径由此定型。
**Tracker:** #14 (https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/14)

**Status:** resolved (2026-09-04, HITL grilling — 8 decisions made)

- [x] 确认演示/验收场景与参考轨迹、姿态来源
- [x] 选定语义（path following vs time-parameterized）与误差定义
- [x] 确定 5D vs 6D / roll 取舍
- [x] 写出验收口径文档（误差定义 + 指标 + 阈值），并回写 MAP 讨论项 A/B/C 定稿

---

# Resolution — 选定目标跟踪语义与验收标准（2026-09-04）

**结论（一句话）：** 最终控制目标是**分层语义的几何路径跟随**——硬目标 = cross-track + 工具轴，性能 = 关节电机能力上限下的速度表现；安全减速不计为误差，时间元素降级为诊断。

---

## Target tracking contract（控制目标契约）

1. **主控制目标** = 几何路径跟随：机器人工具必须沿空间路径运动，cross-track error（路径法向偏差）和 tool-axis error（工具轴对准误差）是硬验收指标。
2. **速度上限** = 关节电机能力（`dq_max` = 额定转速 × 0.70），nominal feedrate 不再限制速度。feedrate 调度实际为 `gamma × min(joint, cbf, rate, tool_axis, endpoint_brake)`。
3. **安全减速** = 设计内行为：CBF 因障碍/限位/奇异性降速是允许的，不计为跟踪误差。
4. **速度平滑** = rate constraint 启用：相邻周期速度变化量有界，防止跳变。temporal_proximal（λ=0.2）保留。
5. **`source_time_s`** = 零控制作用，仅作诊断字段（telemetry 显示、卡死检测代理、evaluator 完成度的历史参考）。
6. **`reference_feedrate_scale`** = 设大值（≥10），让 nominal 不绑定速度，joint cap 成为实际瓶颈。
7. **死代码清理**：`ff_scale`（条件前馈）和 `maximum_reference_feedrate_step_m_s` 标记为待删除（归 09 清点）。
8. **trajectory loader** = 通用接口设计（Q4 定案：当前蝴蝶轨迹是算法测试轨迹，后续换真实任务）。泛化接口最小输入：`(positions_m, rotations, feedrate_m_s, source_time_s)`。
9. **控制器频率** = 50Hz（20ms 预算）为当前部署频率，100Hz（10ms）为优化目标。01A 优化后可切回 100Hz。
10. **5D tool-axis tracking** = 继续现状（roll 自由），冗余 DOF 更多，后续需要时可 additive upgrade 到 6D。

---

## Error semantics（误差语义定义）

### cross_track_m（mandatory primary 位置指标）

**定义**：在参考点处剔除切向分量的法向横偏。

```
error = p_ref - p_actual
lateral = error - tangent * dot(tangent, error)
cross_track = norm(lateral)
```

端点处（`at_endpoint=True`）使用全 3D 误差（不剔切向）。

**来源**：`path_following.py:429-438`，与 gamma（cross-track stop）输入是同一量（但参考点时间不同——evaluator 用周期初参考，gamma 用推进后参考）。

**统计**：RMS、p95、max 三者均保留。

### pos_error_m（diagnostic 位置指标）

**定义**：`‖ee_pos(q_next) − R(s_ref_end)‖`，3D 欧氏距离。混入切向相位滞后（≈ feedrate·dt ≈ 2.7mm @ 0.27m/s）。

**用途**：仅诊断，不判 pass/fail。

### orient_error_rad（mandatory 姿态指标）

**定义**：`sin(θ)`，θ = 工具 X 轴与参考轴的夹角。`‖B_des^T · (a_des × a_cur)‖`，B_des = 期望旋转后两列，a = 旋转矩阵第 0 列。roll 恒 0（5D 任务）。

**小角度**：sin(θ) ≈ θ rad。

**统计**：RMS、max。

---

## Orientation semantics（姿态语义）

- **任务维度**：5D = 3D 位置 + 2D 工具 X 轴。roll 自由。
- **姿态参考来源**：
  - 蝴蝶轨迹（当前）：surface_normal 帧（X=内向径向，Y=圆柱轴，Z=切向），圆柱吸附后派生。
  - 泛化轨迹（后续）：`(N, 3, 3)` 旋转矩阵直接提供，通过 `set_orientation_from_data()` 接口（需新增）。
- **泛化接口**：`orientation_mode` 需新增 `"from_data"` 模式，从轨迹数据直接读姿态，绕过 `"surface_normal"` 和 `"fixed"` 的派生逻辑。
- **升级路径**：如需 6D，task_mode 改 `full_6d`，需先解决参考数据的 roll 来源问题。Additive change，不丢已有工作。

---

## Progress and feedrate semantics（进度与进给率语义）

### progress（弧长进度）

- `reference_progress_m`：虚拟参考点弧长 s_ref，单调不减，受 bounded lead（0.01m）冻结。
- `projected_progress_m`：实测末端位置向路径的投影弧长，单调不减，投影速度上限 3.0 m/s。
- 两者解耦：参考按调度推进，投影按实测推进，lead 保证参考不超前投影超过 10mm。

### nominal feedrate

- **语义**：性能参考，不是硬上限。设大 `reference_feedrate_scale`（≥10）让 nominal 远高于 joint cap。
- **实际速度上限**：`joint cap`（QP 在线推导的关节速度极限）。
- **速度缩放源**：`feedrate_cmd / 1000 × scale × feedrate_scale`（`ik_data_loader.py:188`）。scale 是几何计算产物（~0.967），feedrate_scale 是可配参数。

### feedrate 调度

```
feedrate = gamma × min(joint, cbf, rate, tool_axis, endpoint_brake)
```

| 限制项 | 含义 | 生产状态 |
|---|---|---|
| joint | 关节速度盒约束 | ✅ 生效 |
| cbf | CBF 安全约束 | ✅ 生效 |
| rate | 相邻周期速度变化限制 | ✅ 需启用（当前未启用） |
| tool_axis | 工具轴角速度限制 | ✅ 生效（当前数值不绑定） |
| endpoint_brake | 虚拟参考减速 | ✅ 生效 |
| gamma | cross-track stop（ct≥10mm→feedrate=0） | ✅ 生效 |
| nominal | 源数据进给率 × scale | ❌ 设大后不绑定 |

### CBF 降速

CBF 因障碍降速是**设计内安全行为**，不计为跟踪误差。降速事件记录为 diagnostic（`limiting_reason` 标签）。

---

## Completion semantics（完成度语义）

**唯一定义**：
```
path_completion = projected_arc_length / total_arc_length
```

- **完成事件**：`reference_progress_m ≥ total_length_m − ε` → `completed = True`，feedrate=0，控制器冻结命令。
- **`source_time_s` 不参与完成度计算**（Q6 定案）。
- **`endpoint_hold_s`** 独立记录，不混入 completion。
- **验收**：completion = 100%（mandatory）—— 路径必须走完。

---

## Startup semantics（启动语义）

**启动时投影播种**（Q7 定案）：

1. TransitionExecutor（AEB-RRT* + replay）把机器人带到路径起点附近。
2. 控制器收到 `start_tracking` 服务 → 读当前末端位置 → 调用 `initial_path_follower_state(geometry, config, ee_position)` 投影到路径最近点。
3. `reference_progress_m = projected_arc_length + reference_lead_m`（0.01m）。
4. `projection_segment` 从投影结果初始化。
5. 姿态参考自动为 `R_des(s_proj)`（弧长索引的旋转矩阵）。

**对任意轨迹通用**：投影是纯路径操作（`project_local()`），不依赖圆柱几何。

**边界情况**：如果机器人离路径 > 10mm，gamma 压死 feedrate=0，退化为 A 方案行为。

---

## Acceptance table（验收指标表）

| # | Metric | Definition | Status | Threshold | Evidence/Source |
|---|---|---|---|---|---|
| 1 | cross_track RMS | 路径法向偏差均方根 | **mandatory** | ≤ 0.15 mm | 实测 baseline mean=0.071mm，留 2× 余量。08A 后可收紧 |
| 2 | cross_track p95 | 路径法向偏差 95 百分位 | **mandatory** | ≤ 0.5 mm | 实测 baseline p95=0.193mm，留 2.5× 余量 |
| 3 | cross_track max | 路径法向偏差最大值 | **mandatory** | ≤ 2.0 mm | 实测 baseline max=1.082mm，留 ~2× 余量 |
| 4 | orient RMS | sin(工具轴夹角) 均方根 | **mandatory** | ≤ 0.05° | 实测 baseline=0.0035°，留 14× 余量（泛化后可调） |
| 5 | orient max | sin(工具轴夹角) 最大值 | **mandatory** | ≤ 0.5° | 实测 baseline=0.0069°，留 72× 余量 |
| 6 | completion | projected_arc / total_arc | **mandatory** | = 100% | 路径必须走完 |
| 7 | QP success rate | QP 求解成功步数占比 | **mandatory** | ≥ 99.9% | 实测 baseline=100%。3000 步允许 ≤3 次失败 |
| 8 | min obstacle clearance | 全程最近障碍距离 | **mandatory** | ≥ 0 (无碰撞) | 物理底线 |
| 9 | min obstacle clearance | 同上 | **provisional** | ≥ 30 mm | ISO/TS 15066 手指级最低要求。08A 后可收紧到 50mm |
| 10 | pos_error 3D | 3D 参考点距离（含切向滞后） | **diagnostic** | — | 不判 pass/fail，仅工程诊断 |
| 11 | feedrate mean/max/min | 调度后进给率统计 | **diagnostic** | — | 供 07B/01B 分析 |
| 12 | joint utilization ratio | feedrate / joint_cap | **diagnostic** | — | 反映关节电机利用程度 |
| 13 | slowdown_count | 速度被 cap 限制的次数 | **diagnostic** | — | 按 limiting_reason 分类 |
| 14 | controller_step_ms | 每步计算耗时 | **diagnostic** | — | 记录 |
| 15 | deadline miss rate | 耗时 > 20ms 的步数占比 | **provisional** | ≤ 1% | 50Hz 预算。01A 优化后可切 100Hz |
| 16 | min obstacle clearance | 同上 | **TBD** | 收紧目标 | pending 08A 有障碍实验数据 |
| 17 | cross_track RMS/p95 | 同上 | **TBD** | 收紧目标 | pending 08A 有障碍实验数据 |
| 18 | deadline miss rate | 耗时 > 10ms 的步数占比 | **TBD** | — | pending 01A 优化完成 |

---

## Explicitly rejected alternatives（已否决方案及原因）

### 1. 时间参数化轨迹跟踪（B 方案 — Q1 否决）

**否决原因**：CBF 降速 = 时间误差来源，安全动作与验收指标对抗。避障工况无法按时间口径验收——要么削约束，要么指标必超差。项目无外部时钟同步需求（非产线节拍/多轴协同）。

### 2. 纯几何路径跟随无速度目标（A 方案 — Q1 未选）

**未选原因**：没有速度性能口径，演示可能"通过但慢得难看"。C 方案保留了性能可观测性（feedrate 统计作为 diagnostic），同时不把速度作为硬目标。

### 3. 6D full orientation tracking（B 方案 — Q3 否决）

**否决原因**：(1) 当前参考数据没有真实 roll 信息（surface_normal 帧的 Z 轴是切向，不是物理工具方向）；(2) 少 1 个冗余 DOF（4→3），影响 07B 避障策略；(3) 6D 任务奇异段更多，QP 不可行风险上升。

### 4. nominal feedrate 作为硬跟踪目标（B 方案 — Q5 否决）

**否决原因**：与 Q1 C 方案矛盾。CBF 降速时被 nominal 限制 = 安全系统被性能目标约束，方向错误。

### 5. 3D reference-point distance 作为 primary 位置指标（B 方案 — Q2 未选）

**未选原因**：切向相位滞后（≈2.7mm）是正常跟踪行为，却算进误差。设 3mm 阈值则相位滞后吃掉大部分余量，失去对真实路径偏离的敏感度。

### 6. 强制 transition planner 精确带到路径起点（A 方案 — Q7 未选）

**未选原因**：AEB-RRT* + replay 本身不是高精度定位器，要求毫米级精度不现实。投影播种容忍过渡误差，更鲁棒。

### 7. source_time 完成度（B 方案 — Q6 否决）

**否决原因**：源时间比 ≠ 弧长比（非线性），语义不清晰。弧长完成比与 Q1 C 方案（几何路径为主目标）一致。

### 8. 25Hz 控制频率

**否决原因**：CBF 反应步数只有 ~7 步（每步接近 10.8mm vs d_safe 30mm），安全裕度不足。虽然 cross_track 实测 <10mm（path state machine 有保护），但 CBF 对障碍的响应速度不可接受。

---

## Consequences（后续影响）

### trajectory loader

- `reference_feedrate_scale` 设大（≥10），nominal 不绑定速度。
- 泛化接口需新增 `set_orientation_from_data(R_des_series)` 方法和 `"from_data"` orientation_mode。
- `IKTrajectoryData.__init__()` 的圆柱拟合+吸附需做成可选（flag `snap_to_cylinder`）。
- `load_repository_trajectory()` 是蝴蝶专用，泛化轨迹直接构造 `PathGeometry`。

### path tracking kernel

- **零改动**。`PathGeometry.from_samples()` 和 `project_local()` 已经是通用的。
- `initial_path_follower_state()` 已有播种逻辑，ROS 控制器需调用它替代全零初始化。
- rate constraint 需启用（`rate_limit_du_max` 参数需设合理值）。

### evaluator

- `cross_track` 升级为 mandatory primary（已有，需移除 pos_error 的 primary 地位）。
- `pos_error`（3D）降级为 diagnostic。
- `completion_fraction` 改用弧长比替代源时间比。
- 新增 feedrate diagnostic 指标（joint utilization ratio, slowdown_count）。
- 新增 controller timing 指标（step_ms, deadline_miss_rate）。
- 验收阈值表（上述 Acceptance table）写入 evaluator 配置。

### nullspace / 07B

- 5D 任务有 4 个冗余 DOF（比 6D 多 1 个），07B 冗余策略直接受益。
- 速度上限 = joint cap（不是 nominal），07B 的冗余利用空间更大。
- feedrate diagnostic 数据（joint utilization, slowdown）供 07B 评估策略效果。

### performance / 01B

- 50Hz 预算（20ms）为当前部署频率，100Hz（10ms）为优化目标。
- 01A 优化清单直接影响能否切回 100Hz。
- deadline miss rate ≤ 1% (provisional)。

### tests

- e2e 测试的断言需更新：`source_time > 0.5` → 改用弧长进度断言。
- evaluator 单元测试需覆盖新的指标定义和阈值。
- 投影播种的单元测试需新增。

### documentation

- README / CONTEXT.md / ONBOARDING.md 的"跟踪蝴蝶轨迹"表述需更新为通用描述。
- 验收口径文档（本 resolution）作为后续验收基准。

---

## 新增 tickets

因 06B 决策而变得"现在可以精确定义"的 ticket：

- **09（配置一致性清理）**：删除 `ff_scale` 死代码和 `maximum_reference_feedrate_step_m_s` 死参数；`reference_feedrate_scale` 设大值。已有 ticket，优先级提升。
- **evaluator 更新 ticket**（新增）：根据本 resolution 更新 evaluator 指标定义、阈值、completion 语义。依赖 06B 定案。
