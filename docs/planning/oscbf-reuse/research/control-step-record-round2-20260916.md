# 一步记录的第二轮取证：缺测、四个读者、一个定义、死字段

日期：2026-09-16。
范围：只读检索。本文只写这一个文件，没有改动仓库任何源码、配置、测试或阈值；没有启动 ROS、仿真或硬件。
标记约定：【代码】＝本仓库文件与实测行号；【文档】＝本仓库 Markdown 的路径；【JAX源码】/【JAX文档】/【Python文档】＝已安装库的 docstring 或官方文档；【实测】＝本次在本机用仓库既有代码跑出的结果；【推断】＝我的推理，不是原文。

本文与同一目录下两份既有产物（`control-step-record-20260916.md`、`control-step-record-round2-prototype-20260916.md`）独立：凡是与它们结论一致的地方我给出自己的一手行号，凡是本次核验与它们不一致的地方单列在「与直觉相反之处」。**四个界面的规模本次实测复核**（【实测】，AST 计数 + 逐行点数）：

| 界面 | 实测规模 | 计数方式 |
|---|---|---|
| 内核位置元组 | **42** | 逐行点数【代码】`jax_kernel_factory.py:751-772` 的 `return (...)` 元素 → 42；facade 解包【代码】`jax_control_facade.py:629-641` 同为 42 |
| `JaxPathTrackingResult` | **30** | AST 数 `AnnAssign` 字段（【代码】`jax_control_facade.py:56-90`） |
| 节点 dict | **29** | AST 数 `step_once` 的 `Return` 节点 `Dict.keys`（【代码】`oscbf_controller.py:543-583`） |
| 评价器别名表 | **12** | 逐行点数【代码】`tracking_evaluator.py:97-108` |

---

## Q1：三个无人生产的测量，本轮能不能接上？

### 结论

**不能，而且不应该接。** 这不是 plumbing 缺口，而是三层叠加的既定约束：

1. 「准入/重叠缺测 ⇒ 证据不足」是**写明了的政策**（【文档】`docs/tracking_evaluation.md:8`）；
2. `overlap` 的生产者是 **OFF-02 交付几何事实 + OFF-09 拥有组合与锁存**，边界被规格明确划开，且规格有一条**禁令**禁止把现有 `TrackingStepData.overlap` 升格为准入契约（【文档】`docs/specs/off02_obb_geometry_admission_spec.md:11`、`:155`）；
3. `admission_ok` 缺的**不是数据而是判据**：`primal_residual` 早已算出（【代码】`portable_oscbf/work/jax_kernel_factory.py:354-356`），但要用它必须定 `eps` 的**数值与单位**，而这是安全政策，被 `18-elastic-qp-admission.md §10` 列为待用户决议。

**最要紧的一条**：现有**通过中**的测试 `tests/test_oscbf_controller_smoke.py:506-507` 断言生产链产出 `{"unmeasured": 5}`。本轮接上生产者会**直接改掉一条通过的验收断言**。

### (a) `online_verdict == "insufficient_evidence"` 是文档写明的结果，不是缺陷

【文档】`docs/tracking_evaluation.md:8`（**引文逐字**）：

> `online_verdict`：所记录的准入与重叠判据。任一拒绝/重叠/关键故障会保留失败；缺少准入或重叠记录时为证据不足，不能从 `qp_ok` 推断。

同一份文档 `:80` 进一步写明当前节点只产出 `model/kernel_candidate` 边界，并把未测项点名：

> 当前默认报告是 `model/kernel_candidate`：误差取内核积分后的模型状态，尚未经过发布低通或实际执行。尚未取得的最终命令、模拟状态、真实反馈和准入/重叠证据明确未测；OFF-09/OFF-13 的后续结果可经上述输入契约接入。

【文档】`docs/adr/0008-tracking-evaluation-scope-and-angle.md:13`：

> 证据来源与测量边界分开表达，缺失数据和未接受阈值给出证据不足；求解、准入、命令与反馈继续遵守各自的验证边界。

【文档】`docs/planning/oscbf-reuse/validation/2026-09-12-off15/REPORT.md:64`（当代证据也这么记录）：

> 当前节点只产生内核模型边界的报告；滤波后命令、模拟状态、真实反馈及**未接入的准入/重叠信息明确未测**。

代码侧对应（【代码】`src/robot_safecontrol_moveit/tracking_evaluator.py:402-403`、`:502`）：

```python
admission = Counter("accepted" if s.admission_ok is True else "rejected" if s.admission_ok is False else "unmeasured" for s in steps)
overlap = Counter("overlap" if s.overlap is True else "clear" if s.overlap is False else "unmeasured" for s in steps)
...
online = "fail" if fatal or negative_clearance or admission["rejected"] or overlap["overlap"] else "pass" if n and admission["accepted"] == n and overlap["clear"] == n else "insufficient_evidence"
```

⇒ 三态计数器 + 一条 else 分支：**未测走 `insufficient_evidence` 是被实现的政策**。判为「缺陷」不成立。

### (b) 文档没有任何「生产运行应当达到 `online_verdict == "pass"`」的期望

穷举 `online_verdict == "pass"` 的全部既有记录：

| 证据 | `online_verdict` | 值从哪来 |
|---|---|---|
| `docs/planning/oscbf-reuse/validation/2026-09-12-off15/{full-path,subinterval,held,reversed-axis,invalid-sample}.json:290` | `pass` | 【代码】`examples.py:40-41` **显式注入** `admission_ok=...`、`overlap=False` |
| `.../admission-rejected.json:290` | `fail` | 同上，第 5 个样本注入 `admission_ok=False` |
| `.../controller-tracking.json:290`、`...-review-fixes/controller-terminal.json:290` | `insufficient_evidence` | 真实生产节点路径 |
| `docs/planning/oscbf-reuse/validation/2026-09-12-off15/controller-tracking.md:3` | 「在线门控：证据不足」 | 同上 |

在测试里，唯一能到 `pass` 的写法是夹具注入。**逐字引用**：

- 【代码】`tests/test_tracking_evaluator.py:34`（`_step()` 夹具，全 469 行测试的公共底稿）：
  `qp_ok=True, admission_ok=True, overlap=False, step_latency_ms=1.,`
- 【代码】`docs/planning/oscbf-reuse/validation/2026-09-12-off15/examples.py:40-41`：
  `qp_ok=True, admission_ok=not (name == "admission-rejected" and i == 5),` / `overlap=False, feedrate_m_s=0.01, step_latency_ms=1.,`

与之对照，**生产链无法达到 `pass`，而且这条事实被一条通过的测试固定下来**——【代码】`tests/test_oscbf_controller_smoke.py:506-507`：

```python
    assert report.admission_counts == {"unmeasured": 5}
    assert report.overlap_counts == {"unmeasured": 5}
```

该测试的上下文（同文件 `:483-496`）：`node.step_once(_START_Q)` 五次 → `node._evaluator.update(step, ...)` → `node.tracking_report()`，即走的是真实节点 dict，不是夹具。

### (c) `overlap` 与 `obstacle_clearance_m` 的生产者归属

| 量 | 生产者归属 | 依据（引文） |
|---|---|---|
| `overlap` | 几何事实由 **OFF-02** 交付；组合与锁存由 **OFF-09** 拥有 | 【文档】`off02_obb_geometry_admission_spec.md:11`：「输出必须区分点态事实、区域覆盖和未知状态，**供后续 OFF-09 组合准入；OFF-02 本身不拥有锁存、恢复或命令发送职责**。」同文 `:150`：`overlap` 的处置是「拒绝并由 **OFF-09** 锁存」 |
| `obstacle_clearance_m` | **无票认领**。文档只写「调用方显式提供、已注明几何度量的净空」，并把接入点指向 OFF-09/OFF-13 的后续结果 | 【文档】`docs/tracking_evaluation.md:31`：「`obstacle_clearance_m` \| 调用方显式提供、已注明几何度量的净空 \| 没有量测时保持缺失，不能从裕度或 inactive sentinel 反推」；同文 `:80` 末句：「OFF-09/OFF-13 的后续结果可经上述输入契约接入」 |
| `admission_ok` | 未定（判据本身未决） | 见 (d) |

补充边界（【文档】`off02_obb_geometry_admission_spec.md §7`，`:268-283`）——以下均「不属于已证明能力」，其中与本问直接相关的三条逐字引用：

> - 完整蝴蝶跟踪的连续九轴任务域；
> - 生产 MoveIt 过渡是否始终位于当前局部盒内；
> - 实际状态误差、反馈延迟、反应和物理停车包络；
> - 真实 OBB/mesh 制造、装配和标定误差；
> …
> - OFF-09 的求解后组合门、冻结锁存和人工恢复；
> - OFF-13 的最终过滤命令复核与发送边界；

【推断】由此，`obstacle_clearance_m` 在 OFF-02 的能力范围内也只是**逐对保守下界**（`off02_obb_geometry_admission_spec.md:91-97`：`L - relative_motion_bound - geometry_error - numerical_tolerance` 与 `required_clearance_m` 比较），不是实测物理净空；把它当作 `obstacle_clearance_m` 会违反 `tracking_evaluation.md:31` 的「不能从裕度反推」。

### (d) `admission_ok` 是 OPEN question，不是 DECIDED policy

【文档】`docs/planning/oscbf-reuse/research/18-elastic-qp-admission.md §10` 全文只有 5 条，**逐字引用前两条**：

> 1. **选 A / B / C 还是组合**（例如 A 做常规门 + C 做补充检查），以及门不通过时的动作是"冻结当前位姿"、"锁存到人工复位"、"退出跟踪但保持位置环"，还是"按预置的应急制动轨迹退出"。
> 2. **`eps` 的数值与单位**。"允许花掉多少裕度"是安全政策。若采用 A，还需明确 `eps` 作用在哪个量上：`max_i t_i`（混合 m/s 与 rad/s，且被 α 缩放）、`max_i t_i/α_i`（各行 h 单位）、还是重建的 `min h`（rad / m / 奇异值乘积混合）。

同文 §6.1 给出了判据形态本身（**引文**）：

> 准入检查（admission check）＝求解之后，取该 tick 的松弛统计量并与阈值比较：
> ```
> max_i t_i  ≤  eps        （等价地：重建的 min_i h_i  ≥  -eps_h ）
> 不满足 → 该 tick 的命令不被采信（冻结/降级/锁存，具体动作属政策）
> ```

⇒ 被决定的是「判据长什么样」；**未决定的是哪个量、`eps` 取值与单位、以及失败动作**。所以 `admission_ok` 的 `eps` 不存在，无从生产。

### (e) `delta_slack` 单独不足：量纲 + 噪声地板

**量纲**（【文档】同文 §6.3，引文逐字）：

> 弹性松弛 `t_i` 是**加到行上界上**的：`ḣ_i ≤ h_row_i + t_i ⟹ t_i 与 h_row_i 同量纲 = ḣ_i 的量纲`
> 所以：关节限位行（`α = 10`）：`t` 的单位是 **rad/s**；自碰撞/障碍行（`α = 10` / `1.5`）：`t` 的单位是 **m/s**；奇异行：单位随 `h` 的乘子量纲。
> 把 `t_i` 换算成"这条行容许多花多少 h"要**除以 `α_i`**：`Δh_i = t_i / α_i`。

同文并明确警告：直接比较 `max_i t_i` 与以米为直觉的 `eps` 「会把 m/s 与 rad/s 混为一谈，并且被 `α ∈ {10, 1.5}` 缩放（两者相差 6.7 倍）」。

**噪声地板**（同文 §6.4，引文逐字）：

> 场景 A 的 60 个干净 tick 上：`eps = 1e-9` → **冻 1 个 tick**；`eps ≥ 1e-6` → 0 个；而场景 C 的 8 个 tick 在 `eps ∈ {1e-9, 1e-6, 1e-3, 1e-2}` 下**全部**被判冻结。也就是噪声地板的最大值（1.87e-9）落在 1e-9 与 1e-6 之间，`eps` 必须显著大于 ~2e-9 才不会在干净工况误触发；1e-6 在本次样本里已经足够。
> **但必须标注**：这个"噪声地板"是**一台机器、一次 60 tick 采样**（x64、CPU、`OPENBLAS_NUM_THREADS=1`、`OMP_NUM_THREADS=1`、无感知噪声注入）的最大值样本，**不是上界**。样本最大值不能当硬上界用。

⇒ 结论：`delta_slack` 的存在**不能免除**一次政策决策。至少要选 (i) 量（`max t` / `max t/α` / `min h`）、(ii) 单位、(iii) `eps` 数值。同文 §10 第 3、4 条还额外把「是否按行分组给不同 `eps`」与「是否先补长时 slack 分布」列为待决。

顺带一条可靠性缺陷（【文档】同文 §6.2 第 3 点，引文）：

> **`solver_tol` 不能直接当这个 `eps`**。`solver_tol = 0.001` 的定义是"KKT 残差的 ∞-范数阈值"……是一个无量纲化的混合量，**不是**对 `max t` 的上界，也不是对 `min h` 的上界。所以"因为 solver_tol 是 1e-3，所以 eps 也取 1e-3"是**没有依据的**。

### (f) `TrackingStepData.overlap` 被规格明文禁止升格

【文档】`docs/specs/off02_obb_geometry_admission_spec.md:155`（**引文逐字**）：

> 现有 `TrackingStepData.overlap: bool | None` 只可作为诊断投影，**不得作为该准入契约本身**。

这句话就紧跟在 §2.5「OFF-09 消费契约」的组合真值表之后（`:136-153`），所以它不是随口一提，而是规格正文对消费契约的限定。

### 与直觉相反之处（Q1）

1. **直觉**：三个量没人生产 = 忘了接线，本轮顺手接上。
   **实际**：接线会（i）绕过一项未决的安全政策，(ii) 违反 OFF-02 规格 `:155` 的禁令，(iii) 让 `tests/test_oscbf_controller_smoke.py:506-507` 这条**通过中**的断言失败。
2. **直觉**：`qp_ok=True` 说明这一步是安全的，可以据此给出 `pass`。
   **实际**：`docs/tracking_evaluation.md:8` 专门写了「**不能从 `qp_ok` 推断**」。生产 QP 恒为弹性求解，`qp_ok` 只由 `isfinite(u) & converged` 决定（【代码】`portable_oscbf/work/jax_kernel_factory.py:342-346`），不含任何屏障或松弛判据（【文档】`18-elastic-qp-admission.md §5.1`）。
3. **直觉**：`overlap` 字段已经存在，等于生产者已经存在。
   **实际**：字段存在 ≠ 契约可用，规格 `:155` 直接把它否掉作为准入契约。
4. **直觉**：`admission_ok` 缺的是测量。
   **实际**：缺的是**判据**。测量量（`primal_residual`）已在返回值里（【代码】`jax_kernel_factory.py:354-356`），今天只进诊断（【代码】`oscbf_controller.py:559-561`）。

---

## Q2：节点的 29 键 dict 该不该继续手写？

### 结论

**可以删，但不能「只删」——必须先补两个前置动作**：

1. **`qp_primal_residual` 必须先搬进记录**：它是侧信道上唯一被真正控制路径读到的值（【代码】`oscbf_controller.py:560` 读 `self._loop.last_qp_primal_residual`），而记录里没有这个字段。
2. **3 个键没有记录字段**（`projection_before_m`、`path_progress_m`、`qp_primal_residual`），其中 2 个是宿主侧派生量；删表时要保留这 3 个的计算位置。

另外**确认**：dict 从不上任何 topic，也从不被逐字写盘。节点只发布 `JointState`。dict 的内容**间接**进入 `tracking_report.samples.json`，但只经过评价器的规范化白名单（13 个指标键），不是逐字持久化。

### (a) 发布面：只有 `JointState`

【代码】`src/robot_safecontrol_moveit/oscbf_controller.py` 全文只有两处发布相关代码：

| 行 | 内容 |
|---|---|
| `:147-149` | `self._publisher = self.create_publisher(JointState, publish_topic, qos_profile_sensor_data)` |
| `:786` | `self._publisher.publish(message)`（在 `_publish_positions` 内，`:782-786` 构造 `JointState`） |

其它 ROS 实体：`create_subscription`（`:141-146` `JointState`；`:165-167` `Float32MultiArray`，仅当 `enable_perception_obstacles`）、`create_service`（`:171-174` `Trigger`，作为**服务端**）、两个 timer（`:153`、`:154`）。全文件**没有** `create_publisher` 的第二处，也没有 `std_msgs`/自定义消息的发布。

⇒ **确认主代理的判断**：只有 `JointState` 出网。dict 不上任何 topic。

**但 dict 会间接落盘**：【代码】`:648` 每 tick `self._evaluator.update(step, wall_time_s=start)` → 评价器构造 `TrackingStepData`（只保留别名白名单 + 具名字段）→ `_report_writer.submit(...)` → `tracking_report.samples.json`。实测持久化样本的形状（【实测】，读 `docs/planning/oscbf-reuse/validation/2026-09-12-off15-review-fixes/controller-terminal.samples.json`）：

- `samples[0]` 的键：`admission_ok, at_endpoint, constraint_metrics, limiting_reason_code, overlap, projected_progress_m, projection_before_m, qp_ok, reference_progress_m, values, wall_time_s`（11 个）
- `samples[0]["values"]` 的键：`actual_tangent_speed_m_s, cross_track_m, delta_slack, feedrate_m_s, legacy_orientation_error, obstacle_clearance_m, obstacle_margin_m, online_cross_track_m, pos_error_m, qp_primal_residual, source_time_s, step_latency_ms, tool_axis_error_rad`（13 个）

⇒ 29 个 dict 键里，只有 **11** 个的语义进入落盘记录，且全部改名（见 Q3/第一轮 C 部分）。`q_next`、`u_safe`、`err_6d` 的矢量本身不落盘。

### (b) 计数与清单

**29 个键**（【实测】AST 解析 `step_once` 的 `Return` 节点 `Dict.keys` 长度 = 29，行 `:543-583`）。逐键列出：

| # | 行 | 键 | # | 行 | 键 |
|---|---|---|---|---|---|
| 1 | 544 | `q_next` | 16 | 564 | `min_obs_dist` |
| 2 | 545 | `u_safe` | 17 | 566 | `delta_slack` |
| 3 | 546 | `err_6d` | 18 | 569 | `reference_source_time_s` |
| 4 | 547 | `ee_pos` | 19 | 570 | `reference_at_endpoint` |
| 5 | 548 | `ee_rot` | 20 | 571 | `cross_track_error_m` |
| 6 | 549 | `reference_rotation` | 21 | 572 | `feedrate_m_s` |
| 7 | 550 | `reference_tangent` | 22 | 573 | `feedrate_nominal_m_s` |
| 8 | 551 | `path_state` | 23 | 574 | `feedrate_joint_limit_m_s` |
| 9 | 552 | `projection_before_m` | 24 | 575 | `feedrate_cbf_limit_m_s` |
| 10 | 553 | `actual_tangent_speed_m_s` | 25 | 576 | `feedrate_rate_limit_m_s` |
| 11 | 554 | `constraint_metrics` | 26 | 577 | `feedrate_tool_axis_limit_m_s` |
| 12 | 555 | `reference_position_m` | 27 | 579 | `feedrate_endpoint_brake_limit_m_s` |
| 13 | 557 | `path_progress_m` | 28 | 581 | `gamma` |
| 14 | 558 | `qp_ok` | 29 | 582 | `limiting_reason_code` |
| 15 | 559 | `qp_primal_residual` | | | |

**读回的键**（【实测】正则扫描 `_control_tick` 全文 `:622-769`）：`_control_tick` 内出现 **19** 个 `step["..."]` 形式，其中 `:647` 是**写**（赋值左侧），所以**真读 18 个**：

`q_next`、`u_safe`、`err_6d`、`ee_pos`、`reference_position_m`、`qp_ok`、`reference_source_time_s`、`reference_at_endpoint`、`cross_track_error_m`、`feedrate_m_s`、`feedrate_nominal_m_s`、`feedrate_joint_limit_m_s`、`feedrate_cbf_limit_m_s`、`feedrate_rate_limit_m_s`、`feedrate_tool_axis_limit_m_s`、`feedrate_endpoint_brake_limit_m_s`、`gamma`、`limiting_reason_code`。

**三类读者合并后：29 个键没有一个「完全没人读」**（【实测】集合运算）：

| 读者 | 键数 | 独有键 |
|---|---|---|
| `_control_tick` | 18 | 7 个 `feedrate_*` 中的 6 个常驻诊断键、`gamma`、`q_next`、`u_safe` |
| 评价器（`step_from_result` 别名表 + 具名 `get()` + `TrackingStepData` 字段） | 20 | `actual_tangent_speed_m_s`、`constraint_metrics`、`delta_slack`、`ee_rot`、`min_obs_dist`、`path_progress_m`、`path_state`、`projection_before_m`、`qp_primal_residual`、`reference_rotation`、`reference_tangent` |
| 测试（`tests/test_oscbf_controller_smoke.py`） | 10 | — |
| 并集 | **29** | 空 |

注意 3 点：

- 6 个 `feedrate_*_limit_m_s` 与 `gamma` 只在 **DETAL 日志行**（`:665-689`，每 300 步一次）被读；它们不是计算输入，是日志格式的一部分。
- `path_progress_m` 走的是**回退路径**：`step_from_result` 里 `reference_progress = _number(get("reference_progress_m", get("path_progress_m")))`（【代码】`tracking_evaluator.py:140`）——dict 里没有 `reference_progress_m`，所以回退**总是**生效。
- `min_obs_dist` 在测试里被单独断言（`:480`）。

**`step_latency_ms` 由调用方在 `step_once` 返回后写入**：

- 【代码】`:638` `start = time.perf_counter()` → `:641` `step = self.step_once(q_now, obs_kwargs=obs_kwargs)` → `:642` `duration_ms = (time.perf_counter() - start) * 1000.0` → **`:647` `step["step_latency_ms"] = duration_ms`** → `:648` `self._evaluator.update(step, wall_time_s=start)`。
- **为什么**：`step_once` 无法自量总耗时（要含内核输出到主机的转换）；这一边界被写进证据身份——【代码】`:597` `time_basis="perf_counter sample start; step_once includes kernel and host diagnostics"`。【文档】`docs/planning/oscbf-reuse/handoffs/17-control-budget.md:15`：「节点 `_step_durations` 计时包含 `step_once` 及内核输出主机转换；不包含后续评价、日志、低通、ROS 发布、CAN 和电机响应。」【文档】`docs/tracking_evaluation.md:32` 给出该指标的公布含义。
- 另有旁证：`portable_oscbf/scripts/prototype_step_record.py:232` 的注释也把这一写法描述为「augments with step_latency_ms after step_once returns」。

### (c) 没有记录字段的键：3 个

【实测】AST 对比（dict 键集 vs `JaxPathTrackingResult` 的 `AnnAssign` 字段集）：

| 键 | 行 | 为什么没有记录字段 |
|---|---|---|
| `projection_before_m` | `:552` | **纯宿主侧**：由 `self._evaluation_geometry.project_local(result.ee_pos_before, ...)` 计算（`:537-540`），只对第一步查询，之后用 `float(self._path_state[1])`。记录里对应字段只有 `ee_pos_before`（`:90`），不带投影语义 |
| `path_progress_m` | `:557` | **派生**：`float(result.path_state[0])`，与已有字段 `path_state` 冗余 |
| `qp_primal_residual` | `:559` | **来自侧信道**：`float(self._loop.last_qp_primal_residual)`。记录里没有这个字段 |

反向差额：记录里有 **4** 个字段节点从不读——`u_nom`、`posture_reference`、`reference_omega_per_m`、`ee_pos_before`。⇒「记录是给节点用的」不成立，删表不能反过来把记录裁到 29 项。

⇒ 主代理提名的两个候选**都成立**（`projection_before_m`、`path_progress_m`），且**还漏了第三个** `qp_primal_residual`——而它恰恰是最危险的一个，因为它来自侧信道。

### (d) 节点之外读这个 dict 的地方

【实测】全仓扫描 `[<key>]` 形式的索引：

| 文件 | 函数 / 位置 | 读的键 |
|---|---|---|
| `tests/test_oscbf_controller_smoke.py`（4 个测试函数） | `test_production_config_refactor_preserves_control_outputs_from_8479740`（`:276-306`） | `q_next`、`err_6d`、`feedrate_m_s`、`path_progress_m`、`qp_primal_residual`、`qp_ok` |
| | `test_hold_commands_continue_during_background_report`（`:407-409`） | `reference_at_endpoint`（并且**写**该键） |
| | `test_step_once_returns_valid_safe_state`（`:471-480`） | `q_next`、`u_safe`、`err_6d`、`qp_ok`、`min_obs_dist` |
| | `test_perf_report_p95_within_budget`（`:692-694`） | `q_next` |
| | `test_tracking_evaluator_integration`（`:491-492`） | 不按下标读，整块交给 `evaluator.update(step, ...)` |

合计：**4 个测试函数按下标读，共 9 个不同键**；另一个测试整块转发给评价器。

**有编号、有内容、值得单说**：【代码】`tests/test_oscbf_controller_smoke.py:480`

```python
    assert step["min_obs_dist"] is None  # disabled obstacle sentinel is not a measurement
```

**研究脚本**：`portable_oscbf/scripts/prototype_step_record.py` **不调用** `step_once`，它用 AST 解析节点源码取键名（`:327-335`，`raise RuntimeError("could not find the step_once dict")`）。所以它读的是**源码文本**，不是运行时 dict——删 dict 会让这个脚本报「could not find the step_once dict」（见 Q3「与直觉相反之处」）。

### (e) `min_obs_dist` 的条件等价与那条哨兵注释

【代码】`oscbf_controller.py:562-565`（**引文逐字**）：

```python
            # The kernel uses 1.0 as the disabled-obstacle sentinel. Do not
            # report it as a measured metre of clearance/margin.
            "min_obs_dist": float(result.min_obs_dist) if obs_kwargs and np.any(
                np.asarray(obs_kwargs.get("obs_enabled", [])) > 0.5) else None,
```

**确切条件**：`obs_kwargs` 为真值（非 `None`、非空 dict）**且** `obs_kwargs["obs_enabled"]` 里至少有一个元素 `> 0.5`。取不到 `obs_enabled` 键时 `np.asarray([])` → `np.any([])` 为 `False` → 结果 `None`。

**facade 侧不做同样的事**：【代码】`jax_control_facade.py:681` `min_obs_dist=float(min_dist)` —— 原样透传内核的 `1.0` 哨兵；`_update_qp_diagnostics`（`:754`）也原样写进 `last_min_obs_dist`。

**哨兵的产生点**（【代码】`portable_oscbf/work/jax_kernel_factory.py:225-227`）：

```python
        h_obs_masked = jnp.where(obs_enabled[None, :] > 0.5, h_obs, 1e3)
        min_obs_dist = jnp.where(
            jnp.any(obs_enabled > 0.5), jnp.min(h_obs_masked), 1.0)
```

⇒ 同一个量在三个界面上是三种语义：内核给哨兵 `1.0`、记录给 `1.0`（未标注）、节点给 `None`（已标注）。**唯一正确的写法只存在于节点那一层。**

### 与直觉相反之处（Q2）

1. **直觉**：dict 是记录的冗余影子，删掉就完事。
   **实际**：有 3 个键在记录里**不存在**，其中 `qp_primal_residual` 依赖侧信道（`_loop.last_qp_primal_residual`）。先删 dict 会让节点失去这个量——**必须先把它搬进记录**。
2. **直觉**：dict 键里总有几个没人读，可以顺手清。
   **实际**：29 个键**全部**至少有一个读者（节点 18、评价器 20、测试 9，并集 29）。不能按「没人读」为由裁键。
3. **直觉**：`min_obs_dist` 是记录的直通键。
   **实际**：节点把内核的 `1.0` 哨兵**主动转成 `None`**，facade 直通 `1.0`。这是**语义修正**，不是改名；删表时若按「同名字段直通」处理，会把「未测」退化成「1.0 米裕度」，直接违反 `docs/tracking_evaluation.md:31` 与 `:38`（「不把内核的占位距离当观测」）。
4. **直觉**：`portable_oscbf/scripts/prototype_step_record.py` 是运行时消费者。
   **实际**：它 AST 解析源码文本取键名（`:327-335`）。删 dict 会让它抛 `RuntimeError`，这是**工具脚本的失败**，不是生产/测试的失败。

---

## Q3：一个具名定义还是两个？

### 结论

**技术上可以是一个，但有一个硬约束会否决最自然的形态**：

- `typing.NamedTuple` 是**已注册的 pytree 节点**，`jax.jit` 可直接收发，同时保持普通 tuple 的位置解包与 `tree_map` 行为（【实测】）。**无需任何注册**。
- frozen `@dataclass` **不能**被 `jax.jit` 收发，除非 `jax.tree_util.register_dataclass`；未注册时整个实例被当成**一个 leaf**，进出 jit 直接 `TypeError`（【实测】）。
- **但即使注册了，现有 `JaxPathTrackingResult` 也回不去 jit**：它含 `constraint_metrics: dict | None`，其值是带 `'quantity'/'unit'/'source'` **字符串**的 dict，而**被 jit 的函数不能返回 `str`**（【实测】`TypeError: ... returned a value of type <class 'str'> ... which is not a valid JAX type`）。
- 于是第二轮的「一个定义」现实上有两种落法：**(甲)** 内核继续返回位置元组（或 NamedTuple），记录作为**host 侧**的唯一定义；**(乙)** 把记录拆成「可进 jit 的数组部分 + host 侧的元数据部分」。选哪种是设计取舍，不是事实问题；事实是**现有 30 字段的记录整体进不了 jit**。

**分层不构成障碍**：`portable_oscbf/work/` 不导入任何 ROS 模块，也不导入 `src/robot_safecontrol_moveit`（【实测】AST 扫描，0 命中）；而 `src/robot_safecontrol_moveit/` **已经在导入** `work.*`。方向与现有依赖一致。

### (a) `jax.jit` 收发 `NamedTuple`；它仍是普通 tuple

【实测】（jax 0.6.2，本机）：

```
NT is tuple: True                    # isinstance(nt, tuple) == True
leaves: [Array([0., 0., 0.], dtype=float32), 1.0]
tree_map: NT(a=Array([1., 1., 1.], dtype=float32), b=2.0)   # 类型被保留
jit out: NT  b=2.0  a=[0. 0. 0.]     # @jax.jit 直接收发
unpack positional: (Array([0.,0.,0.], dtype=float32), Array(2., dtype=float32, weak_type=True))
```

【JAX文档】pytrees 页对 namedtuple 的说明是把 `GetAttrKey(name: str)` 列为「For `namedtuple`s and preferably custom pytree nodes」，即 namedtuple 是内置的容器节点，带 key 路径。Python 侧身份来自【Python文档】`collections.namedtuple`：

> Returns a new tuple subclass named typename. The new subclass is used to create tuple-like objects that have fields accessible by attribute lookup as well as being indexable and iterable.

【Python文档】`typing.NamedTuple` 首句：「Typed version of `collections.namedtuple()`。」

本仓库已有先例：【代码】`portable_oscbf/work/qpax_warmstart.py:31-44` 的 `WarmStartState(NamedTuple)`，docstring 写「accepts the preceding converged state as an **input to a surrounding JIT control loop**」，且带显式有效性位 `valid: jax.Array`；`:46-53` 的 `empty_warm_start_state` 提供「还没有有效前值」的状态。

### (b) frozen dataclass 独有 vs NamedTuple 独有

【实测】逐个验证（Python 3.10，本机）：

| 能力 | `@dataclass(frozen=True)` | `typing.NamedTuple` | 依据 |
|---|---|---|---|
| `dataclasses.replace` | ✅ `DC(a=5, b=2.0)` | ❌ `TypeError: replace() should be called on dataclass instances` | 【Python文档】`dataclasses.replace`：「Creates a new object of the same type as `obj`… If `obj` is not a Data Class, raises `TypeError`」 |
| `dataclasses.asdict` | ✅ `{'a': 1, 'b': 2.0}` | ❌ `TypeError: asdict() should be called on dataclass instances` | 【Python文档】「`asdict()` raises `TypeError` if `obj` is not a dataclass instance」 |
| `dataclasses.fields()` | ✅ `['a','b']` | ❌ `TypeError: must be called with a dataclass type or instance` | 【Python文档】`dataclasses.fields` 条目 |
| `__post_init__` 校验 | ✅ 构造时执行（实测 `ValueError`） | ❌ 无对应钩子 | 【Python文档】「When defined on the class, it will be called by the generated `__init__()`」 |
| `frozen=True` 不可写 | ✅ `FrozenInstanceError` | ✅（tuple 天生不可写） | 【Python文档】「dataclasses will add `__setattr__()` and `__delattr__()`… raise a `FrozenInstanceError`」 |
| 哈希 | ✅ 自动生成（`eq` 与 `frozen` 均真时） | ✅ 恒等 | 【Python文档】「If eq and frozen are both true, by default `@dataclass` will generate a `__hash__()` method」 |
| `isinstance(x, tuple)` | ❌ `False`；`tuple(dc)` 也 `TypeError` | ✅ `True` | 【Python文档】「Both instances in the comparison must be of the identical type」（`__eq__` 不与 tuple 互等，实测 `DC(1)==(1,2.0)` → `False`） |
| 位置解包 | ❌ `TypeError: 'DC' object is not iterable` | ✅ | 同上 |
| 默认值 | ✅ | ✅（`_field_defaults`，`namedtuple` API） | 【Python文档】`typing.NamedTuple`「The field names are in the `_fields` attribute and the default values are in the `_field_defaults` attribute」 |
| 进 jit | ❌（未注册） | ✅ | 【实测】 |

### (c) 本仓库是否真的依赖这些 dataclass 独有能力？

**`JaxPathTrackingResult`：不依赖，只有一处工具脚本使用 `__dataclass_fields__`。**

【实测】全仓 `JaxPathTrackingResult` 的出现点：

| 位置 | 性质 | 用法 |
|---|---|---|
| `portable_oscbf/work/jax_control_facade.py:57`、`:600`、`:673` | 生产 | **定义、类型标注、构造**（在 jit 调用**之后**、由已转 numpy 的值构造） |
| `portable_oscbf/scripts/prototype_step_record.py:51,95,101,111,114,442` | 研究脚本 | `:442` `fields = [f.name for f in JaxPathTrackingResult.__dataclass_fields__.values()]` |
| `portable_oscbf/scripts/prototype_round2_seam.py:66` | 研究脚本 | `RECORD_CLASS = "JaxPathTrackingResult"`（字符串常量，AST 用） |

**关键事实**：`JaxPathTrackingResult` **从未**出现在 JAX pytree 位置——jit 的输入/输出是 42 槽位置元组（【代码】`jax_kernel_factory.py:751-772`），facade 在 `:629-641` 按位置解包、在 `:673-705` 才构造记录。全仓**没有**对它的 `replace(`、`asdict(`、`fields(` 调用（生产与测试均无）。

**但 `TrackingStepData`（评价器的记录）确实依赖 dataclass 独有能力**：

- 【代码】`src/robot_safecontrol_moveit/tracking_evaluator.py:90-94`：`return replace(result, values=values, qp_ok=..., ...)` —— `dataclasses.replace`（`:12` 导入）。
- 【代码】`src/robot_safecontrol_moveit/tracking_evaluator.py:390`：`"samples": [dict(asdict(step), wall_time_s=stamp) ...]` —— `asdict`。
- 【代码】`:538`：`asdict(_statistics(...))`。
- 【代码】`:347`：`self._steps.append(deepcopy(step_from_result(step)))`。

【推断】如果「一个定义」意味着把 `TrackingStepData`/`JaxPathTrackingResult` 合并成**一个 NamedTuple**，那么 `:90` 的 `replace(...)` 与 `:390` 的 `asdict(...)` 会**立即 TypeError**（【实测】）。若合并成一个 frozen dataclass，则它进不了 jit（(a)(d)）。这是「一个定义」方案里必须正面处理的一处张力。

### (d) `jax.tree_util.register_dataclass` 何时必需

【JAX源码】本机 `jax.tree_util.register_dataclass.__doc__`（**引文逐字**）：

> Extends the set of types that are considered internal nodes in pytrees.
> `data_fields`: … Data fields *must* be JAX-compatible objects such as arrays (`jax.Array` or `numpy.ndarray`), scalars, or pytrees whose leaves are arrays or scalars. **Note that `None` is a valid data field, as JAX recognizes this as an empty pytree.**
> `meta_fields`: … Metadata fields *must* be static, hashable, immutable objects, as **these objects are used to generate JIT cache keys**. In particular, metadata fields cannot contain `jax.Array` or `numpy.ndarray` objects.

**何时必需**：只有「要让一个 **dataclass** 穿过 jit / `tree_map` / `grad` 等变换」时才必需。【实测】未注册的 frozen dataclass 被当成**单个 leaf**，且 `jax.jit` 直接拒绝：

```
DC leaves: [DC(a=Array([0., 0., 0.], dtype=float32), b=1.0)]     # 整个实例 = 1 个 leaf
jit DC out ERROR: TypeError Error interpreting argument ... as an abstract array ...
```

**本仓库当前不需要它**，因为 `JaxPathTrackingResult` 不进 jit；**如果改成「内核直接返回 dataclass 记录」，就必须注册**。两条附带约束（【JAX源码】同上）：`meta_fields` 进 JIT 缓存键，所以任何进 meta 的东西必须静态且 hashable；`data_fields` 必须是数组/标量/pytree。

### (e) 放进 `portable_oscbf/work/` 是否引入分层违规？

**不引入。** 两侧实测：

- **内核包不依赖 ROS**：【实测】AST 扫描 `portable_oscbf/work/*.py` 的全部 `Import`/`ImportFrom`，对 `{rclpy, rcl_interfaces, std_msgs, sensor_msgs, geometry_msgs, nav_msgs, rosidl_runtime_py, ament_index_python, moveit, tf2_ros, tf_transformations, builtin_interfaces, visualization_msgs, trajectory_msgs, newaxis, robot_safecontrol_moveit}` 的命中数 = **0**。`portable_oscbf/work/__init__.py` 是**空文件**（0 字节）。
- **`src/` 已经在依赖 `portable_oscbf`**：`src/robot_safecontrol_moveit/perception_bridge.py:81-87`、`oscbf_plant.py:52-54`、`perception_demo.py:51-56`、`continuous_ik.py:405`、`oscbf_trajectory.py:57` 都 `from work.X import ...`；入口是 `oscbf_trajectory.bootstrap_portable()`（`oscbf_controller.py:29` 导入、`:130` 调用）。

反方向的依赖只存在于**测试与研究脚本**，不在 `work/` 包内：`portable_oscbf/tests/test_path_precision_initialization.py:21-22`（`from robot_safecontrol_moveit.tracking_contract/tracking_evaluator import ...`）、`portable_oscbf/scripts/prototype_round2_seam.py:53-56`。

⇒ 【推断】把「一步记录」定义在 `portable_oscbf/work/` 并由 `src/robot_safecontrol_moveit/` 消费，是**顺着现有依赖方向**的；反过来（记录定义在 `src/` 而内核要返回它）才会引入内核 → ROS 侧的反向依赖。

### 与直觉相反之处（Q3）

1. **直觉**：`NamedTuple` 和 frozen dataclass 都能当 pytree，选哪个看口味。
   **实际**：frozen dataclass **默认不是** pytree 内部节点（整个实例 = 1 个 leaf，进 jit 直接 `TypeError`）；`NamedTuple` 是内置已注册节点，零成本。
2. **直觉**：那给 dataclass 加 `@jax.tree_util.register_dataclass` 就行了。
   **实际**：注册也救不了现有记录——`constraint_metrics` 里带**字符串**，而被 jit 的函数**不能返回 `str`**（【实测】`TypeError: function f ... returned a value of type <class 'str'> at output component ['s']`）。要「一个定义进 jit」，必须先把这类字段改成数组（或拆到 host 侧）。
3. **直觉**：Python 的 `bool`/`int` 标注在 jit 里还是 `bool`/`int`。
   **实际**：`return True` 从 jit 出来是 `Array(True, dtype=bool)`，`return 3` 是 `Array(3, dtype=int32, weak_type=True)`。所以 `qp_ok: bool`、`limiting_reason_code: int` 若在 jit 内构造，类型契约**静默变化**；facade 现在是在 jit 返回后于 host 上做 `bool(qp_ok)`（【代码】`:680`）、`int(limiting_reason_code)`（`:698`），这一步也需要 device→host 同步。
4. **直觉**：`JaxPathTrackingResult` 是 dataclass，说明它需要 `asdict`/`replace` 这类能力。
   **实际**：生产与测试**从未**对它用 `asdict`/`replace`/`fields`；唯一的 dataclass 反射用法在一个研究脚本（`prototype_step_record.py:442` 读 `__dataclass_fields__`）。而真正依赖 `replace`/`asdict` 的是**评价器的 `TrackingStepData`**（`tracking_evaluator.py:90`、`:390`）——「一个定义」若取 NamedTuple 会在这两行翻车。
5. **直觉**：把定义搬进 `portable_oscbf/work/` 是让纯内核包反向依赖 ROS。
   **实际**：方向相反。`src/` 早已 `from work.X import ...`；`work/` 对 ROS 与 `src/` 的命中数为 0。搬进 `work/` 是**顺着**现有方向。

---

## Q4：「未测」应该怎么表达？

### 结论

在这个仓库里，四个候选编码的含义**各不相同，而且只有一种在两侧都诚实**：

| 编码 | 值槽（如 `obstacle_margin_m`） | 门槽（如 `overlap`） |
|---|---|---|
| **键不存在 / 写 `None`** | **诚实**：`missing` | **诚实**：`unmeasured` |
| `0` / `1.0` | **谎报**：被读成真实测量值 `0.0` / `1.0` | 静默退化为 `unmeasured`（不是 `clear`！） |
| `NaN` | 不谎报但**失真**：记成 `invalid` 而非 `missing` | 静默退化为 `unmeasured` |
| `False` | —（值槽收到 bool 会变 `NaN` → `invalid`） | **真值语义**：`clear` |
| `0.0` + 旁路 `__status` 键 | **旁路键会被静默丢弃** | 同左 |
| `0.0` + 具名状态字段 | ✅ 可行（需改字段） | ✅ 可行（需改字段） |

**根因**：`None` 与 `NaN` 是两个不同的桶（`missing` vs `invalid`），而门槽的类型判据只认 `bool`/`np.bool_`，所以「门槽的诚实」是**类型检查的副产品**，不是设计出来的契约。

**最要紧的一条**：最省事的旁路方案（`<键>__status`）会**静默失效**——`step_from_result` 的别名表是一张**封闭白名单**（【代码】`tracking_evaluator.py:96-110`），未登记的键在转换后**根本不存在**，不报错也不保留。

### (a) `None` → `missing`；`NaN` → `invalid`。区分在数据层有意义，在判据文案层无意义

【代码】`src/robot_safecontrol_moveit/tracking_evaluator.py:181-194`（**引文逐字**）：

```python
def _statistics(values: list[float | None]) -> MetricStatistics:
    missing = sum(v is None for v in values)
    finite = [v for v in values if v is not None and math.isfinite(v)]
    invalid = len(values) - missing - len(finite)
    if not finite:
        return MetricStatistics(0, missing, invalid)
```

`count` 只数 `finite`。**三个桶互斥且穷尽**：`None` → `missing`；非有限数（NaN/±inf）→ `invalid`；其余 → 有值。

**区分在三个地方是有意义的**：

1. 【代码】`:275` Markdown 表头逐字打印 `{stat.count}/{stat.missing}/{stat.invalid}` —— 三列独立。
2. 【代码】`:373-382` `trace_json()` 的编码器把非有限数写成 `{"nonfinite": "nan"}`，而 `None` 走 JSON `null`。实测持久化记录里确实能看到这个区分（【代码】`tests/test_tracking_evaluator.py:458`：`assert record["values"]["tool_axis_error_rad"] == {"nonfinite": "nan"}`）。
3. 【代码】`:539` `_constraint_report` 分别统计 `inactive_count` 与 `statistics` 的 `missing`/`invalid`。

**区分在判据层没有意义**：【代码】`:494-495`

```python
            elif not valid or value is None:
                verdict, reason = "insufficient_evidence", "required samples missing or invalid"
```

`valid` 来自 `stat.complete`（`:177-178`，要求 `missing == 0 and invalid == 0`），失败原因文案是**同一句**「missing or invalid」。⇒ 数据层分桶、判据层合流。

### (b) `_number` 收到 `bool` 返回 `math.nan` 的后果

【代码】`:26-34`（**引文逐字**）：

```python
def _number(value: Any) -> float | None:
    if value is None:
        return None
    try:
        if isinstance(value, (bool, np.bool_)):
            return math.nan
        return float(value)
    except (TypeError, ValueError, OverflowError):
        return math.nan
```

【实测】把 `True` 送进一个值槽的后果：

- `_number(True)` → `nan`；
- `_statistics([nan, nan, nan])` → `count=0, missing=0, invalid=3`；
- ⇒ 记录里出现 **3 个 `invalid`**，而不是 3 个 `missing`。

**含义**：写 `True`/`False` 到值槽不会谎报一个数字，但它会**谎报「我们尝试测量了三次，三次都拿到垃圾」**。这比 `None` 更远离事实，且 `MetricStatistics.complete` 同样为 `False`，所以两者在判据上不可区分——错只在数据叙事上。

**注意 `float(value)` 的宽松性**（【实测】）：`_number("0")` → `0.0`，`_number("1.0")` → `1.0`，`_number("false")` → `nan`，`_number([1])` → `nan`。⇒ 字符串**会被强转成数字**，不是一律拒绝。这是一个容易忽略的入参面。

### (c) 门槽收到 `0` / `1.0` / `False` / `"false"` 的确切结果

【代码】`:37-38`（**引文逐字**）：

```python
def _boolean(value: Any) -> bool | None:
    return bool(value) if isinstance(value, (bool, np.bool_)) else None
```

【实测】逐个：

| 输入 | `_boolean` 结果 | `report` 里的桶 | 是否等于「未测」 |
|---|---|---|---|
| `False` | `False` | `clear`（overlap）/ `rejected`（admission_ok） | 否 |
| `True` | `True` | `overlap` / `accepted` | 否 |
| `0`（int） | `None` | `unmeasured` | **是（静默）** |
| `1`（int） | `None` | `unmeasured` | **是（静默）** |
| `1.0`（float） | `None` | `unmeasured` | **是（静默）** |
| `0.0`（float） | `None` | `unmeasured` | **是（静默）** |
| `"false"` | `None` | `unmeasured` | **是（静默）** |
| `np.bool_(True)` | `True` | `overlap` / `accepted` | 否 |

⇒ **最危险的组合**：生产者用「`0` 表示没有重叠」这一 C 语言习惯写门槽，得到的是 **`unmeasured`（`insufficient_evidence`）而不是 `clear`（可能 `pass`）**。它不会造成假 `pass`，但会**静默丢失一次真实测量**——线上永远到不了 `online_verdict == "pass"`，而报告不会告诉你为什么。反方向也是：用 `1.0` 表示「有重叠」同样**不会**得到 `fail`，只是又一次 `unmeasured`。

**与值槽的对照**（同一份代码、同一个数字、相反后果）：`0` 在值槽被读成**真实测量 0.0**（谎报），在门槽被读成**未测**（丢测）。这就是「门槽的诚实是类型检查的副产品」。

### (d) 诚实性对照表

**(i) 值槽 —— 以 `obstacle_margin_m`（来源键 `min_obs_dist`）为例**

| 候选写法 | 评价器读到 | 数据桶 | 判定 | 是否谎报有量测 |
|---|---|---|---|---|
| 键不存在 | `None` | `missing` | `insufficient_evidence` | 否 |
| `None` | `None` | `missing` | `insufficient_evidence` | 否 |
| `0` | `0.0` | 有值 | 参与统计（可能 `pass`） | **是**——「净空正好 0 m」这个具体断言从未被测量过 |
| `1.0` | `1.0` | 有值 | 参与统计 | **是**（尤其致命：`1.0` 正是内核的 disabled sentinel，见 (f)） |
| `NaN` | `NaN` | `invalid` | `insufficient_evidence` | 否，但**把「没测」记成「测到垃圾」** |
| `0.0` + 旁路 `__status` 键 | **`__status` 被静默丢弃**，`0.0` 仍有值 | 有值 | 参与统计 | **是**（且状态信息丢失） |
| `0.0` + 具名字段（如 `TrackingStepData` 的既有字段） | 有值 + 状态可读 | 有值 | 需新判据 | 否（前提是消费方按状态字段解释） |

**(ii) 门槽 —— 以 `overlap` 为例**

| 候选写法 | 评价器读到 | 桶 | 判定 | 说明 |
|---|---|---|---|---|
| 键不存在 | `None` | `unmeasured` | `insufficient_evidence` | 诚实 |
| `None` | `None` | `unmeasured` | `insufficient_evidence` | 诚实 |
| `0` | `None` | `unmeasured` | `insufficient_evidence` | **静默丢测**（不是「无重叠」） |
| `1.0` | `None` | `unmeasured` | `insufficient_evidence` | **静默丢测**（不是「有重叠」） |
| `NaN` | `None` | `unmeasured` | `insufficient_evidence` | 静默丢测 |
| `False` | `False` | `clear` | 参与 `online` 判定 | 唯一能表达「无重叠」的写法 |
| `"false"` | `None` | `unmeasured` | `insufficient_evidence` | 静默丢测 |
| `0.0` + 旁路 `__status` 键 | 状态键被丢弃，`0.0` → `unmeasured` | `unmeasured` | `insufficient_evidence` | 双重丢失 |
| `0.0` + 具名字段 | `unmeasured` + 状态可读 | `unmeasured` | 需新判据 | 新增字段才行 |

**旁路键被丢弃的确切机制**（【代码】`tracking_evaluator.py:96-110`）：

```python
    aliases = { ...12 项... }
    values = {name: _number(get(key)) for name, key in aliases.items()}
```

`values` 只由**这张表的 12 个条目**生成；输入里任何别的键都不会进入 `TrackingStepData.values`。而 `values` 是 `dict[str, float | None]`——只能放数字或 `None`，**放不下状态枚举**。⇒ 「加一个状态键」这条路在 dict 输入下**静默失效**；只有调用方直接递交 `TrackingStepData` 实例时才能携带新的具名状态（那条路径在 `:87-94`，不经别名表）。

### (e) JAX 允许什么当 pytree leaf；`None` 能否在 jit 返回值里活下来

【JAX文档】pytrees 页（**引文**）：tree 工具「treat `None` as the absence of a pytree node, not as a leaf」，`jax.tree.leaves([None, None, None])` 给出 `[]`；要保留需传 `is_leaf`。

【实测】本机验证：

```
None in jit output: (Array([0., 0.], dtype=float32), None)   # 能返回，不会报错
leaves of (1,None): [1]                                      # None 不是 leaf
tree_structure (1,None): PyTreeDef((*, None))
tree_structure (1,0):    PyTreeDef((*, *))                   # 与 0 结构不同
tree_structure (1,nan):  PyTreeDef((*, *))                   # 与 0 结构相同
```

**对「内核必须按槽报告未测」的后果**——三条，全部【实测】确认：

1. **`None` 可以出现在 jit 的返回值里**，但它是一个**静态结构位**，不是数据。返回值里 `None` 所在的位置在**编译期**就固定了。
2. **因此 `None` 不能按步/按槽动态选择**：`return None if x > 0 else jnp.zeros(2)` 直接 `TracerBoolConversionError`（实测：`Attempted boolean conversion of traced array`）。⇒ 内核不能「这一步这条路没量测所以返回 None、下一步量测了所以返回数组」。**「未测」必须在结构上静态、在数据上另找载体**（例如一个独立的 `valid`/`active` 0-d 数组，正如 `qpax_warmstart.py:43` 的 `valid: jax.Array` 所做的）。
3. **`str` 不是合法的 jit 输出**：`return (x, "label")` → `TypeError: returned a value of type <class 'str'> at output component [1], which is not a valid JAX type`。另外【实测】`jnp.asarray(np.array([None,1.0],dtype=object))` → `TypeError: Dtype object is not a valid JAX array type`。⇒ **内核侧没有任何「对象/字符串」通道可以携带未测状态**，只能用数字位。

【JAX源码】`register_dataclass` docstring 补充了一条正面许可：「Note that `None` is a valid data field, as JAX recognizes this as an empty pytree.」

### (f) 仓库里已有的先例：节点对 `min_obs_dist` 的 `None`

【代码】`src/robot_safecontrol_moveit/oscbf_controller.py:562-565`（引文见 Q2(e) 的同一段）：注释明写内核的 `1.0` 是 **disabled sentinel**，「Do not report it as a measured metre of clearance/margin」，然后**用条件表达式把它换成 `None`**。这是本链条里**唯一**一处把「未启用/未测」正确表达的写法。

【文档】`docs/tracking_evaluation.md:38` 把这条上升为口径：

> 障碍或 ESDF 未启用时保留未测和 inactive 计数，**不把内核的占位距离当观测**。

【代码】facade 侧用 `active` 位表达同一件事（`jax_control_facade.py:733-745`）：`metrics` 每一项带 `'active': enabled`，未启用时 `'value': None`。⇒ **两条既有先例都是「`None`/`active` 位」，不是哨兵数。**

【文档】`docs/planning/oscbf-reuse/validation/2026-09-12-off15/REPORT.md:19` 记录了这次口径修正：

> 原来对未启用障碍返回的 `1.0` sentinel 断言已改为明确"未测"，避免把占位值当安全证据。

### 与直觉相反之处（Q4）

1. **直觉**：`None` 和 `NaN` 都是「没有有效值」，用哪个都行。
   **实际**：`missing` 与 `invalid` 是报告里的两个**不同**桶（`:182-184`），且 `trace_json()` 对它们分别编码（`null` vs `{"nonfinite":"nan"}`）。用 `NaN` 表达缺测＝把「没测」写成「测到了垃圾」。
2. **直觉**：门槽是布尔语义，`0`/`1` 应当能表达假/真。
   **实际**：`_boolean` 只认 `bool`/`np.bool_`（`:37-38`），`0`、`1`、`1.0`、`0.0`、`"false"` 全部 → `None` → `unmeasured`。用 `0` 表示「无重叠」**静默丢失一次真实测量**，线上因此永远到不了 `pass`，且报告不解释原因。
3. **直觉**：`False` 和 `0` 在 Python 里差不多。
   **实际**：在门槽里 `False` → `clear`（能参与判定），`0` → `unmeasured`（不能）。**这是本问里最容易踩的一个坑。**
4. **直觉**：加一个 `<键>__status` 旁路键最省事。
   **实际**：别名表是**封闭白名单**（`:96-110`），旁路键在 `step_from_result` 之后**不存在**，不报错、不保留、不进入样本记录。
5. **直觉**：`None` 进得去 `jax.jit` 的返回值，所以内核可以按槽报未测。
   **实际**：`None` 是**静态空子树**，位置在编译期固定，不能按步动态（`TracerBoolConversionError`，【实测】）；而且 `str`/object 数组一律非法。内核按槽报未测**只能靠数字位**（`valid`/`active` 0-d 数组）。

---

## Q5：跨步记忆与 jit 边界

### 结论

- `u_safe_prev` 的默认值**不是**遗留便捷写法，而是**被测试写明的契约**：它承担「哪些输入可缓存、哪些必须每步刷新」这条区分。【代码】`portable_oscbf/tests/test_jax_default_input_cache.py:1,16,25`。
- 它也是**唯一真正进入控制计算**的跨步记忆：生产 profile 的 `temporal_lambda = 0.2`（【代码】`config/oscbf_controller.yaml:37`）使 `_last_u_safe` 出现在 QP 目标的 P 与 q 两项里。**实际生效的两行**（【代码】`portable_oscbf/work/oscbf_velocity_config.py:216-218` 与 `:229-231`，该类是 cbfpy `P_config`/`q_config` 的实现）：

  ```python
          if self.temporal_lambda <= 0.0 or u_safe_prev is None:
              return p_task
          p_total = p_task + jnp.diag(self._temporal_weight_sq())      # :218
  ...
          if self.temporal_lambda <= 0.0 or u_safe_prev is None:
              return q_task
          return q_task - self._temporal_weight_sq() * u_safe_prev     # :231
  ```

  权重为 `_temporal_weight_sq() = temporal_lambda * temporal_wu**2`（`:206-207`）；`u_safe_prev` 在 h_args 里排第 8 位，由 cbfpy 经 `*h_args` 透传进这两个覆盖实现（`oscbf_velocity_config.py:139` 给出零向量默认）。
- `last_cbf_h_delta_norm` / `last_cbf_grad_delta_norm` 的**唯一活读者是一条通过中的测试**（`portable_oscbf/tests/test_jax_tracking_step.py:100-101`）。删它们＝**改验收信号**。
- `_last_cbf_h` / `_last_cbf_grad` 只在 facade 内部 + 一份研究脚本被读，无生产消费者。
- 两个 delta norm 与 `_last_cbf_h`/`_last_cbf_grad` 都在 **jit 边界之外**、在 host 上算（`np.asarray` + `np.linalg.norm`），结果落在 Python/NumPy 域；**把它们搬进记录不会**改变 `u_safe_prev` 那种「每步不同值」的缓存行为（jit 缓存键只看结构与 aval）。

### (a) `u_safe_prev` 默认值的确切行为

【代码】`portable_oscbf/work/jax_control_facade.py:868-869`（`_normalise_obstacle_inputs` 末尾，**引文逐字**）：

```python
        if u_safe_prev is None:
            u_safe_prev = self._last_u_safe
```

另有一条**缓存快捷路径**（【代码】`:811-814`，`_prepare_jax_inputs` 内）：

```python
                # JAX arrays are immutable. Copy only the mapping and always
                # refresh the previous control used by the rate constraints.
                return dict(self._default_jax_inputs, u_safe_prev=jnp.asarray(
                    self._last_u_safe if u_safe_prev is None else u_safe_prev))
```

⇒ 调用方省略时，取的是**当前** `self._last_u_safe`（不是缓存里的旧值）；缓存只复用**默认障碍/ESDF 输入**，`u_safe_prev` 每次现取。

**这是文档化契约还是实现便利？** 文档侧**没有**任何地方把它写成契约（全仓 `*.md` 对 `u_safe_prev` 的命中只有：`18-elastic-qp-admission.md:208` 的代码注释引用、以及两份第一轮研究文档）。**但测试把它写成了契约**——【代码】`portable_oscbf/tests/test_jax_default_input_cache.py`，模块 docstring（`:1`）逐字为：

> Default device inputs may be reused; live control/obstacle state may not.

三处断言（`:15`、`:16`、`:25`）：

```python
    assert second['obs_pos'] is first['obs_pos']                      # 编译输入被复用（同一对象）
    np.testing.assert_array_equal(second['u_safe_prev'], loop._last_u_safe)   # 每步现取
    ...
    np.testing.assert_array_equal(inactive['u_safe_prev'], loop._last_u_safe) # 障碍状态不缓存后仍现取
```

【实测】该模块 1 个测试，**通过**（与 `test_jax_tracking_step.py` 的 2 个一起跑：`3 passed, 3 skipped in 37.39s`）。

### (b) `last_cbf_h_delta_norm` / `last_cbf_grad_delta_norm` / `_last_cbf_h` / `_last_cbf_grad` 的读者清点

【实测】四个名字在 `src/`、`portable_oscbf/work/`、`portable_oscbf/tests/`、`tests/` 下的全部命中：

| 名字 | 写 | 读 | 分类 |
|---|---|---|---|
| `last_cbf_h_delta_norm` | `facade:220`（init）、`:768`（NaN 分支）、`:775-777` | **`portable_oscbf/tests/test_jax_tracking_step.py:100`**；`portable_oscbf/tests/test_jax_tracking_hard_stop.py:68`（stub 类属性，模块 skip） | **测试（其中 1 条活、1 条 skip）** |
| `last_cbf_grad_delta_norm` | `facade:221`、`:769`、`:778-780` | **`test_jax_tracking_step.py:101`**；`test_jax_tracking_hard_stop.py:69`（skip） | 同上 |
| `_last_cbf_h` | `facade:222`（`None`）、`:770`（`None`）、`:781`（`.copy()`） | `facade:776`、`:777`（**facade 自读**）；`docs/planning/oscbf-reuse/research/5-redundancy-evidence/compare.py:56` | facade 私有 + **研究脚本** |
| `_last_cbf_grad` | `facade:223`、`:771`、`:782` | `facade:779`、`:780`（**facade 自读**） | facade 私有 |

**生产读者：0。** `src/robot_safecontrol_moveit/` 里四个名字都不出现。

**活测试的断言内容**（【代码】`test_jax_tracking_step.py:94-101`，测试名 `test_tracking_fast_path_preserves_safe_command_without_h_gradient_telemetry`，【实测】**通过**）：

```python
    full_result = loop.tracking_step(**common)
    loop.collect_cbf_diagnostics = False
    fast_result = loop.tracking_step(**common)

    np.testing.assert_allclose(fast_result[1], full_result[1], atol=1e-10)
    assert fast_result[6]
    assert np.isnan(loop.last_cbf_h_delta_norm)
    assert np.isnan(loop.last_cbf_grad_delta_norm)
```

即断言「关掉 h/grad 遥测后**两个 delta norm 必须是 NaN**」——这条断言的正是在测 `facade:767-772` 的那个分支。

### (c) `controller_step_cache.py` 已建立的模式

【代码】`portable_oscbf/work/controller_step_cache.py`（共 66 行），接口：

| 项 | 行 | 内容 |
|---|---|---|
| `RobotStepCache` | `:12-20` | `@dataclass(frozen=True)`，字段 `q, T_all: Dict[str, np.ndarray], ee_pos, ee_rot, J_s, J_pos, J_full` |
| `_skew(v)` | `:23-28` | 私有辅助 |
| `point_jacobian_from_spatial(J_s, link_idx, point_world, n_joints=9)` | `:31-38` | 从缓存的空间雅可比推点线速度雅可比（**纯函数**） |
| `build_robot_step_cache(kin, q)` | `:41-66` | 单步构建：优先 `forward_kinematics_and_jacobian`（融合单遍），否则 FK + 空间雅可比；全部 `.copy()` 后装进 frozen dataclass |

**它建立的模式**：把「一步之内多处复用的、jit 不变的主机侧量」装进一个**frozen dataclass 值对象**，显式传参，不挂在模块/对象状态上。

**一个必须指出的现状**（【实测】全仓引用扫描）：只有 `point_jacobian_from_spatial` 被别的模块导入（`portable_oscbf/work/{point_cloud_obstacles.py:20, point_cloud_obstacles_dynamic.py:20, dynamic_obstacles.py:15}`）；**`RobotStepCache` 与 `build_robot_step_cache` 在模块外没有任何调用点**。

⇒ 【推断】这个文件作为「先例」是成立的（形态正确、有 docstring 说明意图），但作为「正在被用于生产的模式」不成立——它是**尚未接线**的基础设施。引用它作为设计先例时应当同时说明这一点。

### (d) 两个 delta norm 在 jit 边界的哪一侧

**都在边界之外**，且是 **NumPy/Python 域**：

【代码】`jax_control_facade.py:642-646`（jit 调用**已返回**之后）：

```python
        result = self._path_tracking_fn(...)          # :616-628，jitted 函数
        (q_next, u_safe, u_candidate, u_nom, err_6d, ee_pos, ee_rot, ...
         constraint_residuals, ee_pos_before) = result  # :629-641，纯位置解包
        self._update_qp_diagnostics(
            qp_ok, min_dist, min_esdf, rate_constraint_violation,
            rate_solver_slack, h_vals, cbf_grad,
            active_count, primal_residual, terminal_kkt_residual,
            terminal_kkt_accepted, dual_max, qp_iterations)   # :642-646
```

`_update_qp_diagnostics`（`:748-782`）里的关键三行（**引文**）：

```python
        h_now = np.asarray(h_vals)            # :773  ← device→host
        grad_now = np.asarray(cbf_grad)       # :774  ← device→host
        self.last_cbf_h_delta_norm = (
            0.0 if self._last_cbf_h is None
            else float(np.linalg.norm(h_now - self._last_cbf_h)))   # :775-777
```

⇒ `h_vals` / `cbf_grad` 是 jit 返回的 JAX 数组（`path_tracking_step` 里只有它们**没被** `np.asarray` 预处理，对比 `:674-705` 的其它字段），`np.asarray` 在 host 上把它们拉下来，`np.linalg.norm` 在 NumPy 里算，结果是 Python `float`。`_last_cbf_h` 存的是 `np.ndarray` 副本（`:781`）。

**两个副作用，与「搬进记录」直接相关**：

1. **这一步本来就有 device→host 同步**（`np.asarray` 两个数组）。也就是说，把 delta norm 变成记录字段**不会新增**一次同步，除非改成在 jit 内算。
2. **搬进记录不会改变 jit 缓存键**。【实测】`jax.jit` 的缓存键取决于参数树结构与 aval（shape/dtype），与**值**无关：

```
cache after 21 different-value calls (same shape): 1     # 21 个不同的 u_prev 值，仍只有 1 个缓存条目
cache after dtype change: 2                              # dtype 变了才新增
```

⇒ 若把 `_last_cbf_h` 作为**新的数组入参**传给 `_path_tracking_fn`，那会新增**一次**编译（新 treedef），此后仍是 1 个条目、逐步不重编；与 `u_safe_prev` 现在的处境完全相同。**逐位/逐步代价为零；一次性代价是一次 trace。**

### 与直觉相反之处（Q5）

1. **直觉**：`u_safe_prev` 的默认值是历史便利，删掉让它必填更干净。
   **实际**：它是**被测试写明的契约**（`test_jax_default_input_cache.py:1,16,25`，【实测】通过）。删默认值＝删掉「实时控制量每步刷新」这条区分，直接使两条断言失效。
2. **直觉**：`last_cbf_h_delta_norm` 无人用于生产，所以「只有测试读」≈安全可删。
   **实际**：它唯一的读者是一条**通过中的**测试（`test_jax_tracking_step.py:100-101`）。删它＝改验收信号（与 Q6 的处置建议冲突，见 Q6(b)）。
3. **直觉**：`_last_cbf_h` / `_last_cbf_grad` 是「jit 内部状态」。
   **实际**：它们在 jit **之外**、在 host 上以 NumPy 计算（`:773-782`）。jit 返回的是 `h_vals`/`cbf_grad` 数组，跨步记忆本身住 Python 属性里。
4. **直觉**：把跨步记忆搬进返回值会付出「每步重编译」的代价。
   **实际**：jit 缓存键只看结构与 aval，不看值（【实测】21 个不同值 → 仍是 1 个缓存条目）。代价是一次性的新 treedef 编译。
5. **直觉**：`controller_step_cache.py` 是本仓库「显式状态对象」的生产先例。
   **实际**：它**尚未接线**——`RobotStepCache` / `build_robot_step_cache` 在模块外零调用点，只有 `point_jacobian_from_spatial` 被用。

---

## Q6：死字段与归档模块的处置

### 结论

**分四类，不能一刀切**：

| 类 | 成员 | 处置 |
|---|---|---|
| **零读者、写的是常量** | `last_qp_warm_start_used` | 可删（只动 facade 两行）；**但它是恒真的正确常量**，不是「报假账」 |
| **读者只在 skip 模块 / stub 里** | `last_rate_constraint_violation`、`last_rate_slack` | 删不改变任何**被执行的**断言，但必须同步改两个 skip 模块的源码，否则留悬挂引用 |
| **读者只有研究脚本** | `last_delta_slack`、`_last_cbf_h` | 删需同步改 `5-redundancy-evidence/compare.py` |
| **读者是活测试** | `last_cbf_h_delta_norm`、`last_cbf_grad_delta_norm` | **删会改变验收信号**——必须单独立票 |
| **生产必需的跨步记忆** | `_last_u_safe` | **不能删**（Q5） |
| **无读者但不在本次范围** | `last_min_obs_dist` | 可删；删后与 `last_min_esdf_dist`（有生产读者）形成不对称，需说明 |

**`last_rate_slack` 是 `last_rate_constraint_violation` 的纯别名**，逐字赋值（`facade:758`），区别只在注释里自述的兼容意图。

**`qpax_warmstart.py` 从生产不可达**（facade 主动抛错拒绝该 flag），有**独立通过的测试 1 个**，且它的 SHA-256 被 **7 个已归档证据清单文件**记录。本轮的处置建议：**留着**。

### 全量写/读清点（11 个名字）

【实测】扫描范围 `src/`、`portable_oscbf/work/`、`portable_oscbf/tests/`、`tests/`，另附 `docs/` 命中。

| 名字 | 写 | 读 | 读者分类 |
|---|---|---|---|
| `last_qp_warm_start_used` | `facade:215`（init `False`）、`:762`（每次求解后 `False`） | 无 | **无读者**。`portable_oscbf/tests/test_jax_tracking_hard_stop.py:63` 是同名**局部/类属性**（stub，模块 skip） |
| `last_delta_slack` | `facade:467`（`step`）、`:583`（`tracking_step`）；**`__init__` 未初始化**；**`path_tracking_step` 不写** | `docs/planning/oscbf-reuse/research/5-redundancy-evidence/compare.py:55` | **研究脚本** |
| `last_rate_slack` | `facade:211`（init）、`:758`（＝ `last_rate_constraint_violation`） | `portable_oscbf/tests/test_jax_rate_limit.py:51`（模块 **skip**）；`test_jax_tracking_hard_stop.py:59`（stub，**skip**） | **测试（全为 skip）** |
| `last_rate_constraint_violation` | `facade:210`（init）、`:756-757` | `portable_oscbf/tests/test_jax_rate_limit.py:48`、`:69`、`:72`（模块 **skip**） | **测试（全为 skip）** |
| `last_min_obs_dist` | `facade:204`（init `1.0`）、`:754` | 无 | **无读者** |
| `last_min_esdf_dist` | `facade:205`（init `1.0`）、`:755` | `src/robot_safecontrol_moveit/perception_demo.py:307`；`portable_oscbf/tests/test_jax_esdf_cbf.py:93` | **生产（demo 工具）** + 测试（活，【实测】3 passed） |
| `last_cbf_h_delta_norm` | `facade:220`、`:768`（NaN）、`:775-777` | `portable_oscbf/tests/test_jax_tracking_step.py:100`（**活**）；`test_jax_tracking_hard_stop.py:68`（skip stub） | **测试（1 活 1 skip）** |
| `last_cbf_grad_delta_norm` | `facade:221`、`:769`、`:778-780` | `test_jax_tracking_step.py:101`（**活**）；`test_jax_tracking_hard_stop.py:69`（skip） | **测试（1 活 1 skip）** |
| `_last_cbf_h` | `facade:222`、`:770`、`:781` | `facade:776`、`:777`（facade 自读）；`compare.py:56` | facade 私有 + 研究脚本 |
| `_last_cbf_grad` | `facade:223`、`:771`、`:782` | `facade:779`、`:780` | facade 私有 |
| `_last_u_safe` | `facade:224`（init）、`:465`、`:581`、`:647` | `facade:814`、`:869` → `u_safe_prev`；`portable_oscbf/tests/test_jax_default_input_cache.py:13,16,25`（**活**）；`compare.py:36`（写） | **生产必需 + 活测试 + 研究脚本** |

**skip 状态核实**（【实测】）：

- `portable_oscbf/tests/test_jax_rate_limit.py:7` `pytestmark = pytest.mark.skip(reason="depends on newaxis ...")` —— 模块级，2 个测试全 skip。
- `portable_oscbf/tests/test_jax_tracking_hard_stop.py:9` `pytestmark = pytest.mark.skip(reason="depends on newaxis and HARD_STOP semantics ...")` —— 模块级，1 个测试 skip。
- 【实测】这四个模块一起跑的结果：`3 passed, 3 skipped in 37.39s`（3 passed = `test_jax_tracking_step` 的 2 个 + `test_jax_default_input_cache` 的 1 个；3 skipped = hard_stop 的 1 个 + rate_limit 的 2 个）。
- 另：`newaxis` 包在本机**不可导入**（【实测】`importlib.util.find_spec('newaxis')` → `None`），所以这两个 skip 模块的依赖本身不存在。

### (a) 哪些属性「读的人一个都没有」

**没有任何读者（facade 自己也不读）**：**2 个** —— `last_qp_warm_start_used`、`last_min_obs_dist`。

**只有 facade 自己读**：`_last_cbf_h`（`:776-777`）、`_last_cbf_grad`（`:779-780`）。

**其余都有模块外读者**，但「有读者」不等于「活契约」：`last_rate_constraint_violation` / `last_rate_slack` 的读者**全在 skip 模块**；`last_delta_slack` / `_last_cbf_h` 的模块外读者**只有研究脚本**。

**关于 `test_jax_tracking_hard_stop.py:55-70` 的那个 stub**：它是**测试替身，而且是给另一个代码库写的**。逐字看它的身份：

- 【代码】`:22-24`：`import newaxis.tracking_execution as tracking` / `from newaxis.hardware_stop_execution import HardwareStopExecutionMixin` / `from newaxis.tracking_execution import TrackingExecutionMixin`。
- 【代码】`:57-69`：`class _JaxLoop:` 依次列出 `last_rate_slack`、`last_qp_candidate`、`last_qp_active_count`、`last_qp_iterations`、`last_qp_warm_start_used`、`last_qp_primal_residual`、`last_qp_terminal_kkt_residual`、`last_qp_terminal_kkt_accepted`、`last_qp_dual_max`、`last_cbf_h_delta_norm`、`last_cbf_grad_delta_norm`。
- 【代码】`:72-83`：`path_tracking_step` 返回 `SimpleNamespace(...)`，只带 8 个字段，**不是** `JaxPathTrackingResult`。

⇒ 【推断】这个 `_JaxLoop` 是 **`newaxis` 运行器的替身**，不是 `work.jax_control_facade.JaxControlLoop` 的替身。它的属性列表说明的是「`newaxis` 的 mixin 期望这些属性名」，**不是**「本仓库的 facade 必须提供这些属性」。因此：

- **属性名对这条 skip 测试是承重的**（对 `newaxis` 侧），
- **但对本仓库的 facade 不承重**：删本仓库的 `last_qp_warm_start_used` **不会**让这个 stub 失效（它自己定义了同名属性），也**不会**让该测试从 skip 变 fail——因为它本来就不运行，且 `newaxis` 不存在。

### (b) 逐个属性的删除代价（以及必须点名的验收信号变化）

| 拟删属性 | 必须改的文件 | 是否改变测试断言 |
|---|---|---|
| `last_qp_warm_start_used` | `portable_oscbf/work/jax_control_facade.py:215,762` | **否**。无活读者；stub 自带同名属性；`newaxis` 模块 skip |
| `last_min_obs_dist` | `portable_oscbf/work/jax_control_facade.py:204,754` | **否**。无读者。代价是**不对称**：孪生的 `last_min_esdf_dist` 有生产读者（`perception_demo.py:307`），删一半会让两者形态分叉 |
| `last_delta_slack` | `portable_oscbf/work/jax_control_facade.py:467,583`；`docs/planning/oscbf-reuse/research/5-redundancy-evidence/compare.py:55` | **否**（研究脚本不是验收）。**但它本身是一个既有缺陷**：`__init__` 不初始化、且生产入口 `path_tracking_step` 不写它（见下） |
| `last_rate_slack` | `portable_oscbf/work/jax_control_facade.py:206-211,758`；`portable_oscbf/tests/test_jax_rate_limit.py:51`；`portable_oscbf/tests/test_jax_tracking_hard_stop.py:59` | **否（不改变被执行的结果，因为模块 skip）**，但会**删掉一条兼容承诺**（`:206-209` 的注释明写「retained as a public compatibility alias」）。⚠ **源码层面的断言会被移除**，应作为「契约收回」单独记录，而不是当清理 |
| `last_rate_constraint_violation` | 同上 + `test_jax_rate_limit.py:48,69,72` | 同 `last_rate_slack`。⚠ 注意 `:72` 把它作为 `classify_control_safety_state(rate_slack_rad_s=...)` 的输入——那是 `newaxis` 侧的接口，本仓库无法验证 |
| `last_cbf_h_delta_norm` | `portable_oscbf/work/jax_control_facade.py:220,768,775-777`；`portable_oscbf/tests/test_jax_tracking_step.py:100` | ✅ **是——改验收信号**。⚠⚠ 见下方专门论证 |
| `last_cbf_grad_delta_norm` | `portable_oscbf/work/jax_control_facade.py:221,769,778-780`；`portable_oscbf/tests/test_jax_tracking_step.py:101` | ✅ **是——改验收信号**。⚠⚠ 同上 |
| `_last_cbf_h` | `portable_oscbf/work/jax_control_facade.py:222,770,776,777,781`；`compare.py:56` | 否（facade 私有 + 研究脚本）。代价：`last_cbf_h_delta_norm` 失去数据来源，二者必须成组处置 |
| `_last_cbf_grad` | `portable_oscbf/work/jax_control_facade.py:223,771,779,780,782` | 否（纯 facade 私有） |
| `_last_u_safe` | —— | **不可删**。见 Q5(a)：它是唯一进入控制计算的跨步记忆（`oscbf_velocity_config.py:216-218`、`:229-231` 的时序近端项），且被 3 条活断言检查 |

**⚠⚠ 点名：`last_cbf_h_delta_norm` / `last_cbf_grad_delta_norm` 的删除会改变验收信号**

- 断言位置：【代码】`portable_oscbf/tests/test_jax_tracking_step.py:100-101`，测试名 `test_tracking_fast_path_preserves_safe_command_without_h_gradient_telemetry`。
- 断言内容：关掉 `collect_cbf_diagnostics` 后，两个 delta norm **必须为 `NaN`**。它测的是 `facade:767-772` 的分支。
- 【实测】该测试在本次核验中**通过**（与 ESDF 模块分别跑：`3 passed` / `3 passed`）。
- **反对删除的理由**：这条断言是「快路径不带遥测」这一行为的**唯一**回归保护。删掉属性等于删掉断言，从此没有人检查「关掉遥测后有没有偷偷留下旧值」。按 `LESSONS_LEARNED.md:179`（迁移断言的方法论，见【文档】第一轮文档引用）要求，**不能用 skip 或删断言来掩埋**；若要删，必须先把等价断言改挂到新记录上（例如「记录里对应槽位是 `valid=False`」）。
- **支持删除的理由**：`last_cbf_h_delta_norm` 的初始值 `0.0`（`:220`）与「首次为 0.0」（`:776`）语义不区分「没算过」和「变化量为 0」，这本身是一个弱设计；把状态收进记录时顺带修掉它是合理的。
- **结论**：**应与记录改造合并成同一票、并显式换掉断言，而不是在「清死字段」里顺手删。**

**`last_delta_slack` 的既有缺陷（独立于删除）**：

- 未在 `__init__` 初始化（【实测】`grep` 全仓，`:467`、`:583` 是仅有的两处写，`__init__` 段 `:203-226` 里没有它）。
- 【代码】`path_tracking_step`（`:589-705`）只写 `_last_u_safe`（`:647`）、`last_qp_candidate`（`:648`）、`last_path_metrics`（`:650`），**不写 `last_delta_slack`**；而 `path_tracking_step` 是生产唯一入口（`oscbf_controller.py:536` 调用）。
- ⇒ 【推断】在生产路径上，`self._loop.last_delta_slack` 要么 `AttributeError`，要么是一个**更早某次测试调用 `step`/`tracking_step` 留下的陈旧值**。这是一个真实存在的潜伏缺陷：侧信道的**「哪个入口写哪些属性」并不一致**。同一个量在返回值里是可靠的（`JaxPathTrackingResult.delta_slack`），在侧信道上不可靠。**这正是「`last_*` 不是接口，是缓存」的最强证据。**

### (c) `qpax_warmstart.py` 的生产可达性与测试

| 问题 | 实测答案 |
|---|---|
| 生产可达吗？ | **不可达**。`portable_oscbf/work/qpax_warmstart.py` 在 `work/` 内**零引用**；唯一的 facade 入口被主动封死：【代码】`jax_control_facade.py:145-148` `if self.qp_warm_start: raise ValueError('the custom qpax PDIP warm-start path is archived; production JAX control uses qpax baseline solves')`。另有测试专门锁定这个封禁：【代码】`portable_oscbf/tests/test_jax_qp_warmstart_integration.py:10-11` `with pytest.raises(ValueError, match='archived'):` |
| 有独立测试吗？通过吗？ | **有，且通过**。【代码】`portable_oscbf/tests/test_qpax_warmstart.py`（1 个测试，`:8-32`，断言「warm 解与 cold 解一致，且 warm 迭代次数更少」）。【实测】与 integration 一起跑：`2 passed in 0.37s` |
| 删除它会让测试数怎么变？ | **正好少 1 个**。`test_qpax_warmstart.py` 全模块会在 import 阶段失败（`:13 from work.qpax_warmstart import ...`）；`test_jax_qp_warmstart_integration.py` 不受影响（它只 import `JaxControlLoop`，且断言的是**拒绝**该 flag）。另需同步改【代码】`portable_oscbf/README.md:37`（`│   ├── qpax_warmstart.py          # PDIP warm-start 适配器`） |
| 该删还是该留？ | **本轮留着。** 三条理由：(1) 它是「跨步记忆显式化」的**唯一已落地先例**（`WarmStartState(NamedTuple)` 带 `valid` 位，`qpax_warmstart.py:31-44`），对本轮设计有直接参考价值；(2) 它的 SHA-256 被**已归档证据清单**记录，删它会与既有证据的身份记录冲突——【实测】命中 **7 个文件**：`docs/planning/oscbf-reuse/validation/2026-09-12-off15/controller-runtime-snapshot.json:448`、`.../2026-09-12-off15-review-fixes/controller-runtime-snapshot.json:449`、`docs/planning/oscbf-reuse/research/3-latency-evidence/cpu/metadata.json:39,93`、`.../3-latency-evidence/gpu/metadata.json:39,93`、`docs/planning/oscbf-reuse/research/5-redundancy-evidence/metadata.json:57`、`docs/planning/oscbf-reuse/research/offline-latency-20260911/cpu.json:42,107`、`.../offline-latency-20260911/gpu.json:42,107`；(3) 删除是一个**独立的契约收回**动作，与「一步记录」的结构改动没有因果关系，混在一轮里会让评审无法分辨哪一处改了什么 |

**顺带核验到的一件事（与 Q5 的一处引用错误有关）**：`portable_oscbf/work/qpax_solver.py`（137 行以上，手写 QP 求解器）在本仓库**没有任何导入者**——【实测】在 `src/`、`portable_oscbf/`、`tests/` 下检索 `qpax_solver`，唯一命中是它自己的 docstring 首行（`qpax_solver.py:3`）；`portable_oscbf/tests/` 下也没有 `test_qpax_solver*.py`。

- 因此**上一轮文档把 `qpax_solver.py:148-191` 当作 `u_safe_prev` 在生产中的作用点是不成立的**——那个文件的时序近端项是**死代码**。真正生效的是 `oscbf_velocity_config.py:206-231`（本文件 Q5(a) 已改正）。
- 与 `qpax_warmstart.py` 形成对照：两者都从生产不可达，但 `qpax_warmstart.py` **有 1 个通过的独立测试**，而 `qpax_solver.py` **连测试都没有**。同为「归档模块」，两者的处置证据强度不同——`qpax_solver.py` 更接近纯死代码，但它不在本轮 Q6 的询问范围内，故本文只记录事实、不给删除建议。

### (d) `last_rate_slack` 是不是纯别名

**是。** 【代码】`jax_control_facade.py:756-759`（**引文逐字**）：

```python
        self.last_rate_constraint_violation = max(
            0.0, float(rate_constraint_violation))
        self.last_rate_slack = self.last_rate_constraint_violation
        self.last_rate_solver_slack = max(0.0, float(rate_solver_slack))
```

第二行是**直接赋值**，同一个值写两遍。**没有任何东西区分它们**:`last_rate_slack` 没有独立的数据来源、没有独立的钳制、没有独立的类型。

**唯一区分在文字里**：【代码】`jax_control_facade.py:206-209`（**引文逐字**）：

```python
        # ``last_rate_slack`` is retained as a public compatibility alias for
        # the actual rate-constraint relaxation.  qpax's raw interior-point
        # slack is exposed separately because it is not itself a command
        # violation.
```

⇒ 注释自述它是「公共兼容别名」，并把**真正不同的那一个**（`last_rate_solver_slack`，qpax 原始内点松弛）明确分开。测试也照这个区分断言（【代码】`test_jax_rate_limit.py:51` vs `:52`：前者等于物理松弛、后者 `last_rate_solver_slack >= -1.0e-6`）。所以：

- `last_rate_constraint_violation` 与 `last_rate_slack` —— **同一个量，两个名字**；
- `last_rate_solver_slack` —— **另一个量**，不是别名。

### 与直觉相反之处（Q6）

1. **直觉**：没人读的字段就是死字段，删掉无损。
   **实际**：`last_qp_warm_start_used` 确实是零读者，但它是一个**恒真的正确常量**——因为 `qp_warm_start=True` 会被 facade 拒绝（`:145-148`），所以 `last_qp_warm_start_used` **不可能**为 `True`。它不是「报了假账」，是「携带零信息」。这个区别决定处置语气：删它不掩盖任何缺陷。与之相反，`last_delta_slack` 是**真缺陷**（未初始化 + 生产入口不写）。
2. **直觉**：`test_jax_tracking_hard_stop.py` 的 `_JaxLoop` stub 镜像了 facade 的属性，所以这些属性名是承重的。
   **实际**：那个 stub 是给 **`newaxis`** 写的（`:22-24` `from newaxis.hardware_stop_execution import ...`），而 `newaxis` 在本机**不可导入**，模块整体 skip。它的属性列表约束的是 newaxis 侧，**不是**本仓库 facade。删本仓库的 `last_qp_warm_start_used` 不会让它失效。
3. **直觉**：`last_rate_slack` 有测试断言（`test_jax_rate_limit.py:51`），所以删它要改验收信号。
   **实际**：那个模块 `pytestmark = pytest.mark.skip`（`:7`），断言**从不执行**。真正的活断言只有 `test_jax_tracking_step.py:100-101`（以及 `test_jax_default_input_cache.py:16,25`）。
4. **直觉**：`qpax_warmstart.py` 既然生产不可达，就是死代码，refactor 轮顺手删掉。
   **实际**：它有 1 个**通过**的独立测试，且它的 SHA-256 出现在 **7 份已归档清单**里（含 OFF-15 运行快照与两份 latency 证据）。删它会让测试数 −1 并让既有证据的身份记录与工作树不一致。**留着。**
5. **直觉**：侧信道是「有人在用的公开接口」。
   **实际**：21 个属性里只有 **3 个**有 facade 之外的生产读者（`last_min_esdf_dist`、`last_path_metrics`、`last_qp_primal_residual`），其中**只有 `last_qp_primal_residual` 在真正的控制路径上**（`oscbf_controller.py:559-561`），另两个在 `perception_demo.py`（demo 工具）。删侧信道的代价主要按「改哪些测试」算，不是按「破坏生产」算。
6. **直觉**：`u_safe_prev` 参与生产 QP 的那段代码在 `work/qpax_solver.py`（第一轮文档如此引用）。
   **实际**：`qpax_solver.py` **没有任何导入者**（连测试都没有），那段是死代码。真正生效的是 `work/oscbf_velocity_config.py:206-231`。**结论不变**（`u_safe_prev` 确实进入 QP 目标），但引用位置必须先纠正，否则「它参与控制」这句会被错误地定位到一个不存在的执行路径上。

---

## 未核验清单

| # | 未核验项 | 缺什么 |
|---|---|---|
| 1 | OFF-09 / OFF-13 的**实现进度** | 本文只引用 `off02_obb_geometry_admission_spec.md:11/:136-155/:268-283` 的**职责边界**与 `handoffs/{44,46}` 的决策记录，没有核实这两票是否已开工、是否有代码。未做：检索 tracker（`gh` 未使用）、检索它们的目标文件是否已存在 |
| 2 | `obstacle_clearance_m` 的**正式归属票** | 文档只写「调用方显式提供」（`docs/tracking_evaluation.md:31`）并把接入点指向「OFF-09/OFF-13 的后续结果」（`:80`）。没有任何文档把这一指标的**生产者**指定给一张票。此项为推断，未找到一手指定 |
| 3 | `admission_ok` 的**决策进度** | `18-elastic-qp-admission.md §10` 是 2026-09-12 的待决清单；本文未核验此后是否有新的用户决议落盘（未检索 `docs/adr/` 之外的会话记录、未联网查 tracker） |
| 4 | `last_qp_ok`、`last_qp_candidate`、`last_qp_active_count`、`last_qp_iterations`、`last_qp_dual_max`、`last_qp_terminal_kkt_residual`、`last_qp_terminal_kkt_accepted`、`last_rate_solver_slack`、`last_path_metrics` 的读者普查 | 题目 Q6 只列了 11 个名字，本文严格只对这 11 个做全量写/读清点。其余 10 个只做了「facade 之外的生产读者」定向核验（`last_min_esdf_dist`←`perception_demo.py:307`、`last_path_metrics`←`perception_demo.py:387`、`last_qp_primal_residual`←`oscbf_controller.py:560`），**未做逐个全量** |
| 5 | `docs/planning/oscbf-reuse/handoffs/44-offline-tracking-evaluation.md` 之外，**#19** 及其上游票对 `online_verdict` 的期望 | 本文只核验到「生产链只产出 `insufficient_evidence`」（`controller-tracking.json:290`）与「无文档期望生产达到 `pass`」。**未检索 tracker 上的 #19 讨论**，可能有未落盘的期望 |
| 6 | 全仓测试总数与「删除 X 后测试数变化」的完整影响面 | 本文的测试数结论基于**定向执行**（`test_qpax_warmstart` + `test_jax_qp_warmstart_integration`：`2 passed`；`test_jax_tracking_step` + `test_jax_default_input_cache` + 两个 skip 模块：`3 passed, 3 skipped`；`test_jax_esdf_cbf`：`3 passed`）。**未运行 `run_all_tests.sh`**，因此不能给出全仓总数，也无法排除某处间接影响 |
| 7 | 研究脚本 `docs/planning/oscbf-reuse/research/5-redundancy-evidence/compare.py` 是否可复现/是否仍可运行 | 只核验了它在 `:55`/`:56` 读 `last_delta_slack`/`_last_cbf_h`，**未运行它**，也未核实它依赖的其它私有属性是否仍在 |
| 8 | 「把记录定义搬进 `portable_oscbf/work/`」对**打包/安装**的影响 | 【未核验】`setup.py` / `package.xml` / `production_config.py` 的 share 目录布局细节，以及 `portable_oscbf/work/` 是否作为独立包被安装（只核验了 `oscbf_trajectory.bootstrap_portable` 与 `perception_bridge.py:76-87` 的 `sys.path` 注入）。新增一个模块是否需要改打包清单，**未核验** |
| 9 | `mock`/动态属性访问的盲区 | 本文的所有「读者清点」都是**按属性名静态检索**（`grep` + AST）。通过 `getattr(obj, "last_" + suffix)`、`__dict__` 遍历或 logging 反射消费的情况**不会被统计到**。本次未在仓库中发现这种写法，但方法本身有此已知盲区 |
| 10 | jit 缓存键的实测范围 | 【实测】只在一个 2 参数的玩具函数上验证了「值不影响缓存条目数、dtype 影响」。**未在真实内核 `_path_tracking_fn` 上**验证「新增一个 `_last_cbf_h` 数组入参只新增一次编译」这一推论 |
| 11 | 改动后 `_last_cbf_h` 是否会引入 host→device 往返的**性能**代价 | Q5(d) 只论证了「缓存键不变」与「本来就有一次 device→host 同步」。**未测**把它作为 jit 入参所增加的 host→device 传输对 50 Hz/100 Hz 预算的影响 |

### 本文的核验方式（供复核）

- 只读命令：`grep`、`find`、`sed -n`、`python3 -c`（AST/正则/集合运算）、`git status --short`。
- 测试执行：仅 3 组**定向** pytest，命令均带 `PYTHONDONTWRITEBYTECODE=1` 与 `-p no:cacheprovider -o addopts=""`，不写仓库文件；JAX 平台限 `cpu`。**未运行** `run_demo.sh`、`run_all_tests.sh`、ROS、MuJoCo、CAN 或硬件。
- JAX/Python 行为结论均来自本机实测（jax 0.6.2；`jax.tree_util.register_dataclass.__doc__` 作为【JAX源码】引用）。
- 仓库已有的 11 项未提交改动未被触碰（`git status --short` 在本次核验前后一致；本文是该状态下的新增文件）。
