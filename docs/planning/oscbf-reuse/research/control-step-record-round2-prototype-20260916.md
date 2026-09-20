# 一份步记录该带什么：第二轮问题的原型答案

日期：2026-09-16。
配套产物：`portable_oscbf/scripts/prototype_round2_seam.py`（一次性原型，不属于生产代码）、同目录 `prototype_round2_seam.json`（机器可读结果）。
复现：

```bash
python3 portable_oscbf/scripts/prototype_round2_seam.py
```

原型不修改仓库任何文件，只写一个 JSON。所有计数都由 `ast` 解析源码得出，没有一个名字是手抄的；A 部分调用的是**真实的** `robot_safecontrol_moveit.tracking_evaluator`，不是它的模型。

本题要回答的是第一轮评审之后的六个结构问题。原型与第一轮那份的区别：第一轮问「记录里有什么」，这一轮问「缺测怎么写、四个读者各自该读什么、要删的东西有谁在读」。

---

## A. 「未测」的写法：哪一种不撒谎

用真实评价器跑 7 种写法。每一步都送 3 个样本，值位取 `obstacle_margin_m`（来源键 `min_obs_dist`），门位取 `admission_ok` / `overlap`。

| 写法 | 值位被读成 | 门位被读成 | `online_verdict` | 是否谎报有量测 |
|---|---|---|---|---|
| 键不存在 | missing (None) | unmeasured | insufficient_evidence | 否 |
| 写 `None` | missing (None) | unmeasured | insufficient_evidence | 否 |
| 写整数 `0` | **measured (0.0)** | unmeasured | insufficient_evidence | **是（`obstacle_margin_m`）** |
| 写浮点 `1.0` | **measured (1.0)** | unmeasured | insufficient_evidence | **是（`obstacle_margin_m`）** |
| 写 `NaN` | **invalid (NaN)** | unmeasured | insufficient_evidence | 否 |
| 写 `False`（真值「否」） | measured (0.5) | False | **fail** | 是（门位） |
| 哨兵 `1.0` + `<键>__status` | **measured (1.0)** | unmeasured | insufficient_evidence | **是（`obstacle_margin_m`）** |

四条结论，都是这次跑出来的而不是推的：

1. **`None` 是唯一在值位上诚实的标量写法。** 评价器把 `None` 计入 `missing`，NaN 计入 `invalid`（`tracking_evaluator.py:183-184`：`missing = sum(v is None ...)`，`finite` 只收 `math.isfinite`）。这两个是不同的桶：「没测」和「测了但不是有效数」。
2. **`0` 和 `1.0` 在值位上是谎报。** `_number` 对它们返回真实浮点，于是报告里出现一个从未被量测过的读数。`0.0` 尤其危险——它是「净空正好为零」这个很具体的物理断言的合法取值。
3. **NaN 不是「未测」，而是「无效」。** 它不会撒谎，但会把缺测记成另一件事，而 `missing`/`invalid` 在报告里是两个不同的计数。用它表达缺测，等于把「我们没测」写成「我们测到了垃圾」。
4. **门位对类型很挑：`0`、`1.0`、`NaN` 都会被读成 unmeasured。** 因为 `_boolean` 只认 `bool`/`np.bool_`，其余一律 `None`（`tracking_evaluator.py:37-38`）。也就是说门位的「诚实」是**类型检查的副产品**，不是设计出来的契约：同一个哨兵数在值位上是谎报、在门位上是缺测。

### 一个 `__status` 后缀键根本到不了阅读方

原型实测：把一个 `<键>__status` 键放进节点那种 dict，`step_from_result` 转换后**它不存在**（`status_key_present_in_converted_values: False`），任意未登记键同样消失。原因在 `tracking_evaluator.py:110`：

```python
values = {name: _number(get(key)) for name, key in aliases.items()}
```

别名表是一个**封闭白名单**：只从输入里读它列出的 12 个键。所以「再开一个状态键」这个最省事的方案在节点这条路上会**静默失效**——不是报错，是无声丢弃。同一个键只有在调用方直接递交 `TrackingStepData` 实例时才存活（`arbitrary_key_survives_a_TrackingStepData: True`），而控制节点从不这么做。

**因此缺测状态必须以具名字段进入阅读方**（像 `admission_ok` 那样是 `TrackingStepData` 的一个字段），不能靠字典里的旁路键。这是第四问最关键的一条实现约束。

### 仓库里已经有一个正确答案，只是在错的那一层

控制节点对 `min_obs_dist` 已经这么写了（`oscbf_controller.py:560-563`）：

```python
# The kernel uses 1.0 as the disabled-obstacle sentinel. Do not
# report it as a measured metre of clearance/margin.
"min_obs_dist": float(result.min_obs_dist) if obs_kwargs and np.any(
    np.asarray(obs_kwargs.get("obs_enabled", [])) > 0.5) else None,
```

而内核侧照旧把 `1.0` 这个哨兵当普通浮点往回传（`jax_kernel_factory.py:226-228`，`min_obs_dist = jnp.where(..., 1.0)`），facade 也照旧原样存进 `last_min_obs_dist`（`jax_control_facade.py:754`）。

也就是说：**判据已经存在、写法已经正确（`None`），但它是消费者自己回头查 `obs_enabled` 才得出的**。改成生产者报告状态、消费者读状态，能让现有生产行为逐位不变，同时把这段重复判断从节点移走。这比新发明一套编码强。

---

## B. facade 上 21 个 `last_*` 属性，谁在读

按「facade 自己读」「facade 之外的生产读」「测试读」「研究脚本读」四类分别统计。分开的理由：facade 读自己的状态是私有实现，不是消费者；把两者混在一起会把「有外部契约」放大成假象。

| 属性 | facade 自读 | 生产读 | 测试读 | 研究脚本读 |
|---|---|---|---|---|
| `last_min_obs_dist` | 0 | 0 | 0 | 0 |
| `last_qp_ok` | 0 | 0 | 0 | 0 |
| `last_qp_warm_start_used` | 0 | 0 | 0 | 0 |
| `last_delta_slack` | 0 | 0 | 0 | 1 |
| `_last_cbf_grad` | 2 | 0 | 0 | 0 |
| `_last_cbf_h` | 2 | 0 | 0 | 1 |
| `_last_u_safe` | 2 | 0 | 3 | 1 |
| `last_rate_constraint_violation` | 1 | 0 | 3 | 0 |
| `last_cbf_grad_delta_norm` | 0 | 0 | 1 | 0 |
| `last_cbf_h_delta_norm` | 0 | 0 | 1 | 0 |
| `last_rate_slack` | 0 | 0 | 1 | 0 |
| `last_qp_dual_max` | 0 | 0 | 1 | 0 |
| `last_qp_active_count` | 0 | 0 | 1 | 1 |
| `last_qp_iterations` | 0 | 0 | 1 | 4 |
| `last_qp_terminal_kkt_accepted` | 0 | 0 | 1 | 1 |
| `last_qp_terminal_kkt_residual` | 0 | 0 | 1 | 1 |
| `last_rate_solver_slack` | 0 | 0 | 2 | 0 |
| `last_qp_candidate` | 0 | 0 | 2 | 0 |
| `last_min_esdf_dist` | 0 | **1** | 1 | 0 |
| `last_path_metrics` | 0 | **1** | 0 | 0 |
| `last_qp_primal_residual` | 0 | **1** | 1 | 3 |

汇总：

| 类别 | 数量 | 成员 |
|---|---|---|
| **任何地方都没人读**（写了就烂在那） | **3** | `last_min_obs_dist`、`last_qp_ok`、`last_qp_warm_start_used` |
| 只有研究脚本读 | 1 | `last_delta_slack` |
| 有 facade 之外的生产读者 | **3** | `last_min_esdf_dist`（`perception_demo.py:307`）、`last_path_metrics`（`perception_demo.py:387`）、`last_qp_primal_residual`（`oscbf_controller.py:560`） |
| 其余 | 14 | 只被 facade 自己或测试读 |

21 个属性里，**只有 3 个有 facade 之外的生产读者，而其中只有 1 个（`last_qp_primal_residual`）在真正的控制路径上**；另外两个在 `perception_demo.py`。有 12 个的唯一存在理由是测试会断言它们，另有 3 个任何人都没读过。

这条直接影响第二轮第五、六问：侧信道不是「有人在用的公开接口」，而主要是**测试夹具**。删它的代价要按「要改哪些测试」来算，不是按「会不会破坏生产」来算。

---

## C. 一份数据要过几道改名

由 AST 数出来的四个界面规模与改名点：

| 项 | 数量 |
|---|---|
| 内核位置元组槽位 | 42 |
| 记录（`JaxPathTrackingResult`）字段 | 30 |
| 节点手写 dict 键 | 29 |
| 评价器别名表条目 | 12 |
| **内核 → 记录 改名** | **14** |
| **记录 → 节点 改名** | **0** |
| **记录 → 评价器 改名** | **4** |
| 合计改名点 | **18** |

三处细节值得单独说：

- **内核→记录的 14 处**就是 facade 拆元组得到的局部名与记录字段名不一致的地方，例如 `min_dist`→`min_obs_dist`、`next_state_np`→`path_state`。这一层如果让内核直接返回记录，14 处全部消失。
- **记录→节点是 0 处改名。** 节点那 29 个键里，25 个是同名字段的直接透传，2 个是从同名字段派生（`path_progress_m ← float(result.path_state[0])`、`min_obs_dist ← obs_enabled 条件式`），2 个是纯 host 侧（`projection_before_m`、`qp_primal_residual`）。所以「删掉手写 dict」不需要任何改名，只需要保留这 4 个派生/宿主侧项。
- **记录→评价器的 4 处**是：`cross_track_m ← measured_cross_track_error_m`、`online_cross_track_m ← cross_track_error_m`、`obstacle_margin_m ← min_obs_dist`、`source_time_s ← reference_source_time_s`。这 4 处是**命名取舍**，不是必须的翻译：记录字段若采用评价器的名字，它们就归零；否则应留一张有文字理由的 4 行映射表，而不是现在这张 12 行、真假混杂的表。

还有一个数字对第二问有用：记录有 **4 个字段节点从不读**——`ee_pos_before`、`posture_reference`、`reference_omega_per_m`、`u_nom`。它们存在是为了别的消费者，所以「记录是给节点用的」这个假设不成立。

**`qp_primal_residual` 是侧信道上唯一被控制路径读到的值**（`oscbf_controller.py:560` 读 `self._loop.last_qp_primal_residual`）。记录里没有这个字段。所以：如果决定取消侧信道，`qp_primal_residual` 必须先搬进记录；否则节点会失去它。

---

## D. 第一轮的第二轮建议里，被本次核验推翻的两条

### D.1 Q1：「本轮接上 `admission_ok`」不成立

第一轮给出的建议是「本轮接上 `admission_ok`，另两个如实缺测」。本次核验发现这条**不应该采纳**，理由有三层，全部来自仓库已有文档：

1. **「准入与重叠缺测 ⇒ 证据不足」是写明了的政策**，不是缺陷。`docs/tracking_evaluation.md:8`：「`online_verdict`：所记录的准入与重叠判据。任一拒绝/重叠/关键故障会保留失败；**缺少准入或重叠记录时为证据不足，不能从 `qp_ok` 推断**。」
2. **生产方属于后续票，而且边界被明确划开。** `docs/specs/off02_obb_geometry_admission_spec.md:11`：「输出必须区分点态事实、区域覆盖和未知状态，供后续 OFF-09 组合准入；**OFF-02 本身不拥有锁存、恢复或命令发送职责**。」同文 `:150`：`overlap` 的处置是「拒绝并由 **OFF-09** 锁存」。而 `:155` 还有一条禁令：「现有 `TrackingStepData.overlap: bool | None` 只可作为诊断投影，**不得作为该准入契约本身**。」——把当前 `overlap` 字段直接升格成准入字段会违反规格。
3. **`admission_ok` 缺的不是数据而是判据，而判据是安全政策。** `docs/planning/oscbf-reuse/research/18-elastic-qp-admission.md` 已经查明：该量所需的 `primal_residual` 早已算出、今天只进诊断（`:266-268`），但要用它必须定 `eps` 的数值与单位。同文 `§6.3` 指出松弛量的量纲随行组变化（关节行 rad/s、障碍行 m/s，且被 `α∈{10,1.5}` 缩放），`§6.4` 给出干净工况的噪声地板约 1.9e-9 且**明确标注这是单机单次样本、不是上界**，`§10` 则把 `eps` 数值、按行组是否分组、失败后是冻结还是锁存四项一并列为**待用户决议的政策项**。

结论：在这一轮接 `admission_ok` 等于把一项未定的安全政策偷偷塞进一次结构性重构，并且会改变 fail-closed 行为。**正确做法是三个都不接**，让记录如实写明缺测；同时把「线上 `pass` 不可达」的验收门改为「信息不丢失」，而不是「结论变 pass」。

### D.2 Q5：「顺手把 `u_safe_prev` 默认值删掉」不成立

第一轮的建议是「取消 `u_safe_prev` 的默认值，让它成为必填」。本次核验发现这个默认值不是遗留便捷写法，而是**被测试写明的契约**。

`portable_oscbf/tests/test_jax_default_input_cache.py` 的模块 docstring 就是这条契约：「Default device inputs may be reused; live control/obstacle state may not.」它断言三件事：

- `obs_pos` 的默认值是**同一个对象**（第 15 行 `is` 比较）——即编译输入被缓存复用；
- `u_safe_prev` 的默认值**每次都从 `_last_u_safe` 现取**（第 16、25 行），不随缓存变陈旧；
- 实时障碍状态**不缓存**（第 23-24 行：先传过 active 障碍，再取默认，得到的是全零而不是上次的 active 值）。

也就是说，这个默认值承担的是「哪些输入可以缓存、哪些必须每步刷新」这条区分，`u_safe_prev` 属于「必须刷新」那一类。删掉它会让第 16、25 两行断言失效，等于**改动验收信号**——这与 Q6 里「只有测试读的字段」是同一类问题。

可以做的替代：保留默认值但把它显式化为一条独立的首输入契约（与 `obs_pos` 的复用契约并列写明），或者保留默认值不动、仅把 `_last_u_safe` 的更名/搬移做完。**不该做的是当成顺手清理删掉。**

与 `u_safe_prev` 形成对照的是 `_last_cbf_h` / `_last_cbf_grad`：这两个只有 facade 自己（和一份研究脚本）在读，没有任何生产消费者，才是真正可以随记录改造一起处置的部分。

### D.3 Q3：「内核直接返回记录」不成立（本次核验推翻了本原型自己的建议）

上面 C 部分只数字段数与改名点，得出「一个定义」可行；**那只算了翻译成本，没有算 jit 能不能收这个类型**。实测（jax 0.6.2，本机，与背景研究独立复现）后这条建议必须收回：

| 实测 | 结果 |
|---|---|
| `typing.NamedTuple` 进出 `jax.jit` | **直接可用**。`isinstance(nt, tuple)` 为真；2 个字段 = 2 个 leaf；jit 返回仍是 `NT` |
| 未注册的 frozen `@dataclass` 进 jit | **报错**。`tree_leaves(dc)` = `[dc]`（整个实例当成 **1 个 leaf**），`jax.jit` 抛 `TypeError: Error interpreting argument ... as an abstract array` |
| 加了 `register_dataclass` 的 frozen dataclass 进 jit | 收发**可以**，但**返回字符串不行** |
| 被 jit 的函数返回 `str` | **报错**：`returned a value of type <class 'str'> at output component ...` |
| jit 返回值里的 `None` | **能返回**，但 `tree_structure((1, None))` = `PyTreeDef((*, None))`，是**编译期固定的静态结构位**，不是数据 |
| 按步动态选 `None` | **报错**：`TracerBoolConversionError` |
| 同一个 jit 函数用 21 个不同的**值**调用 | 缓存条目仍是 **1** |

由此三条硬约束：

1. **现有 30 字段记录整体进不了 jit。** 它含 `constraint_metrics`（字符串 `quantity`/`unit`/`source`），而被 jit 的函数**不能返回 `str`**。注册 dataclass 也救不了这一条。
2. **要跨 jit 的类型只能是 `NamedTuple`**（或注册过的 dataclass），不能是普通 frozen dataclass。这一条推翻了 C 部分把两者当作等价选项的隐含前提。
3. **「未测」在内核侧不能靠 `None` 表达。** `None` 是静态空子树，位置在编译期就定了，不能按槽按步切换。内核要报告某槽未测，只能用**数字位**（一个 `valid`/`active` 0-d 数组），正像仓库里已有的 `qpax_warmstart.WarmStartState.valid: jax.Array`（`qpax_warmstart.py:31-44`）。

所以「一个具名定义」现实上只有两种落法，都能做，但**都不是**「内核直接返回现在的记录」：

- **(甲) 记录拆两半**：可进 jit 的**数组部分**用 `NamedTuple`（含显式 `valid` 位），字符串/单位/原因码等元数据留在 host 侧的伴生结构。这正是 Q4 需要的形态。
- **(乙) 内核继续返回位置元组**，把 host 侧记录定为唯一定义。翻译成本仍是 C 部分那 14 处改名。

另一条反向约束：**评价器的 `TrackingStepData` 不能一起改成 NamedTuple**。它依赖两个 dataclass 专属能力——`dataclasses.replace`（`tracking_evaluator.py:90`）与 `dataclasses.asdict`（`:390`、`:538`）。改成 NamedTuple 这两处会立刻 `TypeError`。所以「一个定义」如果要覆盖评价器那一侧，就必须先改写这两处调用点。

**结论修正**：Q3 的答案从「一个（内核直接返回记录），把握中高」改为「**一个定义在原理上可达，但必须先决定 (甲)/(乙) 的拆法，且跨 jit 的部分只能是 NamedTuple + 数字有效位**」。这是设计取舍，不是我已经能替你定的问题——但它有一个明确的事实底座：**把现在的记录直接交给 jit 是做不到的。**

---

## E. 对六个问题的结论

| 问题 | 结论 | 依据 | 把握 |
|---|---|---|---|
| Q1 三个无人生产的量 | **三个都不接**。如实记缺测；`online_verdict` 保持 `insufficient_evidence` 是设计意图。接上还会让 `tests/test_oscbf_controller_smoke.py:506-507` 这条**通过中**的断言（`{"unmeasured": 5}`）失败 | `tracking_evaluation.md:8`、OFF-02 规格 `:11/:150/:155`、研究 18 `§6/§10`、smoke test `:506-507` | 高（文档直接写明 + 活测试锁定） |
| Q2 节点的 29 键 dict | **可以删。** 0 处改名需要处理，25 个键是同名透传，另 4 个要保留为派生/宿主侧；**但 `qp_primal_residual` 必须先搬进记录**，否则节点失去侧信道上唯一被控制路径读到的值 | 本原型 C 部分 + `oscbf_controller.py:560` | 高（AST 计数） |
| Q3 一个具名定义还是两个 | **修正为：一个定义原理上可达，但不能「内核直接返回现有记录」。** 现有 30 字段记录含字符串，被 jit 的函数不能返回 `str`；跨 jit 的类型只能取 `NamedTuple` + 数字有效位，元数据留 host 侧。见 §D.3 | 本原型 §D.3 实测 + 背景研究 Q3 | 高（jit 实测三条硬约束） |
| Q4 缺测怎么落地 | 值位在**值槽**用 `None`；状态必须走具名字段（不能靠 `__status` 旁路键，会被白名单静默丢弃）；`NaN` 与 `0`/`1.0` 都不可用作「未测」。生产者侧只能用数字 `valid` 位 | 本原型 A 部分 + §D.3 的 jit 实测 | 高（真实评价器实测） |
| Q5 跨步记忆怎么进 jit | `_last_cbf_h`/`_last_cbf_grad` 只在 facade 内部（及一个研究脚本）被读，没有生产消费者；但 `u_safe_prev` 的默认值是**被测试写明的契约**，不能当清理顺手删。新增数组入参**不付逐步代价**（值不进缓存键），只付一次 trace | 本原型 B 部分 + `test_jax_default_input_cache.py` + §D.3 缓存实测 | 中高（两份直接证据） |
| Q6 死字段与归档模块 | **任何地方都没人读**的 3 个（`last_min_obs_dist`、`last_qp_ok`、`last_qp_warm_start_used`）可安全删；`last_delta_slack` 是**真缺陷**（未初始化 + 生产入口不写它），删需同时动一个研究脚本；**只有 `last_cbf_h_delta_norm`/`last_cbf_grad_delta_norm` 的读者是活测试**（`test_jax_tracking_step.py:100-101`），删它们＝改验收信号，应单独一票；`qpax_warmstart.py` **留着** | 本原型 B 部分 + 背景研究 Q6 | 中高（AST 普查 + skip 状态核实） |

---

## F. 本原型没有证明的事

- **没跑内核、没跑 ROS、没接硬件。** 第五问涉及的 jit 边界与 `u_safe_prev` 默认值是**读源码**得出的，不是实验结论；内核侧的数值等价性由第一轮原型的另一支实验覆盖。
- **没有测「具名状态字段」进 jit 的代价。** A 部分只证明了哪种写法在**消费者侧**的含义正确。生产者侧由 §D.3 补上（NamedTuple 可、未注册 dataclass 不可、`str` 不可、动态 `None` 不可），但**§D.3 只验证了类型合法性，没有测真实内核加一个入参/出参的编译与运行开销**。
- **AST 读者普查只看属性名。** 通过 `getattr(obj, "last_" + suffix)` 之类的动态访问不会被统计到；本次没有在仓库中发现这种写法，但这是方法的已知盲区。
- **`read_by_nothing_outside_facade` 不等于「可以删」**。`last_min_obs_dist`、`last_qp_ok` 仍在 facade 内部被写，删除要确认没有外部通过 `__dict__`/logging 之类隐式消费。
- **Q1 的政策项仍未决。** `eps` 的数值与单位、失败后的动作由 `docs/planning/oscbf-reuse/research/18-elastic-qp-admission.md §10` 列为待决议，本原型不回答它们。
- 未核验 `docs/specs/off02_obb_geometry_admission_spec.md` 所说 OFF-09 的当前实现进度；本文只引用规格里的职责边界，不主张 OFF-09 已经开工。
- **§D.3 的 jit 实测是在玩具类型上做的**（一个 2 字段 NamedTuple、一个 2 字段 dataclass），**没有在真实内核 `_path_tracking_fn` 上**验证「新增一个有效位/数组入参只新增一次编译」。缓存键只看结构与 aval 这一点已实测（21 个不同值 → 1 个条目），但真实 pytree 的规模效应未测。

---

## G. 与同一轮背景研究（`control-step-record-round2-20260916.md`）的对账

两份产物相互独立（一份是原型实验，一份是只读取证），我做了一次逐条对账。**没有互相矛盾的事实性结论**，但有 1 处**推翻**和 3 处**细化**：

### G.1 推翻：Q3 的「内核直接返回记录」

见 §D.3。这是本次对账里唯一改变建议方向的一条，也是本原型自己写错的一条。

### G.2 细化一：Q6 里「只有测试读」不等于「改动验收信号」

我原来的 B 表把 12 个属性都归为「只有测试读」并统一标为「改动验收信号」。背景研究核实了模块级 skip 状态后，这个说法**过粗**：

- `portable_oscbf/tests/test_jax_rate_limit.py:7` 与 `portable_oscbf/tests/test_jax_tracking_hard_stop.py:9` 都是**模块级** `pytestmark = pytest.mark.skip`（理由是依赖 `newaxis`，而该包在本机不可导入）。所以 `last_rate_slack`、`last_rate_constraint_violation`、`last_rate_solver_slack` 的读者**从不执行**，删它们不改变任何被执行的验收；
- `test_jax_tracking_hard_stop.py` 里的 `_JaxLoop`（`:55-70`）是给 **`newaxis`** 写的测试替身（`:22-24` `from newaxis.hardware_stop_execution import ...`），它自己定义同名属性，所以删本仓库 facade 的属性不会让它失效；
- **真正「删了就改验收」的只有两个**：`last_cbf_h_delta_norm`、`last_cbf_grad_delta_norm`（`portable_oscbf/tests/test_jax_tracking_step.py:100-101`，测试名 `test_tracking_fast_path_preserves_safe_command_without_h_gradient_telemetry`，本次核验中**通过**）。它断言的是「关掉遥测后两个 delta norm 必须是 NaN」，是「快路径不偷偷留下旧值」这条行为的唯一回归保护。

### G.3 细化二：Q1 的代价不只是「绕过未决政策」，还是一条活测试

背景研究找到 `tests/test_oscbf_controller_smoke.py:506-507`：

```python
    assert report.admission_counts == {"unmeasured": 5}
    assert report.overlap_counts == {"unmeasured": 5}
```

这条断言跑的是**真实节点 dict**（`:483-496` 五次 `node.step_once(_START_Q)`），通过中。接上任何生产者都会让它失败。同一文件 `:480` 还有 `assert step["min_obs_dist"] is None`，进一步把「未启用障碍 ⇒ 未测」这条口径锁在生产链的验收里。

### G.4 细化三：`last_delta_slack` 与 `last_qp_warm_start_used` 的性质不同

我在 B 表里把 `last_delta_slack` 记成「只有研究脚本读」，实际上它本身是**缺陷**：`__init__`（`:203-226`）不初始化它，写点只有 `:467`（`step()`）与 `:583`（`tracking_step()`），而**生产唯一入口 `path_tracking_step`（`:589-705`）不写它**。所以生产路径上它是「未定义或陈旧值」——同一个量在返回值里可靠、在侧信道上不可靠，这是「`last_*` 不是接口，是缓存」的最强证据。与之相对，`last_qp_warm_start_used` 虽然零读者，但它是**恒真的正确常量**（`qp_warm_start=True` 在 `:145-148` 被主动拒绝，所以它不可能为 `True`），不是报假账——删它的语气应当是「携带零信息」，不是「掩盖缺陷」。

### G.5 对账确认一致的部分

- 三个界面规模（42 内核槽 / 30 记录字段 / 29 节点键 / 12 别名）—— 双方各自独立计数，一致。
- 「3 个 dict 键没有同名记录字段」—— 我列的 2 个派生（`path_progress_m`、`min_obs_dist`）+ 2 个宿主侧（`projection_before_m`、`qp_primal_residual`）与背景研究列的 3 个（`projection_before_m`、`path_progress_m`、`qp_primal_residual`）是**同一集合的两种归类**（我把 `min_obs_dist` 单独算成「条件派生」），无冲突；两份都独立指出 `qp_primal_residual` 来自侧信道、必须先搬进记录。
- `min_obs_dist` 是「判据已正确、但写在错的那一层」（节点转 `None`、内核与 facade 直通 `1.0` 哨兵）—— 一致。
- `u_safe_prev` 的默认值是被测试写明的契约（`test_jax_default_input_cache.py`）—— 一致。
- 21 个 `last_*` 里只有 3 个有 facade 之外的生产读者，其中只有 `last_qp_primal_residual` 在控制路径上 —— 一致。
- 侧信道不是「有人用的公开接口」，主要是测试夹具 —— 一致。

