# 一步记录改造：逐项改动清单

日期：2026-09-16。
性质：**待用户确认的改动清单，尚未动任何代码。** 本文不是实施授权；用户确认后按批执行，每批交回验证结果。

来源：`docs/planning/oscbf-reuse/research/control-step-record-round2-20260916.md`（只读取证）、`control-step-record-round2-prototype-20260916.md`（原型与对账）、`control-step-record-round1` 两份（第一轮）。本文只把已确认的结论落成可执行的改动项，不新增结论。

## 0. 已确认的前提

用户在 2026-09-16 的两轮回答中确认：

1. **控制输出必须与现状逐位相同**，只准加信息、不准改数值，改动分小批走。
2. **「格式不对导致的测量丢失」改成响亮**：报告里明确记录，但结论仍是「证据不足」，fail-closed 不变。报告只做加法，不改现有键的含义。
3. **三个无人生产的字段本轮都不补**（`admission_ok`、`overlap`、`obstacle_clearance_m`），如实保持缺测；「允许花掉多少安全余量」（`eps`）挂成独立政策工单。「这份报告永远达不到 `pass`」是正常状态，不是退步。
4. **只删彻底没人读的字段**；涉及测试断言的删除单独一批、单独审。

以下由证据确定，用户已授权按此执行（可随时否决）：

- 合并成一个**具名、可被编译**的结构，放 `portable_oscbf/work/`（该目录对 ROS 与 `src/` 的导入命中数为 0；`src/` 早已 `from work.X import ...`，方向是顺的）。
- 跨 jit 的类型只能取 `NamedTuple`：未注册的 frozen dataclass 整个实例是一个 leaf，进 jit 直接 `TypeError`；`None` 在 jit 返回值里是编译期固定的静态结构位，不能按步表达未测，只能用数字标记位。
- 评价器内部那个记录（`TrackingStepData`）**不并进**单一定义：它依赖 `dataclasses.replace`（`tracking_evaluator.py:90`）与 `asdict`（`:390`、`:538`），改成 `NamedTuple` 会当场 `TypeError`，收益为零。
- 评价器的 12 行别名表**先不动**：它继续服务历史验证脚本与「扔字典进去」的老调用方，动它会牵连历史证据的复现脚本。
- 有效位只加在「本来就可能不适用」的槽上，不是 42 个全加。

## 1. 批 1：定义收成一个具名结构（内核侧）

| 位置 | 现在 | 改成 | 为什么 |
|---|---|---|---|
| `portable_oscbf/work/jax_kernel_factory.py:751-772` | `return (...)` 42 项全靠位置认 | 返回具名结构 | 42 项的位置顺序就是最贵的隐性契约，加一项要改三处 |
| 同上（新增） | — | `NamedTuple` 定义，字段名采用现在记录里的名字 | `NamedTuple` 是已注册 pytree，进出 jit 零成本、位置解包照旧可用 |
| `jax_kernel_factory.py:225-227` | 障碍未启用时 `min_obs_dist` 填哨兵 `1.0` | 数值保留，另配「是否启用」的数字位 | 内核不能用 `None` 报未测（静态结构位）；哨兵数不能当量测 |
| 同上（ESDF） | `sdf_enabled` 只在入参 | 同样配数字位 | `tracking_evaluation.md:38` 要求未启用时保留未测计数 |
| `jax_control_facade.py:629-641` | 按位置解包 42 项 | 具名访问 | 消掉这一层的错位风险 |
| `jax_control_facade.py:673-705` | 记录构造里有 13 处改名的直通（`min_dist`→`min_obs_dist`、`next_path_state`→`path_state`、`reference_source_time`→`reference_source_time_s`、`cross_track_error`→`cross_track_error_m`、7 个 `feedrate_*`、`actual_tangent_speed`…）+ 1 处加工（`constraint_residuals`→`constraint_metrics`） | 13 处改名消失；1 处加工保留（约束指标是 host 侧用字面量拼的，见 `:731-745`） | 同一个数三个名字是四层形状问题的根源 |
| `jax_control_facade.py:681` | `min_obs_dist=float(min_dist)` 直通哨兵 | 携带状态位 | 消费者不必再回头查 `obs_enabled` |
| 记录（新增字段） | `qp_primal_residual` 只在侧信道（`:763`） | 进记录 | 内核返回值里本来就有 `primal_residual`；它是侧信道上唯一被控制路径读到的量（`oscbf_controller.py:560`） |

**验收**：`tests/test_oscbf_controller_smoke.py` 的黄金值测试（`:240-306`：固定 q、非默认参数、4 步、对 `err_6d`/`feedrate_m_s`/`path_progress_m`/`qp_primal_residual`/`qp_ok` 有冻结期望数组）**期望数组一字不改**且通过。
**代价**：内核返回值换了 pytree 结构，会触发一次重新编译（一次 trace）；逐步开销不变（缓存键只看结构与 aval，实测 21 个不同值仍只有一个缓存条目）。

## 2. 批 2：记录成为唯一通道（节点侧）

| 位置 | 现在 | 改成 | 为什么 |
|---|---|---|---|
| `src/robot_safecontrol_moveit/oscbf_controller.py:543-583` | 手写 29 键 dict | 返回记录 | 记录→节点这一层 0 处改名，25 个键是同名透传，删除不需要任何翻译 |
| `:555-565`（`min_obs_dist`） | 节点回头查 `obs_kwargs["obs_enabled"]`，否则填 `None` | 改读记录的状态位 | 判据已经正确、但写在消费方；每个新消费者都要重算一遍 |
| `:559-561`（`qp_primal_residual`） | 读 `self._loop.last_qp_primal_residual`（侧信道） | 改读记录字段 | 侧信道不是接口；这是它唯一被控制路径读到的值 |
| `:557`（`path_progress_m`） | `float(result.path_state[0])` | 改读记录（`path_state[0]`） | 纯粹是同一份数据的第二个名字 |
| `:552` + `:537-540`（`projection_before_m`） | 塞进 dict | 节点自己算，调用评价器时单独传参 | 只有节点知道评价几何，这个量本来就不属于「一步的输出」 |
| `:647` | `step["step_latency_ms"] = duration_ms` | `update(..., step_latency_ms=duration_ms)` | 耗时是**调用方**的测量边界（含内核输出到主机的转换），不属于 `step_once` 的产物；记录不可变，也不该硬塞 |
| `:644-689` | 18 处 `step["..."]` | 记录属性访问 | 同上 |
| `tracking_evaluator.py:87` | `step_from_result` 靠 `getattr` 兜住对象 | 加一条明确的「收记录」入口 | 现在能兜住是巧合（`isinstance(result, Mapping)` 的 else 分支），不是契约 |
| `tracking_evaluator.py:216-218`（`update`） | — | 增加 `step_latency_ms`、`projection_before_m` 两个参数 | 承接上面两项 |
| `tracking_evaluator.py:37-38` + `:402-403` | 门槽收到非布尔时**静默**计入 `unmeasured` | 记成无效并留一条说明（结论仍是证据不足） | 整个研究里唯一「出了问题查不出来」的故障模式：用 `0` 表示「无重叠」会静默丢一次真实测量 |

**验收**：黄金值测试通过；`:480`（`step["min_obs_dist"] is None`）与 `:506-507`（`admission_counts == {"unmeasured": 5}`、`overlap_counts` 同）保持通过——这三条正是「缺测语义没变」的证明。

### 必须改动的测试（逐条）

| 位置 | 现在 | 改成 | 理由 |
|---|---|---|---|
| `tests/test_oscbf_controller_smoke.py:276-306` | 按下标读 `q_next`/`err_6d`/`feedrate_m_s`/`path_progress_m`/`qp_primal_residual`/`qp_ok` | 属性访问 | 形状从 dict 变记录 |
| `:407` | `step["reference_at_endpoint"] = True`（改写返回值伪造终止） | 构造一份替换了该字段的副本 | 记录不可变 |
| `:471-480` | 按下标读 5 个键 | 属性访问 | 同上；`:480` 的 `is None` **语义必须保持不变** |
| `:491-492` | 整个 dict 交给 `evaluator.update` | 改传记录 | 同上 |
| `:692-694` | 按下标读 `q_next` | 属性访问 | 同上 |

**这批是「改验收信号」吗？** 不是。这 5 处改的是**读法**，断言的值和含义都不变。真正改验收信号的只有批 3 的一处（见下）。

## 3. 批 3：跨步状态显式化

| 位置 | 现在 | 改成 | 为什么 |
|---|---|---|---|
| `jax_control_facade.py:224`（`_last_u_safe`：`:465`/`:581`/`:647` 写，`:814`/`:869` 读作 `u_safe_prev`） | 挂在对象上的可变属性 | 收进一个具名状态结构，显式进出 | 它是唯一真正参与控制计算的跨步记忆（生产 `temporal_lambda=0.2`，生效位置 `oscbf_velocity_config.py:216-218`、`:229-231`） |
| `:222-223`（`_last_cbf_h`/`_last_cbf_grad`） | 同上 | 收进同一状态结构 | 它们是两个 delta 范数的数据来源，必须成组处置 |
| `:775-780`（两个 delta 范数） | 写在侧信道 | 移进记录 | 现在只有一条测试读它们 |
| `:748-782`（`_update_qp_diagnostics`） | 一次性写 21 个属性 | 拆分：写记录的归记录，写侧信道的归侧信道 | 「哪个入口写哪些属性」不一致已经产生了真缺陷（`last_delta_slack`） |
| `:814`、`:869`（`u_safe_prev` 的默认值） | 取当前 `self._last_u_safe` | **保留不动** | `portable_oscbf/tests/test_jax_default_input_cache.py` 的模块 docstring 就是这条契约：「可缓存的是编译输入，实时控制量每步刷新」 |
| 新增 | — | 跨步状态具名结构（`NamedTuple`，带有效位） | 形式照抄 `portable_oscbf/work/qpax_warmstart.py:31-44` 的 `WarmStartState.valid: jax.Array` |

### 唯一一处真正改验收信号的改动

`portable_oscbf/tests/test_jax_tracking_step.py:100-101`：断言「关掉 `collect_cbf_diagnostics` 后两个 delta 范数必须是 `NaN`」。这条测试**现在通过**，且是「快路径不偷偷留下旧值」这条行为的唯一回归保护。

改动方式：**换位置，不放弃**——断言移到记录的对应槽位上，仍断言 `NaN`。语义等价，但因为断言的载体变了，属于「改验收信号」，需要在提交说明里明确标出，不能夹在别处悄悄改。行号：`test_jax_tracking_step.py:100-101`。

**验收**：`portable_oscbf/tests/test_jax_default_input_cache.py` **一字不改**通过——这是「缓存与刷新契约没被破坏」的直接证明。

## 4. 批 4：死字段清理

### 4a 零读者（无风险）

| 删除对象 | 位置 | 依据 |
|---|---|---|
| `last_qp_ok` | `jax_control_facade.py:203`、`:753` | 全仓零读者 |
| `last_min_obs_dist` | `:204`、`:754` | 全仓零读者。**注意**：孪生的 `last_min_esdf_dist`（`:205`、`:755`）**保留**，它有生产读者（`perception_demo.py:307`），删完两者形态不对称，需在提交说明写明 |
| `last_qp_warm_start_used` | `:215`、`:762` | 零读者，且是**恒真的正确常量**（`qp_warm_start=True` 在 `:145-148` 被主动拒绝，它不可能为 `True`）——删它的理由是「携带零信息」，不是「掩盖缺陷」 |
| `last_delta_slack` | `:467`、`:583`；`__init__` 本来就漏了它 | **真缺陷**：生产唯一入口 `path_tracking_step`（`:589-705`）不写它，所以生产下读到的是陈旧值或 `AttributeError`。需同步改研究脚本 `docs/planning/oscbf-reuse/research/5-redundancy-evidence/compare.py:55` |

### 4b 涉及测试（单独一批、单独审）

| 删除对象 | 位置 | 性质 |
|---|---|---|
| `last_rate_slack` | `:206-211`、`:758` | 是 `last_rate_constraint_violation` 的**纯别名**（逐字赋值）。注释（`:206-209`）自述是「公共兼容别名」，删它＝**收回一条兼容承诺**，应按契约收回记录，不当清理 |
| `last_rate_constraint_violation` | `:210`、`:756-757` | 同上 |
| `last_rate_solver_slack` | `:212`、`:759` | 是**另一个量**（qpax 原始内点松弛），不是别名 |

读者全部位于**模块级 skip** 的模块：`portable_oscbf/tests/test_jax_rate_limit.py:7`、`portable_oscbf/tests/test_jax_tracking_hard_stop.py:9`（依赖的 `newaxis` 在本机不可导入）。
代价：**不改变任何在跑的验收**，但必须同步改这两个 skip 模块的源码，否则留下悬挂引用。

## 5. 独立小票（不混进本轮）

| 对象 | 处置 | 依据 |
|---|---|---|
| `portable_oscbf/work/qpax_solver.py` | **单开小票** | 全仓零导入者、零测试（第一轮文档曾误把它当作 `u_safe_prev` 的生产作用点，已在文档中更正） |
| `portable_oscbf/work/qpax_warmstart.py` | **保留** | 有 1 个通过的独立测试；SHA-256 被 7 份已归档证据清单记录（删了会让既有证据身份与工作树不一致）；是本轮「跨步状态显式化」唯一已落地的现成先例 |

## 6. 纯加法项（不影响任何现有结论）

- 记录新增 `qp_primal_residual` 字段。
- 记录新增「是否量测」的状态位（障碍距离、ESDF 距离）。
- 评价器报告新增「格式不对导致的无效」计数。
- 评价器新增「直接收记录」入口（老的字典型入口原样保留）。
- 以上全部只增不改：现有键的含义与历史报告结构都不变，历史报告不重算。

## 7. 验证方法

| # | 手段 | 证明什么 |
|---|---|---|
| 1 | 黄金值测试 `tests/test_oscbf_controller_smoke.py:240-306`（固定 q、非默认参数、4 步，对 5 个量有冻结期望数组）**期望数组一字不改**且通过 | 控制输出逐位未变。这是本次最强的证据：数字对得上就是对得上，没有解释空间 |
| 2 | `portable_oscbf/tests/test_jax_default_input_cache.py` 一字不改通过 | 缓存/刷新契约未破 |
| 3 | `tests/test_oscbf_controller_smoke.py:480` + `:506-507` 通过 | 缺测语义未变（哨兵没被当量测；准入/重叠计数仍是 `unmeasured`） |
| 4 | 定向跑受影响的测试模块 | 每批的即时回归 |
| 5 | 最后跑一次 `bash run_all_tests.sh` | 全量兜底 |

**不启动仿真、不碰 C++ 插件（无需重建 AEB）、不碰真实硬件。** 这轮是纯 Python 结构整理，「live 保持 fail closed」的状态不变。

## 8. 风险与遗留

- **一次重新编译**：批 1 改了内核返回值的 pytree 结构，会触发一次 trace。逐步开销不变。
- **两个「只有调用方知道」的量**（`projection_before_m`、`step_latency_ms`）改由节点在调用评价器时单独传参。这是本次唯一改变「谁提供这个量」的地方。
- **有效位的完整清单要在批 1 实现时逐槽过一遍**：判据是「这个槽有没有『本来就可能不适用』的情况」。初步筛出障碍距离、ESDF 距离两项，其余（位置、误差、耗时）永远有值，不加。
- **门槽改响亮后报告会多一栏计数**：只做加法。历史报告不会重算，读取者不会因为多一栏而失败。
- **未决政策项**（`eps` 数值与单位、失败后的动作、`obstacle_clearance_m` 的归属票）不在本轮内，挂独立工单。
- **未核验**：本文所有行号来自当前工作树（含 11 项未提交改动）。若那些改动先被提交或回退，行号需要重新核对。
