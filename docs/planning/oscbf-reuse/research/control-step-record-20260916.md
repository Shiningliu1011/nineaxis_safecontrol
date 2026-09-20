# 单个控制步的记录接口：五问研究

日期：2026-09-16。研究问题：把“一步控制”的数据跨进程内接缝传递时，输出记录该取什么形态、缺测该怎么表达、per-step 测量与跨步记忆该怎么分、改动范围该到哪里、测试该怎么跟随。**本文只读源码与文档，未运行 ROS、未改任何源码或测试，未做真机动作。**

证据标记沿用本目录既有写法：**【代码】= 本仓库一手源码（读时工作树，行号为读取时行号）**；**【文档】= 本仓库一手文档**；**【JAX 源码】/【PEP】/【JAX 文档】= 外部一手来源**；**【实验】= 本机只读 Python 片段实测**；**【推断】= 本文推理，无一手来源直接支持**。

## 0. 事实核验：四形态、两通道

主代理给出的形态描述基本成立，但两处规模与“约 24 个 `last_*`”有出入，以下为实测值。

| 层 | 形态 | 位置 | 实测规模 |
|---|---|---|---|
| 内核 | 位置元组（`return (...)`） | `portable_oscbf/work/jax_kernel_factory.py:751-778` | **42 个元素**（逐项数解包得到，见下行） |
| facade | 按位置解包 → `@dataclass(frozen=True) JaxPathTrackingResult` | 类定义 `jax_control_facade.py:55-90`；解包 `jax_control_facade.py:625-668` | **30 个 `AnnAssign` 字段** |
| ROS 控制器 | 手写重建的 `dict` | `src/robot_safecontrol_moveit/oscbf_controller.py:538-583` | **29 个键** |
| 评价器 | `TrackingStepData`（frozen dataclass） | `src/robot_safecontrol_moveit/tracking_evaluator.py:64-78` | 9 个显式字段 + `values: dict` |

【代码】42 / 30 / 29 三个数字由 AST 与解包文本直接数出（`step_once` 的 `Return` 节点 `dict.keys` 长度 = 29；`JaxPathTrackingResult` 的注解赋值数 = 30）。**主代理所述“31 字段”与实测不符，实测 30**；请以实测为准。

第二通道（副作用）实测 **21 个**，不是“约 24 个”：`jax_control_facade.py:203-226` 声明 20 个（18 个 `last_*` + 3 个 `_last_*`，其中 `_last_cbf_h`/`_last_cbf_grad`/`_last_u_safe` 为下划线私有），另有 1 个 `last_delta_slack` 只在 `:467`、`:583` 被赋值而**从未在 `__init__` 初始化**。真正被生产代码读取的只有 1 个：`oscbf_controller.py:560` 读 `self._loop.last_qp_primal_residual`。

评价器侧“三个字段生产从不提供”核验成立：`admission_ok`、`overlap`、`obstacle_clearance_m` 在 `oscbf_controller.py`、`jax_control_facade.py`、`jax_kernel_factory.py` 中**零命中**（grep 全无）。`tracking_evaluator.py:104` 的别名表把 `obstacle_clearance_m` 映射到它自己，因此 `get()` 恒为 `None`；`tracking_evaluator.py:402-403` 把 `admission_ok`/`overlap` 的 `None` 计数为 `unmeasured`；`:502` 因此把 `online` 钉在 `insufficient_evidence`。别名表本身为 **12 项**（`tracking_evaluator.py:97-109`），与主代理所述一致。

---

## Q1. 改动范围：只动生产路径的输出侧，还是也动宽位置输入与测试专用旧内核？

### 一手证据

**(a) 宽输入签名在 facade 处已经是仅关键字**。【代码】`jax_control_facade.py:524` 的 `def tracking_step(self, *, q, task_pos, ...)` 与 `:589` 的 `def path_tracking_step(self, *, q, path_state, ...)` 都在 `*` 之后，全部参数只能按关键字传入。因此“24 参数 / 21 参数”在 Python 层面**不是位置接口**，增删或改名参数不会让任何调用点静默错位。真正按位置传递的只有内核闭包内部那一处：`jax_control_facade.py:620-624` 以 21 个位置实参调用 `self._path_tracking_fn`（即 `jax_kernel_factory.py:704` 的 `path_tracking_step`），生产者在同一次改动里即可同步。

**(b) 各范围的实际测试影响面（AST 逐测试函数统计，仅计 `test_*` 函数）**：

| 范围 | 受影响测试 | 明细 |
|---|---|---|
| A. 只改生产输出侧 | **约 61 个** | 调 facade `path_tracking_step` 的 10 个（`test_elastic_qp` 2、`test_jax_esdf_cbf` 1、`test_jax_path_posture_reference` 1、`test_jax_tool_axis_tracking` 1、`test_jax_tracking_hard_stop` 1、`test_manipulability_nullspace` 1、`test_modular_jit_equivalence` 1、`test_perception_interface` 2）；`tests/test_oscbf_controller_smoke.py` 读 step-dict 字符串键的 5 个；`tests/test_tracking_evaluator.py` 全部 **42 个**（都经 `_step()` 造 dict 后 `evaluator.update(...)`，`tests/test_tracking_evaluator.py:27-37`）；`tests/test_tracking_report_writer.py` 4 个 |
| B. 另加宽输入签名 | **净增约 0~7 个** | 因输入是仅关键字（见上），只做**增字段/加默认值**时净增 0；只有**删除或改名**才波及这 7 个：调 facade `tracking_step` 的 5 个（`test_jax_rate_limit` 1、`test_jax_tool_axis_tracking` 1、`test_jax_tracking_step` 2、`test_manipulability_nullspace` 1）+ 调 facade `step` 的 2 个（`test_jax_esdf_cbf` 1、`test_jax_frozen_qp_problem` 1） |
| C. 另加测试专用旧内核 | **即上面的 7 个 + 1 个内核级等价测试** | 上述 7 个就是“旧内核”的全部消费者；另有 `test_jax_tracking_step.py:94-96` 在同一测试内比较 `tracking` 与 `tracking_fast` 两条内核路径 |

全仓可收集测试 **568 个，另有 16 个模块导入失败（ROS 依赖）**（`python3 -m pytest --collect-only -q portable_oscbf/tests tests` 实测）。所以范围 A 已覆盖约 11% 的测试，且**集中在 4 个文件**，不是散在全仓。

**(c) 旧内核是否“只有测试在用”**。【代码】生产路径唯一的入口是 `JaxControlLoop.path_tracking_step`；`_step_fn`（`jax_control_facade.py:364`）、`_tracking_fn`（`:365`）、`_tracking_fast_fn`（`:366`）在 `portable_oscbf/work/` 内**没有任何生产调用者**，`src/robot_safecontrol_moveit/` 只出现 `perception_demo.py` 的 `path_tracking_step` 与 `last_path_metrics` 读取（`:307`、`:387`），没有 `.step(` / `.tracking_step(`。结论：`step`、`tracking_step`、`tracking`、`tracking_fast` 这四条确实是**测试与基准专用**。

### 外部一手来源：关于分段推进

- 【PEP】[PEP 387 — Backwards Compatibility Policy](https://peps.python.org/pep-0387/)「Making Incompatible Changes」节原文：**“Making an incompatible change is a gradual process performed over several releases: Discuss the change. …”**，并把“给定参数下的返回值、副作用与异常”和“参数与返回值的**位置**与期望类型”列为受保护项（「Backwards Compatibility Rules」节）。
- 【PEP】同一节明确列出**不受**该策略约束的项，其中包含 **“Test suites. (Anything in the Lib/test directory or test subdirectories of packages.)”** 与 **“Function, class, module, attribute, method, and C-API names and types that are prefixed by ‘_’”**。
- 【JAX 文档】[Common Gotchas in JAX](https://docs.jax.dev/en/latest/notebooks/Common_Gotchas_in_JAX.html)：**“JAX transformation and compilation are designed to work only on Python functions that are functionally pure”**，且“all the input data is passed through the function parameters, all the results are output through the function results”。

**必须说明的界限**：PEP 387 是 **Python 标准库**的兼容策略，它不直接管辖本仓库的内部接口；我把它当作“分段推进 + 测试套件不属于公共契约”这两条原则的**一手出处**，把它当作对本仓库的**直接约束**则是【推断】。本文限定来源集合中没有 PEP 或官方文档直接规定“内部接缝重构该分几段”。

### 推荐答案（Q1）

**范围取 A ∪ B，暂时不动 C。** 理由：

1. 输出侧是四形态的真正来源，且范围 A 影响的 61 个测试集中在 4 个文件，一次改完可归因；这是 PEP 387 所说“单次改动、可审计”的最小充分单元。
2. 输入侧顺带改成“命名结构体入参”收益高、风险低：facade 已是仅关键字，且【JAX 文档】要求输入/输出都走参数与返回值。**但不要在同一段里删参数**，否则会净波及 7 个旧内核测试，把两件事的失败混在一起。
3. C（旧内核）应作为**独立段**处理，因为按 PEP 387，测试套件不是公共 API，退役它们的成本是“改测试+记录”，不是“保兼容”；把它和前两段捆在一起只会让 A 段的验收信号变脏。**本项目自己的迁移模板已经要求每段写清“验收门/回退/退役”**（【文档】`docs/planning/oscbf-reuse/issues/07-migration-order.md`「每一迁移段的交接模板」节），本建议与它一致。

置信标记：**有依据的推断**（范围划分与计数是【代码】实测；“分三段”本身没有一手来源直接规定，PEP 387 只给出原则）。

---

## Q2. 命名结构体能否从 `jax.jit` 函数返回？与返回普通元组等价吗？frozen dataclass 要满足什么？

### 本机实测（JAX 0.6.2 / jaxlib 0.6.2，`/home/lsn/.local/lib/python3.10/site-packages/jax`）

1. **NamedTuple 可以从 jit 函数返回，且类型被保留。** 【实验】`@jax.jit def f(x): return N(x+1., x*2.)` 的返回值 `type(...).__name__ == 'N'`，字段可按键访问；`f(x) == (x+1., x*2.)` 数值逐位一致（`jax.tree_util.tree_flatten` 的 **leaves 完全相同**）。
2. **返回结构体不改变 jit 缓存条目数。** 【实验】同一函数连续以不同输入值调用，`_cache_size()` 保持 1；输出形态不进入缓存键。
3. **编译产物只差名字，不差计算。** 【实验】对 `f_tuple` 与 `f_named` 取 `.lower(x).as_text()` 后做 unified diff，差异只有两处：模块名 `@jit_f_tuple` / `@jit_f_named`，以及 `jax.result_info = "result[0]"` / `"result.a"`。`make_jaxpr` 的 diff 同样只有 `name=f_tuple` / `name=f_named` 一行。计算图其余部分逐字节相同。
4. **字段类型不限数组。** 【实验】NamedTuple 里放 `float` / `bool` / 嵌套 `dict` 都能返回：标量以 **0-d Array** 形式回来（`Array(3., dtype=float32, weak_type=True)`、`Array(True, dtype=bool)`），`dict` 原样保留并作为 pytree 参与 flatten（`PyTreeDef(CustomNode(namedtuple[R], [*, *, *, {'k': {'v': *}}]))`）。也就是说 `float(r.f)`、`bool(r.b)` 仍可直接用。
5. **frozen dataclass 作为 pytree 需要显式注册。** 【实验】`jax.tree_util.register_dataclass(D, data_fields=['a','b'], meta_fields=[])` 之后，`@jax.jit def f(x): return D(x+1., x*2.)` 正常返回 `D` 实例。**未注册的普通 dataclass 不行**（这正是 register 存在的理由）。
6. **“有时是 None、有时是数组”的字段会改变输出 treedef ⇒ 触发重编译。** 【实验】以 `static_argnames=['flag']` 让某个字段在 `x` 与 `None` 之间切换，`_cache_size()` 从 1 变 **2**（两次编译）。若切换条件依赖**被追踪的值**，则直接失败：`TracerBoolConversionError: Attempted boolean conversion of traced array`。配套事实：`jax.tree_util.tree_flatten((a, None))` 的 treedef 是 `(*, None)`，与 `(*, *)` 不同。
7. **register_dataclass 的版本要求。** 【JAX 源码】本机 0.6.2 的 `_src/tree_util.py:914` 定义 `register_dataclass(nodetype, data_fields=None, meta_fields=None, drop_fields=())`；docstring `:954` 写 **“In JAX v0.4.35 or older, you must specify ``data_fields`` and ``meta_fields``”**，`:974` 写 **“Starting in JAX v0.4.36, the ``data_fields`` and ``meta_fields`` arguments are optional”**（可选的前提是 `nodetype` 是 dataclass，未标 static 的字段默认算 `data_fields`）。`meta_fields` 的硬约束：**“Metadata fields *must* be static, hashable, immutable objects, as these objects are used to generate JIT cache keys. In particular, metadata fields cannot contain jax.Array or numpy.ndarray objects.”**；`:1019`/`:1022` 对“只给一个 / 非 dataclass 缺参”抛 `TypeError`。
   关于**首次引入**版本：【JAX 源码】我逐 tag 核对 `jax/_src/tree_util.py`，**`jax-v0.4.26` 无该函数，`jax-v0.4.27` 已有**（截取各 tag 原始文件检索 `def register_dataclass(`）。CHANGELOG 里能找到的只有变更记录：0.4.32 增加“必须覆盖且仅覆盖 `init=True` 字段”的校验，0.4.36 允许内联声明 static（【JAX 文档】[CHANGELOG](https://raw.githubusercontent.com/jax-ml/jax/main/CHANGELOG.md)）。**CHANGELOG 没有“首次引入”条目，上述引入区间是本机核验结果，不是发布说明的表述。**

### 推荐答案（Q2）

**能，且与普通元组在计算与缓存上等价；生产记录建议用 NamedTuple 作为返回值形态，不用需要注册的 frozen dataclass。** 具体：

- 返回形态选 **NamedTuple**：零注册成本、jit 无障碍、字段名自带自证、可在解包处直接 `result.q_next`，并且【代码】本仓库已有先例——`portable_oscbf/work/qpax_warmstart.py:31-44` 的 `WarmStartState` 就是 `NamedTuple`，且它是**跨步记忆**并被当作显式输入/输出在 JIT 内部流动。
- 不选 frozen dataclass 作为**内核返回值**：它需要 `register_dataclass`，而 `meta_fields` 不能含数组、`data_fields` 不能含非 pytree 的普通标量语义（标量会变 0-d Array），且 `JaxPathTrackingResult` 现有字段里混有 `dict | None`，None 与 dict 的 treedef 不同（Q3 会展开）。
- 若坚持用 frozen dataclass，必须在模块导入期显式 `register_dataclass` 并**把全部字段列全**（0.4.32 起缺项/多项会报错）。本项目已装 0.6.2，`data_fields`/`meta_fields` 可省略，但**不要省略**——省略后默认“凡未标 static 的都是 data field”，一旦将来加入字符串/枚举字段会静默进入 data field 并在运行期报错。
- **不要用“有时 None 有时是数组”的字段**（第 6 条实测：多一次编译，且被追踪的切换条件直接报错）。缺测要么用**固定形状的显式哨兵+独立 status 字段**，要么用**结构恒定的 tri-state**，见 Q3。

置信标记：**已验证**（第 1~7 条均为本机实验或本机/远端一手源码直读）。

---

## Q3. “生产方没有测这个”在接口上该怎么表示？

### (a) 本仓库自己的一手约定

本仓库对这个问题**已经给出了两套一致的答案，且都不是“裸 Optional”**：

**约定一：缺测用“显式 null”，并且“缺席”与“显式 null”等价。**

- 【代码】`src/robot_safecontrol_moveit/calibration_record.py:199-209`（`_canonical_object`）对白名单字段一律 `entry.get(name)`，注释原文：**“缺席与显式 null 等价：都规范化为 null，避免写不写该字段就改变身份。”** 即：**身份计算不区分“键不在”与“键在但值为 null”**。
- 【文档】`docs/adr/0009-calibration-record-schema-identity-admission.md:17`：**“未实测字段写 `null`，不编造序列号/残差/日期”**；`:60-61` 给出“`calibrated: false` 时 `timestamp`/`sensor_serial` 可 null，`calibrated: true` 时必填、非空”的**按状态分列**表，并规定 `residual`/`error_estimate` “至少一项非 null”。
- 【文档】`docs/ONBOARDING.md:67` 补充约束：**“记录规范化中的映射键必须为字符串（禁止把数字键转为字符串后覆盖同名键）”**——即 **dict 键的缺失/合并本身已被本仓库当成过事故来源**（详见 `docs/planning/oscbf-reuse/handoffs/45-calibration-record.md:19` 的“数字键 `1` 与字符串键 `"1"` 被归并”）。
- 【代码】`calibration_record.py:417` 对“必须是 null 或某个东西”的字段直接用错误码 `CALIB_PROVENANCE_MISSING` 拒绝；`:766` 要求诊断“必须显式标「未评估」”。

**约定二：跨层判定用显式 tri-state 枚举，且“不确定”必须 fail closed。**

- 【代码】`portable_oscbf/work/obb_geometry_admission.py:65-80` 定义三个 `str, Enum`：`PointCollisionStatus{SEPARATED, OVERLAP, INDETERMINATE}`、`CertificateStatus{..., INDETERMINATE}`、`DomainCoverageStatus{..., INDETERMINATE}`；`:327-330` 的返回分支明确三分，`:438-449` 汇总时“只要有一对 INDETERMINATE 就不能是 SEPARATED”。
- 【文档】`docs/specs/off02_obb_geometry_admission_spec.md:55-57`、`:68`：`indeterminate` 定义为“输入、数值、身份或配对完备性不足，**不能解释为分离**”，并规定“非有限值、维数错误、关节越限、空身份、未知边界、模型身份失配或 SAT 数值边界**必须 fail closed**”。`:105`、`:132` 是同一结构在证书与覆盖两层上的复用。`docs/planning/oscbf-reuse/research/off02-off09-interface-20260915.md` 的开篇结论也明确：**“拒绝动作可以相同，原因不能被压成同一个布尔值。”**
- 【代码】评价器这一层已经是同一个形状：`tracking_evaluator.py:402-403` 把 `None` 分到 **`unmeasured`** 桶（`admission` → `accepted`/`rejected`/`unmeasured`；`overlap` → `overlap`/`clear`/`unmeasured`），`:502` 在 `unmeasured` 时给 `insufficient_evidence` 而不是 `pass`。
- 【文档】`docs/tracking_evaluation.md:31` 对 `obstacle_clearance_m` 的规定最直接：**“没有量测时保持缺失，不能从裕度或 inactive sentinel 反推”**；`:34` 补充“缺失、NaN、Infinity 和空样本不会补零或补成 QP 成功”；`:8` 明确“缺少准入或重叠记录时为证据不足，**不能从 `qp_ok` 推断**”。
- 【文档】`docs/adr/0008-tracking-evaluation-scope-and-angle.md`（结尾段）：**“缺失数据和未接受阈值给出证据不足”**、**“没有采集到的边界不能填写推测值”**（同义表述见 `docs/planning/oscbf-reuse/handoffs/44-offline-tracking-evaluation.md`「按测量边界组织报告与完成结论」节）。

把上面拼起来，本仓库现行约定是**三元语义**：**已测（值）/ 已测且判定为否（False、`rejected`、`clear`、`outside`）/ 未测或不可判定（缺失、`unmeasured`、`indeterminate`、`insufficient_evidence`）**。注意第二元与第三元在**判定结果**上都可能导致拒绝，但本仓库要求它们在记录里**可区分**（`off02-off09-interface-20260915.md`：「`outside` 与 `indeterminate` 对准入都是否决，但前者通常需要换路径或缩域，后者通常需要补证据、修身份或修输入；保留区别能防止错误恢复。」）。

### (b) 外部一手来源

- 【PEP】[PEP 484](https://peps.python.org/pep-0484/)「The typing Module」节：**“Optional, defined by `Optional[t] == Union[t, None]`”**，并在 union 段补充 **“As a shorthand for `Union[T1, None]` you can write `Optional[T1]`”**；关于默认值：**“By default, `None` is an invalid value for any type, unless a default value of `None` has been provided in the function definition.”**
  → **`Optional[T]` 在类型系统里只是 `Union[T, None]`，是二值的一支，不是三态。** 类型标注无法表达“这个 `None` 是缺测还是判定为无”。这一点是我从 PEP 原文直接读出的事实，不是对它的引申。
- 【PEP】[PEP 557 的 Frozen instances 节](https://peps.python.org/pep-0557/#frozen-instances)：**“by passing `frozen=True` to the `@dataclass` decorator you can emulate immutability”**，`__setattr__`/`__delattr__` **“will raise a `FrozenInstanceError` when invoked”**，且**“If either `__getattr__` or `__setattr__` is defined in the class, then `ValueError` is raised.”**
  → frozen 只保证**赋值失败**，不提供任何“值是否被测量过”的语义；若要在 `__post_init__` 里做归一化，`frozen` 类必须用 `object.__setattr__`（本仓库已在用这一手法：【代码】`portable_oscbf/work/safety_snapshot.py:33-41` 的 `SafetyGridSpec.__post_init__`）。
- 【JAX 源码】`meta_fields` 不能含数组、其值参与 JIT 缓存键（见 Q2 第 7 条）：这意味**“缺测”这件事不能在 pytree 层用“字段消失”来表达**——字段消失就是 treedef 变化。

**没有一手来源覆盖的部分（明确说明）**：本文限定的来源集合（JAX 官方文档/源码/发布说明、Python typing 规范、PEP）里，**没有任何一份文献专门回答“接口层该用 Optional None、缺键、还是 tri-state 来表达未测量”**。PEP 484 只定义了 `Optional` 的集合语义，PEP 557 只定义 frozen 的行为，JAX 文档只谈 pytree/缓存。上述“用 tri-state”的结论，其**唯一权威依据是本仓库自己的一手约定 (a)**；把它推广成通用最佳实践则是【推断】。

### 推荐答案（Q3）

**用“值 + 独立的 tri-state 状态字段”，不用裸 `None`、不用缺键，也不用把 `None` 与“判定为否”混在同一个字段里。** 具体落到一步的记录上：

1. 每个“可能有、可能没有”的测量，写成 **两个字段**：`x`（真实测量值，未测时给**固定形状的、可识别的哨兵或 NaN**）与 `x_status ∈ {measured, not_applicable, unmeasured}`。名称可以复用本仓库已有词汇：`measured / unmeasured` 取自 `tracking_evaluator.py:402-403`，`not_applicable` 对应用户在 OFF-02 中已经接受的 `indeterminate` 语义（【文档】`docs/specs/off02_obb_geometry_admission_spec.md:55-57`）。
2. **不要用“键缺席”**：`calibration_record.py:203` 已明确“缺席与显式 null 等价”，本仓库的规范化传统是不让“写不写键”改变身份；而且【JAX 源码】显示 pytree 层字段消失会改 treedef（Q2 第 6 条），在 JIT 边界上代价更硬。
3. **不要让 tri-state 依赖被追踪的值**（Q2 第 6 条实测会 `TracerBoolConversionError`）。状态要么由 host 侧在调用前决定（作为 static/仅关键字参数），要么在内核里用**固定元素数的整数枚举数组**表达。
4. 评价器侧的 tri-state 结果（`admission_counts`/`overlap_counts` 的 `unmeasured`）**已经是对的**，问题只在生产者从不提供 `admission_ok`/`overlap`/`obstacle_clearance_m`。因此这次改动的正确目标是**把生产者真的能测的量接上去**，而不是把评价器改成“缺测就通过”——后者与 `docs/tracking_evaluation.md:8`、`docs/specs/off02_obb_geometry_admission_spec.md:68` 的 fail-closed 约定直接冲突。

置信标记：**已验证**（本仓库约定部分）；外部“未测如何表达”一节，**无权威依据**（已就地写明）。

---

## Q4. “逐步测量放返回记录、跨步记忆留模块状态”这个切分站得住吗？

### 外部一手来源

- 【JAX 文档】[Common Gotchas in JAX](https://docs.jax.dev/en/latest/notebooks/Common_Gotchas_in_JAX.html)：**“JAX transformation and compilation are designed to work only on Python functions that are functionally pure”**，并解释 **“Allowing mutation of variables in-place makes program analysis and transformation difficult.”**、**“JAX requires that programs are pure functions.”**，以及“all the input data is passed through the function parameters, all the results are output through the function results”。
- 【JAX 源码】`jax/_src/api.py:17-21`（模块 docstring）：**“The transformations here mostly wrap internal transformations, providing convenience flags to control behavior and handling Python containers of arguments and outputs. The Python containers handled are pytrees … which include nested tuples/lists/dicts, where the leaves are arrays.”**
- 【JAX 文档】[jit-compilation](https://docs.jax.dev/en/latest/jit-compilation.html)：**“cached code will be used only for the same values of arguments labelled as static”**、**“If any of them change, recompilation occurs.”**

**关键限定**：这些来源约束的是**被 jit 包裹的 `fun`**。本仓库 facade 的 `last_*` 赋值全部发生在 **host 侧的 Python 方法体**里（`jax_control_facade.py:465-467`、`:581-583`、`:647-650`、`:753-782`），**不在 jit 内**，因此**不违反**上述纯粹性要求；把它们说成“违反 JAX 函数式要求”是错的。真正的代价不是违反规定，而是：**这些状态无法成为 pytree 的一部分，因此不参与缓存键、不参与跨设备/跨进程传递，也不能被 `jax.jit` 的返回值语义覆盖**（【推断】）。

**来源集合缺口（明确说明）**：本文限定的来源集合中**没有**任何 PEP 或官方文档专门讨论“per-step 测量 vs 跨步记忆”的接口切分。这一条只能靠本仓库自己的一手先例来判。

### 本仓库已有的一手先例：跨步状态本来就存在，而且是**显式传递**的

- 【代码】`portable_oscbf/work/qpax_warmstart.py:31-44`：`WarmStartState(NamedTuple)` 是“前一步 QP 解的原对偶状态”，docstring 写 **“This adapter preserves the same predictor-corrector method, but accepts the preceding converged state as an input to a surrounding JIT control loop.”** —— **跨步记忆被当作显式 pytree 输入/输出，而不是模块状态**。
- 【代码】`portable_oscbf/work/qpax_warmstart.py:46-53`：`empty_warm_start_state` **“Create an invalid state; the first control period cold-starts qpax.”** —— 跨步记忆带一个显式的“还没有有效前值”状态（与 Q3 的三态同构）。
- 【代码】`jax_control_facade.py:224` 声明 `self._last_u_safe`，在 `:814`（默认输入缓存路径）与 `:869`（`_normalise_obstacle_inputs`）被读作 `u_safe_prev`，在 `:465`/`:581`/`:647` 被写。用途是速率限幅与时间近端项。**这是本仓库唯一真正参与控制计算的跨步记忆。**
  - **2026-09-16 更正**：时域近端项**在生产中**的生效位置是 `portable_oscbf/work/oscbf_velocity_config.py:216-218`（`P()` 加 `diag(self._temporal_weight_sq())`）与 `:229-231`（`q()` 减 `self._temporal_weight_sq() * u_safe_prev`），由生产 profile 的 `temporal_lambda=0.2`（`config/oscbf_controller.yaml:37`）启用。本文初稿把 `portable_oscbf/work/qpax_solver.py:148-151`/`:187-191` 当作该作用点，**引用位置写错了**：`qpax_solver.py` 在本仓库没有任何导入者（连测试都没有，仅有 `portable_oscbf/README.md:36` 与 `OSCBF_PORTING_GUIDE.md:75/190/629` 的存档描述），所以那段时序近端项是死代码。结论不变（`u_safe_prev` 确实进入 QP 目标），但引用以本更正为准。
- 【代码】`portable_oscbf/work/controller_step_cache.py:12-20`：`RobotStepCache(frozen dataclass)` 是**一步之内**的运动学复用缓存（`q`、`T_all`、`ee_pos`、`ee_rot`、`J_s`、`J_pos`、`J_full`），既不是测量也不是跨步记忆，属于“内部中间量的显式化”先例。

### 21 个 `last_*` 的分类（含 `file:line` 证据）

| # | 属性 | 声明 | 写入 | 读取 | 分类 |
|---|---|---|---|---|---|
| 1 | `last_qp_ok` | `facade:203` | `:753` | 无外部读取 | 逐步测量 |
| 2 | `last_min_obs_dist` | `:204` | `:754` | 无外部读取 | 逐步测量（未启用障碍时是哨兵，**不可当净空**） |
| 3 | `last_min_esdf_dist` | `:205` | `:755` | `perception_demo.py:307`；`test_jax_esdf_cbf.py:93` | 逐步测量 |
| 4 | `last_rate_constraint_violation` | `:210` | `:756` | `test_jax_rate_limit.py:48,69,72` | 逐步测量（速率约束的真实松弛） |
| 5 | `last_rate_slack` | `:211` | `:758` | `test_jax_rate_limit.py:51` | 逐步测量（`:206-209` 自述为 #4 的兼容别名） |
| 6 | `last_rate_solver_slack` | `:212` | `:759` | `test_jax_rate_limit.py:52,70` | 逐步测量（求解器内部松弛，明确不是命令违反） |
| 7 | `last_qp_active_count` | `:213` | `:760` | `test_jax_tracking_step.py:55` | 逐步测量（求解诊断） |
| 8 | `last_qp_iterations` | `:214` | `:761` | `test_jax_tracking_step.py:56` | 逐步测量（求解诊断） |
| 9 | `last_qp_warm_start_used` | `:215` | `:762` **恒为 `False`** | **无**（`test_jax_tracking_hard_stop.py:63` 是同名局部变量） | **内部中间量 / 死字段**：从未反映真实热启动，且无读者 |
| 10 | `last_qp_primal_residual` | `:216` | `:763` | **`oscbf_controller.py:560`（生产唯一读取）**；`test_jax_tracking_step.py:57` | 逐步测量（求解诊断） |
| 11 | `last_qp_dual_max` | `:217` | `:766` | `test_jax_tracking_step.py:63` | 逐步测量（求解诊断） |
| 12 | `last_qp_terminal_kkt_residual` | `:218` | `:764` | `test_jax_tracking_step.py:61` | 逐步测量（求解诊断） |
| 13 | `last_qp_terminal_kkt_accepted` | `:219` | `:765` | `test_jax_tracking_step.py:62` | 逐步测量（求解诊断） |
| 14 | `last_cbf_h_delta_norm` | `:220` | `:775-777` | `test_jax_tracking_step.py:100` | **跨步派生量**（依赖 #16 `_last_cbf_h`；首次为 0.0，`h_vals=None` 时为 NaN） |
| 15 | `last_cbf_grad_delta_norm` | `:221` | `:778-780` | `test_jax_tracking_step.py:101` | **跨步派生量**（依赖 #17） |
| 16 | `_last_cbf_h` | `:222` | `:781` | `:776`/`:777`（内部） | **跨步记忆**（原始） |
| 17 | `_last_cbf_grad` | `:223` | `:782` | `:779`/`:780`（内部） | **跨步记忆**（原始） |
| 18 | `_last_u_safe` | `:224` | `:465`/`:581`/`:647` | `:814`、`:869` → `u_safe_prev` | **跨步记忆**（唯一参与控制计算的一条） |
| 19 | `last_qp_candidate` | `:225` | `:466`/`:582`/`:648` | `test_jax_rate_limit.py:47`、`test_jax_frozen_qp_problem.py:59` | 逐步测量（未过健康门的候选命令） |
| 20 | `last_path_metrics` | `:226` | `:650` | `perception_demo.py:387`（生产读取）；`test_jax_tracking_hard_stop.py:116` 设的是**同名不同前缀**的 `_last_path_metrics` | 逐步测量的**派生快照**（17 个键，`:650-680`） |
| 21 | `last_delta_slack` | **未在 `__init__` 初始化** | 仅 `:467`（`step`）、`:583`（`tracking_step`） | 仅 `docs/planning/oscbf-reuse/research/5-redundancy-evidence/compare.py:55`（研究脚本） | **内部中间量 / 缺陷**：生产路径 `path_tracking_step` **不写它**（`:647-650` 只写 `_last_u_safe`/`last_qp_candidate`/`last_path_metrics`），所以生产下它是“未定义或陈旧”；且无生产/测试读者 |

**表中“读取”列的时效限定（2026-09-16 复核）**：第 4、5、6 行的读者全部位于 `portable_oscbf/tests/test_jax_rate_limit.py`，该模块有模块级 `pytestmark = pytest.mark.skip`（`:7`，`newaxis` 不可导入），**断言从不执行**；第 9 行同时被 `test_jax_tracking_hard_stop.py` 命中，该模块在 `:9` 同样模块级 skip。因此这些属性名**对 skip 模块承重、对执行中的验收不承重**。真正**活**的读者只有第 14、15 行（`portable_oscbf/tests/test_jax_tracking_step.py:100-101`，通过中）与第 18 行 `_last_u_safe`（`portable_oscbf/tests/test_jax_default_input_cache.py:16,25`，通过中）。

**由这张表推得的两条实际问题**（非本项目文档所述，属本文【代码】核验）：

- 侧通道存在**入口不对称**：`path_tracking_step` 少写 `last_delta_slack`，而生产只走 `path_tracking_step`；任何“靠 `last_*` 拿这一量的消费者”在真机上会拿到陈旧值或 `AttributeError`。这正是侧通道形态本身带来的风险。
- 侧通道有**死字段**（#9、#21），说明它缺少“谁读它”的压力测试——返回记录一旦成为唯一通道，这类字段会被接口检查自然淘汰。

### 推荐答案（Q4）

**切分本身是连贯的，但要把界线画在“是否参与下一步的数值计算”上，而不是“是否叫 `last_`”。**

- **逐步测量（→ 返回记录）**：上表分类为“逐步测量”的 #1~#8、#10~#13、#19、#20。它们只依赖本步输入与该步求解，放进记录后可被评价器/日志/测试统一消费，并且**不再需要 `last_*` 镜像**。
- **跨步记忆（→ 显式状态，不是模块属性）**：#16~#18。本仓库已有两条一手先例支持“显式化”：`WarmStartState`（`qpax_warmstart.py:31-44`，连有效性都显式）与 `u_safe_prev` 参数（`oscbf_velocity_config.py:216-231`/`facade:814,869`）。建议把 `_last_u_safe`、`_last_cbf_h`、`_last_cbf_grad` 收进一个**具名状态结构体**，作为 `path_tracking_step` 的显式入参与出参；这样跨步记忆与逐步测量在类型上分开，且 Q2 的实测保证这不付出 jit 代价。
  - **2026-09-16 更正**：“NamedTuple 或 frozen dataclass”在**这个位置**上并不等价（第二轮实测）：`typing.NamedTuple` 是已注册的 pytree 节点，可直接进出 `jax.jit`；frozen `@dataclass` 未注册时整个实例是**一个 leaf**，进 jit 直接 `TypeError`，必须 `jax.tree_util.register_dataclass` 才能用。若这个状态结构体要跨 jit 边界，**只能取 NamedTuple**（或注册过的 dataclass），见 `control-step-record-round2-20260916.md` Q3 与 `control-step-record-round2-prototype-20260916.md` §G。
- **内部中间量**：#9、#14、#15、#21。其中 #14/#15 是#16/#17 的派生量，应随跨步状态一起返回或干脆删除；#9、#21 应删除而不是迁移——它们是本次核验发现的死/缺陷字段，迁移会把缺陷固化进新接口。
- **明确写进设计的一句话**：**“`last_*` 不是接口，是缓存。”** 任何需要跨模块稳定读取的量都必须出现在返回值里。这与【JAX 文档】“all the results are output through the function results”方向一致，也与本仓库 `WarmStartState` 的既有做法一致。

置信标记：**有依据的推断**（分类与 `file:line` 为【代码】实测；切分原则由 JAX 文档 + 本仓库先例支持，但“该切在哪里”没有一手来源直接规定）。

---

## Q5. 本仓库有没有“接口变了怎么迁测试”的成文标准？爆炸半径多大？

### (a) 有没有成文标准

**没有一份专门规范“接口变更时如何迁移测试”的文档。** 逐处核验结果：

- 【文档】`AGENTS.md`「构建和验证」只规定入口与工具（`bash build_aeb_moveit.sh`、`bash scripts/agent_check.sh` 是启发式检查、“不能代替行为测试”、`bash run_all_tests.sh` 用于完整集成验证）、以及“不要为纯文档修改启动仿真”。“交付”节只要求“行为或运行方式改变时同步相关文档”。**没有测试迁移流程。**
- 【文档】`CODING_STANDARDS.md` 提供的是**审查检查项**（如 `:11` 要求“确认已有契约测试覆盖变更的接缝，避免新增独立常量副本”；`:16` 要求“已有检查足够时只补缺失的接缝验证，不叠加相同规则”）。它是审查清单，不是迁移规程。
- 【文档】`docs/ONBOARDING.md` 与 `docs/agents/domain.md`、`docs/agents/issue-tracker.md` 均无迁移测试的条目（`docs/agents/domain.md:43` 只要求命名与 `CONTEXT.md` 一致）。
- 【文档】`LESSONS_LEARNED.md:75` 有一节标题就是 **「迁移与验收流程」**，但其下的条目全是**运行时行为**教训（回调组、交接收敛、参考超前量），**没有一条讲接口/字段迁移**。

**真正算得上标准的只有两条，且都是“教训”而非“规程”**：

1. 【文档】`LESSONS_LEARNED.md:177-181`（「遗留参考测试要跟随语义演进」）：现象是 M7 移除硬终端 KKT 检查后全量套件 “从 M0 的 0 失败退化为 3 失败”；修复是 **“按当前语义重写断言并注释原因”**；教训是 **“参考实现的测试不是永久的契约；目标架构每改一处语义，就要同步审计哪些旧测试在测已删除的行为，不能靠 skip 悄悄掩埋。”** 同页 `:174-176` 给出方法：**“拓扑变化后必须 grep 所有依赖旧计数的测试；用‘结构派生’的断言（18+14+10+1+…）比魔法数字好。”**
2. 【文档】`LESSONS_LEARNED.md:10-17`（「弹性 QP 与速率限幅组合崩溃」）：现象正是 **`TypeError: solve_qp_elastic() takes 5 positional arguments but 6 were given`**——一个**位置参数数量不匹配**的真实事故；教训是“求解器对象在配置开关下会换签名/换语义；每一个分支都要走一遍”。**这是本仓库已有的、与“位置元组/位置签名”最接近的一手教训。**

**换一个角度看，本仓库对“接口漂移”的最强一手证据是回退约束**：【文档】`docs/planning/oscbf-reuse/handoffs/44-offline-tracking-evaluation.md:127` 明文 —— **“控制器与内核的遥测接口需要成组回退，不能只撤销返回元组一侧。”** 这等于承认：**该接缝的四形态是一次原子改动，任何一侧单独回退都会破**。它是本仓库对“四形态耦合”成本最直白的一手记录。

**关于“已有教训”这一问的直接回答**：我按关键词（`位置参数`/`positional`/`元组`/`tuple`/`dict`/`键`/`接口漂移`）检索了 `LESSONS_LEARNED.md`、`CODING_STANDARDS.md`、`docs/ONBOARDING.md`、`docs/agents/`、`docs/planning/oscbf-reuse/`（含 `research/`、`handoffs/`、`issues/`）。**结论：本仓库已学到“位置参数数量变化会崩”（`LESSONS_LEARNED.md:10-17`）和“接口必须整组回退”（`handoffs/44:127`），但**没有**把“内核返回 42 元组 / 控制器手写 29 键 dict / 评价器 12 项别名表”这一形态本身登记为教训。** 也就是说，这次要处理的问题是**已知风险类型的一个未登记实例**，不是全新未知。

### (b) 爆炸半径实测（`tests/test_oscbf_controller_smoke.py`，716 行）

方法：AST 遍历，分别统计“以字符串常量下标或 `.get("字面量")` 读取 step dict 的键”与“访问 `_loop`/`_path_state`/`_evaluator`/`_control_tick`/`_make_tracking_evaluator`/`_limits`”的行与测试函数。

| 度量 | 数值 |
|---|---|
| 文件总行数 | 716 |
| 读 step-dict 字符串键的**行** | **16** |
| 访问私有成员的**行** | **15** |
| 两类**同时**出现的行 | **0**（互不重叠） |
| 两类并集 | **31 行（占 716 行的 4.3%）** |
| 读 step-dict 字符串键的**测试函数** | **5**（`test_production_config_refactor_preserves_control_outputs_from_8479740`、`test_hold_commands_continue_during_background_report`、`test_step_once_returns_valid_safe_state`、`test_progress_snapshot_reports_tracking_state`、`test_perf_report_p95_within_budget`） |
| 访问私有成员的**测试函数** | **6**（`test_node_starts_without_move_group`、`test_tracking_sentinels_reach_the_actual_facade_call`、`test_runtime_snapshot_matches_consumers_and_records_version_identity`、`test_hold_commands_continue_during_background_report`、`test_tracking_evaluator_integration`、`test_progress_snapshot_reports_tracking_state`） |
| 两类都碰的测试函数 | **2** |
| 该文件测试函数总数 | 15 |

按键细分（行号）：`q_next` 3 行（278/472/694）、`err_6d` 3 行（284/476/478）、`qp_ok` 2 行（306/479）、`u_safe` 2 行（475/477），其余 `reference_at_endpoint`(408)、`min_obs_dist`(480)、`cross_track_error_m`(638)、`feedrate_m_s`(290)、`path_progress_m`(296)、`qp_primal_residual`(302) 各 1 行。**只有 10 个 step-dict 键被该测试读**（生产 dict 有 29 个键，`_control_tick` 读 19 个，全文件合计 21 个键有读取点）。（`_control_tick` 实测读 19 个键：`q_next`、`u_safe`、`err_6d`、`ee_pos`、`reference_position_m`、`qp_ok`、`reference_source_time_s`、`reference_at_endpoint`、`cross_track_error_m`、`feedrate_m_s`、`feedrate_nominal_m_s`、`feedrate_joint_limit_m_s`、`feedrate_cbf_limit_m_s`、`feedrate_rate_limit_m_s`、`feedrate_tool_axis_limit_m_s`、`feedrate_endpoint_brake_limit_m_s`、`gamma`、`limiting_reason_code`、`step_latency_ms`。）

**其他相关计数（供范围决策）**：
- 直接读 facade 侧通道的测试共 **7 个函数 / 5 个文件**：`test_jax_default_input_cache.py:1`（`_last_u_safe`）、`test_jax_esdf_cbf.py:1`（`last_min_esdf_dist`）、`test_jax_frozen_qp_problem.py:1`（`last_qp_candidate`）、`test_jax_rate_limit.py:2`（`last_qp_candidate`/`last_rate_constraint_violation`/`last_rate_slack`/`last_rate_solver_slack`）、`test_jax_tracking_step.py:2`（8 个求解诊断量）。
- 生产侧读侧通道的**只有 1 处**（`oscbf_controller.py:560`），另有 `perception_demo.py:307`、`:387` 两处（demo 工具，非控制链）。
- 断言 jit 缓存条目数的测试有 3 处（`test_elastic_qp.py:83`、`test_jax_esdf_cbf.py:96`、`test_jax_esdf_cbf.py:156`，都断言 `== 1`）。**这些是本次改动最容易误伤的一类**：任何改变输出 pytree 结构的写法都可能让缓存条目从 1 变多（Q2 第 6 条实测）。

### 推荐答案（Q5）

**没有成文标准；应把 `LESSONS_LEARNED.md:174-181` 的两条既有教训直接套用，并把“接口形态”这一新实例补登进去。** 具体：

1. 迁移断言按 `LESSONS_LEARNED.md:179` 的方法：**grep 所有依赖旧形态的测试，改成结构派生断言并注释原因，不用 skip 掩埋**。就本次而言，第一步就是 grep `step_once`、`path_tracking_step`、`step_from_result`、`\.[a-z_]*last_[a-z_]*` 四个模式，得到本文 Q1/Q5 的清单。
2. 把 `handoffs/44-offline-tracking-evaluation.md:127` 的“成组回退”升级为显式约束：**四形态与两通道必须在同一次改动内切换**，回退说明要写“同组文件”。这比“先改内核、再改控制器、最后改评价器”更符合本仓库已有记录。
3. 侧通道的 7 个测试 + 3 个缓存断言是**独立的门**：迁移时应把它们改为断言**返回值**（而不是 `loop.last_*`），并在迁移完成后**删除对应 `last_*`**，用“无读者”来自证落地；否则会出现 Q4 表里 #9/#21 那类死字段长期残留。
4. **建议在 `LESSONS_LEARNED.md` 补一条**（这是本次研究认为最该回流的产出）：现象=同一份“一步数据”在四个接缝上以四种形态存在且靠位置/字符串键对齐；根因=生产者按位置返回、中间层手写 dict、消费者靠别名表容错，三处都没有编译期检查；修复=（待实施）统一具名结构体；教训=**跨模块的“一步数据”只允许有一个具名定义，任何中间层不得手写第二份键集合**。诚实地说：**这条目前只是本文建议，仓库里还没有这条教训**——而 Q1/Q5 的证据表明它是应当被登记的。

置信标记：**已验证**（“没有成文标准”与所有计数均为逐文件核验）；第 4 条为**有依据的推断**（是否入册属用户决策）。

---

## 来源清单

### 一手来源

**本仓库源码（读取时工作树）**
- `portable_oscbf/work/jax_kernel_factory.py:392-420`、`:704-778`（`tracking_step` 24 参 / `path_tracking_step` 21 参 / 42 元素返回）
- `portable_oscbf/work/jax_control_facade.py:55-90`（`JaxPathTrackingResult` 30 字段）、`:203-226`（20 个 `last_*`/`_last_*`）、`:427-467`、`:524-583`、`:589-668`（解包）、`:748-782`（`_update_qp_diagnostics`）、`:814`、`:869`
- `src/robot_safecontrol_moveit/oscbf_controller.py:538-583`（29 键 dict）、`:560`（唯一生产侧通道读取）、`:648-700`（`_control_tick` 读 19 键）、`:797-840`（`progress_snapshot`）
- `src/robot_safecontrol_moveit/tracking_evaluator.py:64-78`、`:81-152`（12 项别名表在 `:97-109`）、`:402-403`、`:501-502`
- `src/robot_safecontrol_moveit/calibration_record.py:193-209`（`_canonical_object`「缺席与显式 null 等价」）、`:417`、`:766`
- `portable_oscbf/work/obb_geometry_admission.py:65-80`、`:327-330`、`:438-449`
- `portable_oscbf/work/qpax_warmstart.py:31-53`、`portable_oscbf/work/controller_step_cache.py:12-20`、`portable_oscbf/work/safety_snapshot.py:33-41`
  - 2026-09-16 更正：本行初稿还列了 `portable_oscbf/work/qpax_solver.py:148-151`，已删除——该文件无任何导入者，不构成“先例”。另需注意 `controller_step_cache.py` 的 `RobotStepCache`/`build_robot_step_cache` 在模块外**零调用点**（只有 `point_jacobian_from_spatial` 被 `point_cloud_obstacles*.py`、`dynamic_obstacles.py` 使用），形态可引为设计先例，但不能说成“生产中正在使用的模式”。
- 测试：`tests/test_oscbf_controller_smoke.py`（716 行）、`tests/test_tracking_evaluator.py:27-37`、`portable_oscbf/tests/test_jax_tracking_step.py:94-96`、`portable_oscbf/tests/test_elastic_qp.py:83`、`portable_oscbf/tests/test_jax_esdf_cbf.py:96,156`

**本仓库文档**
- `docs/adr/0008-tracking-evaluation-scope-and-angle.md`（结尾段：缺失数据→证据不足）
- `docs/adr/0009-calibration-record-schema-identity-admission.md:17`、`:50-68`
- `docs/specs/off02_obb_geometry_admission_spec.md:45-68`、`:95-140`
- `docs/tracking_evaluation.md:8`、`:9`、`:31`、`:34`
- `docs/planning/oscbf-reuse/research/off02-off09-interface-20260915.md`（开头「结论」节）
- `docs/planning/oscbf-reuse/handoffs/44-offline-tracking-evaluation.md:127`（「控制器与内核的遥测接口需要成组回退」）
- `docs/planning/oscbf-reuse/handoffs/45-calibration-record.md:19`
- `docs/planning/oscbf-reuse/issues/07-migration-order.md`（「每一迁移段的交接模板」）
- `LESSONS_LEARNED.md:10-17`（位置参数数量事故）、`:174-181`（遗留测试跟随语义演进）
- `AGENTS.md`「构建和验证」「交付」；`CODING_STANDARDS.md:11,16`；`docs/ONBOARDING.md:55-73`

**JAX 官方文档 / 源码 / 发布说明**
- 本机安装源码（jax 0.6.2、jaxlib 0.6.2，`/home/lsn/.local/lib/python3.10/site-packages/jax`）：`_src/tree_util.py:914-1022`（`register_dataclass` 及其 docstring `:954`、`:974`、`:1019`、`:1022`）；`_src/api.py:17-21`
- tag 源码核验：`https://raw.githubusercontent.com/jax-ml/jax/jax-v0.4.26/jax/_src/tree_util.py`（无 `register_dataclass`）与 `.../jax-v0.4.27/jax/_src/tree_util.py`（有）
- CHANGELOG：`https://raw.githubusercontent.com/jax-ml/jax/main/CHANGELOG.md`（0.4.32「Changes」校验项；0.4.36「New Features」内联 static）
- [JAX — Just-in-time compilation](https://docs.jax.dev/en/latest/jit-compilation.html)（static 参数与缓存/重编译）
- [JAX — Common Gotchas in JAX](https://docs.jax.dev/en/latest/notebooks/Common_Gotchas_in_JAX.html)（纯粹函数；输入走参数、结果走返回值；禁止 in-place mutation）

**Python 官方规范 / PEP**
- [PEP 484 — Type Hints](https://peps.python.org/pep-0484/)（「The typing Module」：`Optional[t] == Union[t, None]`；union 简写；`None` 默认值规则）
- [PEP 557 — Data Classes](https://peps.python.org/pep-0557/#frozen-instances)（frozen 行为、`FrozenInstanceError`、`__getattr__`/`__setattr__` 冲突抛 `ValueError`）
- [PEP 387 — Backwards Compatibility Policy](https://peps.python.org/pep-0387/)（「Backwards Compatibility Rules」：受保护项含参数/返回值的**位置**；不受保护项含 `_` 前缀名与 **Test suites**；「Making Incompatible Changes」：逐版本渐进的过程）

**本机只读实验（【实验】标记，命令为 `python3 -c`，未写文件）**
- NamedTuple / frozen dataclass 从 `@jax.jit` 返回；`_cache_size()` 变化；`.lower().as_text()` 与 `make_jaxpr` 的 unified diff；`float`/`bool`/嵌套 `dict` 字段；`None` vs 数组的 treedef 与重编译；`TracerBoolConversionError`；`tree_flatten` treedef 对照；`pytest --collect-only` 计数（568 收集 / 16 导入失败）

### 二手来源（明确标注，本文不以其为权威依据）

- Martin Fowler, *Strangler Fig Application*，<https://martinfowler.com/bliki/StranglerFigApplication.html>：把该做法描述为 “a gradual approach to legacy modernization”，新代码建在 “on top of, yet separate to the legacy code base”，靠 “transitional architecture to allow the new and legacy system to coexist” 并存，且该过渡架构 “will go away once the modernization is complete”。**性质：行业博客，非本文规则 1 认可的权威来源；仅用于说明“新旧并存、逐段搬运”是通行做法，Q1 的建议不依赖它。**
- 本文件未引用任何其他博客或二手总结。

### 明文缺口（避免被当作已证结论）

1. **register_dataclass 的“首次引入版本”没有发布说明条目**：0.4.26 无 / 0.4.27 有 是本机对 tag 源码的核验结果；CHANGELOG 只记录了 0.4.32 与 0.4.36 的变更。
2. **“未测量该怎么表达”没有外部一手来源**：PEP 484/557 与 JAX 文档都不涉及该问题；唯一权威依据是本仓库自身约定（Q3 (a)）。
3. **“per-step vs 跨步记忆的切分原则”没有外部一手来源**：JAX 文档只约束 jit 内的纯粹性，而本仓库的 `last_*` 写发生在 jit 外的 host 代码，不受该约束。
4. **“内部接缝重构该分几段”没有一手来源**：PEP 387 是 Python 标准库的公共 API 策略，把它套到本仓库内部接缝是本文的【推断】。
5. **本仓库没有“接口变更如何迁移测试”的成文标准**（Q5 (a)）。
6. 本文未运行 ROS、未运行 JAX 数值闭环、未做真机或 sim 动作；测试计数来自 `--collect-only`（不执行用例），因此**不代表这些用例当前会通过**。
