# 一步记录原型：实测答案

日期：2026-09-16。配套研究见同目录 `control-step-record-20260916.md`（Q1–Q5 的文献与源码论证）。
本文只记录**原型跑出来的数**，不重复论证。

产物（均为未提交的工作树文件，属可丢弃原型）：

| 文件 | 作用 |
|---|---|
| `portable_oscbf/scripts/prototype_step_record.py` | 实验：在真实内核边界上把 42 槽位置元组换成具名结构体，并清点槽与测量 |
| `portable_oscbf/scripts/prototype_step_record.json` | 上面的结果（本文全部数字的来源） |
| `portable_oscbf/scripts/prototype_step_record_model.html` | 可点击试玩的记录形态模型（纯逻辑模块 + 页面外壳） |

运行方式：`python3 portable_oscbf/scripts/prototype_step_record.py`。**不修改仓库任何源码或测试**，只在进程内替换 `loop._path_tracking_fn` 让两种返回形态跑同一组输入。

## 方法上的一个坑（先说，因为它影响结论）

原型最初用 `jax.jit(lambda *a: Record(*tuple_fn(*a)))` 包了一层**额外的 jit**，于是测得「命名返回与元组返回差 1.5e-09」，看起来像是命名有数值代价。

这是原型自己引入的假象。补了第三条对照臂（**不**再套 jit，即真实改动会写成的样子：内核自己返回结构体，只 jit 一次）之后：

| 与位置元组相比 | 最大逐字段差 |
|---|---|
| 多套一层 jit 的包装 | **1.494e-09** |
| 不套 jit 的包装（生产改动的形态） | **0.000e+00**（逐位相同） |

结论：**1e-9 的差全部来自多出来的那次 jit 边界，与命名无关。** 命名本身逐位等价。

## 答案 1：命名结构体可以从 jit 返回，且是位置元组的直接替换

- 42 字段 NamedTuple 的实例**仍然是 tuple**：`isinstance(x, tuple)` 为真、`tuple(x)` 可按位置解包、长度等于槽数。
- facade 的生产解包**是按位置**写的（`jax_control_facade.py:625` 一带）。原型把具名版本换上后，facade 原样跑完 30 步**没有报错、没有改动**——这就是「直接替换」的证据。
- 逐字段对比 30 个已发布字段，不套 jit 时**逐位相同**。
- `jax.jit(...)._cache_size()` 两种形态都是 **1**，缓存条目数不变。

## 答案 2：成本在噪声内

交替、配对测量（每臂 30 次，同一进程、同一输入）：

| 形态 | 中位数 | p95 | 配对中位差（相对元组） |
|---|---|---|---|
| 位置元组 | 7.096 ms | 7.786 ms | — |
| 具名（多套一层 jit） | 7.173 ms | 7.691 ms | +0.002 ms |
| 具名（不套 jit） | 7.249 ms | 7.853 ms | +0.251 ms |

逐次运行的配对中位差在 −0.09 与 +0.25 ms 之间来回翻号（不同次运行），步进中位数都在 7.0–7.2 ms。**没有可测的成本差异**；这也与「改的是容器的名字，不是计算」相符。

## 答案 3：42 个槽全部有归宿，没有「内部丢弃」

用 AST 建 facade 的定值-使用图并**沿值流动方向**传递闭包（这点很关键：`next_state_np = np.asarray(next_path_state)` 若按 AST 正向读，会让 `next_path_state` 看起来是没人消费的叶子）。

| 归宿 | 个数 | 例子 |
|---|---|---|
| 只进返回结果 | 12 | `q_next`、`err_6d`、`ee_pos_before` |
| 只进 `last_*` 副作用通道 | 11 | `cbf_grad`、`primal_residual`、`qp_iterations` |
| 两个通道都有 | 19 | `u_safe`、`qp_ok`、`min_dist`、`h_vals` |
| 两者都没有 | **0** | — |

即：**「42 个槽里有若干根本没被下游用到」这个说法不成立**，每个槽都至少到达一个通道。问题不是有槽被丢弃，而是**同一步数据被同时送达两条通道**，而两条通道的形状不同（30 字段 dataclass / 21 个可变属性）。

## 答案 4：副作用通道的可信度比研究文档说的更低

facade 里 `self.x = ...` 形式的赋值共 60 处，其中 `last_*`/`_last_*` 诊断家族 **21 个**（与研究文档一致）。

- 在 facade 之外被读到的：**15 个**。
- **任何地方都没有读者**：**6 个** —— `_last_cbf_grad`、`_last_cbf_h`、`last_delta_slack`、`last_min_obs_dist`、`last_qp_ok`、`last_qp_warm_start_used`。
- 被 **`src/robot_safecontrol_moveit/` 生产代码**读到的：**3 个** —— `last_qp_primal_residual`（`oscbf_controller.py`）、`last_min_esdf_dist` 与 `last_path_metrics`（`perception_demo.py`）。

`path_tracking_step` 只写 `_last_u_safe`/`last_qp_candidate`/`last_path_metrics` 三个。**所以「副作用通道」在生产路径上实际承载的信息非常少**：真正被生产读取的只有 3 个属性，其中 2 个只在 demo 工具里。这削弱了「必须保留副作用通道」的理由。

## 答案 5：三个槽没有任何生产方，且线上「通过」不可达

对照 `tracking_evaluator.py` 的别名表与直接读取，把评价器报告的 14 个测量逐个分类（生产者是否真的送来键）：

| 分类 | 个数 | 名称 |
|---|---|---|
| 生产方真的给了 | 8 | `online_cross_track_m`、`feedrate_m_s`、`actual_tangent_speed_m_s`、`obstacle_margin_m`、`delta_slack`、`qp_primal_residual`、`step_latency_ms`、`source_time_s` |
| 评价器自己算的（生产方不给） | 3 | `pos_error_m`、`tool_axis_error_rad`、`cross_track_m` |
| **无人生产** | **3** | **`obstacle_clearance_m`、`admission_ok`、`overlap`** |
| 由 `path_state` 推 | 2 | `projected_progress_m`、`reference_progress_m` |

其中 `admission_ok` 与 `overlap` **只有 `tests/test_tracking_evaluator.py` 会设**，生产链从不提供。

把它们代进 `tracking_evaluator.py:501-502` 的判定式（`pass` 要求 `accepted == n` 且 `clear == n`），得到：

- 用**真实数据**跑：`online = insufficient_evidence`，永远。
- 把 `admission_ok` 与 `overlap` 都填成「已测」：`online = pass`。

**也就是说，线上要拿到 `pass`，唯一途径是把这两个没人生产的量填出来。** 这不是评价器的错——评价器守规矩（缺测记 `unmeasured` 而不是推断通过）；是**生产者从来没接上这两个量**，于是整条线上判定链路停在「证据不足」。

这条也直接回答了该往哪改：**要动的是生产方（把真能测的量接上去），不是把评价器放宽。**

## 交互式模型（可点击试玩）

`prototype_step_record_model.html` 把上面第 5 条做成了可拨动的模型。它的纯逻辑模块（文件里第一个 `<script>`）**只调用不发散**，不含 DOM，可直接搬进真实代码。模型算出来的是：

| 记录写法 | 五种情况能无损表达 | 键集合稳定 |
|---|---|---|
| 缺测就删掉键 | 4 / 5 | 否 → 结构随数据变，要重编译 |
| 缺测写 null | 4 / 5 | 是 |
| **值 + 独立状态字段** | **5 / 5** | **是** |

两种朴素写法丢掉的是同一件事：**「缺测、生产方什么都没送」与「缺测、但生产方送出了 1.0（哨兵）」被写成同一份数据**，于是下游再也分不清那个 `1.0` 是哨兵还是读数。这正是 `docs/tracking_evaluation.md:31`「不能从裕度反推净空」要防的失效，而第三种写法则把它变成可表达的。

页面里的引导场景可以直接复现上面每一条。逻辑模块的正确性用 node 做过穷举（16 槽 × 3 状态 × 3 写法，无异常；上面的数字均为实算值）。

## 本次核验推翻或修正的两个先前说法

1. 先前记录的「facade 约 24 个 `last_*`」**不准**：实测 21 个，且其中 6 个全仓无读者。
2. 先前记录的「内核返回 42 元组，末 7 槽无下游消费者」**不成立**：沿值流动方向做传递闭包后，42 个槽 **0 个**无归宿。

另外，`_control_tick` 读的 step-dict 键数是 **19**（不是 8）。先前用「只匹配双引号」的正则统计得到 8，漏掉了日志 f-string 里用单引号写的 `step['gamma']` 一类；改用同时匹配两种引号后复算为 19，与研究文档一致。

### 对研究文档 Q4 表格第 9 行的一处修正

研究文档把 `last_qp_warm_start_used` 记为「死/缺陷字段」，并说它「从未反映真实热启动」。后半句字面为真，但**把它读作缺陷是错的**：

- `jax_control_facade.py:145-148`：`if self.qp_warm_start: raise ValueError('the custom qpax PDIP warm-start path is archived; production JAX control uses qpax baseline solves')`。
- 也就是说生产路径**根本不可能**热启动，`facade:762` 的 `last_qp_warm_start_used = False` 是一个**恒真的正确常量**。

准确定性是：**一个携带零信息的死字段**（恒为常量、且无读者），而不是「报了假账」。这个区别影响处置建议——删掉它不会掩盖任何缺陷。相关的归档模块 `qpax_warmstart.py` 有独立测试 `portable_oscbf/tests/test_qpax_warmstart.py`，删除它需要单独判断，不属于这次接缝改动。

### 一处对研究文档的补充（与 Q 系列结论直接相关）

`admission_ok` 缺的不是数据，是判据。`CONTEXT.md`「准入检查」条把它定义为「求解之后把该 tick 放弃的安全要求量与该类约束的允许额度相比」，而这两侧的输入**都已经在记录里**：`delta_slack`（弹性松弛额度，节点 dict 与 facade 都发布）与 `constraint_metrics`。缺的只是那段比较逻辑。

要注意 `tracking_evaluator` 的 `admission_ok` 与 `calibration_record.admission_ready`（`calibration_record.py:144`，标定准入，ADR 0009）**是两个不同的概念**，`CONTEXT.md` 已明确警告不要混用。前者无生产者，后者已实现。

`overlap` 与 `obstacle_clearance_m` 则不同：它们依赖解析障碍几何（球/圆柱）与臂体碰撞几何的求交，属于 OFF-02 那条线；`min_obs_dist` 是裕度而非净空，`docs/tracking_evaluation.md:31` 明令不得反推。因此这两个量**本轮接不上**。

## 限制（未验证的部分）

- 原型只跑 CPU（本机无 CUDA jaxlib），**未测 GPU**；「成本无差异」只在此条件下成立。
- 原型只在进程内替换入口，**没有真的改 facade/节点/评价器**，也没有跑 `run_all_tests.sh`。因此「替换后测试会通过」**未经验证**。
- 只跑了 30 步的路径链，未覆盖障碍、ESDF、速率限幅等分支。
- HTML 模型**未经浏览器渲染核验**（本机无可用无头浏览器）；其逻辑用 node 穷举核验，外观是静态审查。
- 未做真机、未跑 sim、未改任何硬件相关边界。
