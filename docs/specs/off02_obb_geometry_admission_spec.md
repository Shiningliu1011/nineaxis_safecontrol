# OFF-02 规格说明书：OBB 自碰撞准入与局部区域证书

状态：本地规格草案；决策已接受，未发布到外部 tracker。
日期：2026-09-15。
规范词：本文的“必须”“不得”“仅当”表示验收要求；“当前证据”表示已有本地实现结果，不扩大能力声明。

## 1. 问题

当前 OSCBF 的在线自碰撞表只包含 14 个 OBB 对，不能据此判断整个九轴构型没有自碰撞。历史反例已经证明：在线距离输出为正时，表外非相邻 OBB 对仍可能相交。单点采样无碰撞也不能证明两点之间的运动、状态误差邻域或停车包络连续无碰撞。

OFF-02 需要提供一个独立于可微距离内核的 OBB 几何事实层，并为在线表省略的非相邻对提供有边界、可复核的连续区域证据。输出必须区分点态事实、区域覆盖和未知状态，供后续 OFF-09 组合准入；OFF-02 本身不拥有锁存、恢复或命令发送职责。

## 2. 目标行为

### 2.1 配对范围

机器人 OBB 链按 `base_link, Link1, …, Link9` 排列，共 10 个盒体、45 个无序组合。

- 直接运动链相邻的 9 对不做自碰撞检测。这是按链拓扑定义的免检，不表示空间上相近的任意盒体可免检，也不表示这些相邻连杆已被证明永不接触。
- 其余 36 个非相邻对必须进入独立点态碰撞判据。
- 现有 14 对保留为 `online_cbf_subset`，只是在线 CBF 约束子集，不是完整自碰撞集合，也不是其他配对的豁免依据。
- 在线表省略的 22 个非相邻对必须逐对进入局部区域证书，或逐对报告不能认证的原因。
- Link6 必须作为独立几何对象参与检查，不得继续声明由 Link5 代替覆盖。
- `Link3/Link5` 属于非相邻对，不得永久豁免。

规范集合由以下接口给出：

```python
ALL_NONADJACENT_PAIRS       # 36 对
ONLINE_CONSTRAINT_PAIRS     # 14 对
OMITTED_NONADJACENT_PAIRS   # 22 对
```

### 2.2 点态碰撞评估

点态接口必须对一个明确绑定的九轴状态执行独立 15 轴 OBB SAT 检查：

```python
assess_point_collision(
    q,
    *,
    query_id,
    boundary,
    task_id,
    attachment_id,
    expected_model_id=None,
    numerical_tolerance_m=1e-9,
) -> PointCollisionAssessment
```

返回状态：

| 状态 | 语义 |
| --- | --- |
| `separated` | 36 个非相邻对均得到严格分离结果。 |
| `overlap` | 至少一个非相邻对相交，结果保留命中配对及 SAT 投影间隙。 |
| `indeterminate` | 输入、数值、身份或配对完备性不足，不能解释为分离。 |

请求与结果必须绑定：`query_id`、测量边界、九轴 q 及关节顺序、任务、附件、模型和完整非相邻配对策略。有效测量边界为：

```text
kernel_candidate
filtered_command
simulated_state
real_feedback
```

非有限值、维数错误、关节越限、空身份、未知边界、模型身份失配或 SAT 数值边界必须 fail closed，返回 `indeterminate`。点态 `separated` 只描述该 q，不得升级为路径、区域、停车能力或实机安全结论。

### 2.3 关节区域与连续证书

区域使用九轴轴对齐关节盒 `JointRegionSpec`：

```text
center_q
task_half_width_q
state_error_half_width_q
stop_half_width_q
geometry_error_m
required_clearance_m
task_id
attachment_id
task_region_source
state_error_source
stop_envelope_source
geometry_error_source
```

其中 J1 单位为米，J2–J9 单位为弧度。总半宽为任务、状态误差和停车三项半宽逐轴相加。所有字段必须显式、有限且可追溯；缺少物理误差、延迟或停车依据时必须记为未知，不能通过静默填零产生实机覆盖结论。

`build_region_certificate()` 必须对 22 个省略非相邻对逐对生成证据。对配对 `(Link i, Link j)`：

1. 在区域中心用独立 SAT 得到投影分离下界 `L`；
2. 在 Link i 的相对坐标中消去共同上游刚体运动，只累计 J(i+1)…Jj 对相对位姿的影响；
3. J1 平移使用区间半宽，转动关节使用全域有效的下游力臂和弦长上界；
4. 计算 `L - relative_motion_bound - geometry_error - numerical_tolerance`；
5. 仅当该值严格大于 `required_clearance_m` 时，该对标为 `certified`。

证书状态：

| 状态 | 语义 |
| --- | --- |
| `certified` | 22 对均满足声明净空的保守充分条件。 |
| `not_certified` | 已发现中心重叠，或当前连续下界不足。下界不足不等于已证明碰撞。 |
| `indeterminate` | 区域越限、数值不确定或输入无法形成有效证据。 |

证书必须包含模型 ID、配对策略 ID、证书 ID、完整区域、数值容差、22 对逐对证据和稳定原因码。确定性序列化的相同输入必须生成相同证书 ID 和 JSON 内容。

### 2.4 所需域覆盖评估

调用方不得直接把一张存在的证书当作本次动作已覆盖。必须通过：

```python
assess_domain_coverage(
    required,
    certificate,
    *,
    query_id,
    boundary,
    expected_model_id=None,
    expected_certificate_id=None,
    expected_pair_policy_id=None,
) -> DomainCoverageAssessment
```

返回状态：

| 状态 | 语义 |
| --- | --- |
| `covered` | 所需关节盒完整包含在证书盒内，所需净空得到满足，证书及运行身份全部一致。 |
| `outside` | 身份有效，但所需盒或所需净空明确超出证书能力。 |
| `indeterminate` | 证书缺失、被篡改、不是 `certified`、配对不完整、证据来源或任一身份不一致。 |

强绑定至少核对：query、boundary、model、pair policy、task、attachment、certificate、区域证据来源和几何误差允许量。证书内容必须重新计算身份，不能只信任传入的 `certificate_id` 字符串。

### 2.5 OFF-09 消费契约

OFF-09 只在以下条件全部成立时把 OFF-02 几何分门视为通过：

1. 点态结果为 `separated`；
2. 本次所需域结果为 `covered`；
3. 两份结果与当前候选的 query、测量边界、模型、任务、附件、配对和证书身份完整一致；
4. OFF-09 自己负责的求解器状态、原约束残差及其他门同时通过。

组合真值表：

| 点态 | 所需域 | OFF-09 几何分门 |
| --- | --- | --- |
| `separated` | `covered` | 身份一致时通过 |
| `overlap` | 任意 | 拒绝并由 OFF-09 锁存 |
| `indeterminate` | 任意 | 拒绝并由 OFF-09 锁存 |
| `separated` | `outside` | 拒绝并由任务层换路径或缩域后重新查询 |
| `separated` | `indeterminate` | 拒绝并补证据、修身份或修输入 |

现有 `TrackingStepData.overlap: bool | None` 只可作为诊断投影，不得作为该准入契约本身。

### 2.6 稳定原因码

实现和测试至少覆盖以下机器可判定原因码，不得依赖解析自由文本：

```text
GEOMETRY_OVERLAP
GEOMETRY_INPUT_INVALID
GEOMETRY_NUMERICAL_INDETERMINATE
PAIR_POLICY_INCOMPLETE
DOMAIN_OUTSIDE_CERTIFICATE
CERTIFICATE_MISSING
CERTIFICATE_IDENTITY_MISMATCH
REQUIRED_ENVELOPE_INCOMPLETE
```

同时存在多个问题时保留去重后的全部原因；OFF-09 负责决定锁存的首要停止原因。

## 3. 首轮任务范围与失败处理

首轮验证目标是当前完整蝴蝶任务、一个经离线筛选的固定九轴起点、起点到任务首点的过渡，以及完整跟踪。具体 q 由离线筛选产生，不直接采用零位，也不把随机 seed 当作已验证起点。

失败必须按原因处理：

| 证据 | 处理 |
| --- | --- |
| 具体 q 的 OBB 相交 | 改 IK 分支、起点或路径候选；细分不能消除该点的相交。 |
| 点分离但区域下界不足 | 先利用相对运动关系改善界，再按预算自适应细分。 |
| 只有可选活动范围导致失败 | 可缩小可选范围，但不得裁掉任务、误差、反应、停车或路径连通所需范围。 |
| 输入或物理参数缺失 | 补充依据；缺失期间报告 `indeterminate`/未知。 |
| 预算耗尽或搜索未找到 | 保留已证明局部区域、未覆盖单元和复现实验，不得声称无解或完整任务通过。 |

如完整任务未能覆盖，OFF-02 仍可交付有边界的局部证书或可复现的负结果，但必须把局部证明与完整任务覆盖分别报告。

## 4. 验收标准

### AC-1 配对策略完整

- 断言 `36 = 14 + 22`，两子集互斥且并集等于全部非相邻对。
- 9 个直接相邻对均不在自碰撞集合中。
- Link6 mesh 已加载；`Link3/Link5` 在省略的 22 对内且不属于豁免。
- 生成配置把 14 对标为在线子集，并显式记录相邻免检和配对计数。

### AC-2 点态反例与失败关闭

- 已知“14 对距离为正但其他非相邻 OBB 重叠”的构型必须返回 `overlap` 和 `GEOMETRY_OVERLAP`，至少保留 `base_link/Link9`、`Link6/Link8` 命中证据。
- 一个有效分离构型必须检查恰好 36 对后返回 `separated`。
- NaN/Inf、错误维数、越限、边界无效或模型失配必须返回 `indeterminate`，且不得伪造已检查配对。

### AC-3 至少一个有界局部连续结果

- 对 22 个省略对提供一个非空、有界区域的逐对连续证书；若候选不能认证，则提供相同输入可复现的 `not_certified` 或 `indeterminate` 结果。
- 证据明确区分中心点 SAT、相对运动界、几何误差、数值容差、最终净空下界与要求净空。
- 共同上游运动相消必须有回归测试，例如 `Link3/Link5` 不受 J1 单独变化影响，而 `base_link/Link9` 受 J1 影响。

### AC-4 身份与域检查

- 相同输入重复生成相同证书 ID 和序列化内容。
- 所需盒超界返回 `outside`；证书缺失、篡改、任务/附件/模型/配对/来源失配返回 `indeterminate`。
- `covered` 只在证书完整、身份一致、区域包含和净空要求全部满足时出现。

### AC-5 证据产物

- 离线生成器输出输入哈希、模型/配对/证书 ID、候选上下文、未知项、点态结果、区域和22对逐对证据。
- 实际加载的 OBB、运动学、mesh、任务与控制/轨迹输入按内容计算 SHA-256；工作树未提交修改不能被单一 Git HEAD 掩盖。
- 抽样检查与连续充分证明分栏报告；隔离性能基准与完整 100 Hz 回路 deadline 分栏报告。

### AC-6 测试边界

必须运行直接相关的纯 Python 测试：

```bash
python3 -m pytest -q \
  portable_oscbf/tests/test_obb_geometry_admission.py \
  portable_oscbf/tests/test_obb_model.py
```

测试覆盖配对分区、已知反例、非法输入、模型/证书身份、区域越界、相对运动界、连续证书、Link6 与生成配置语义。OFF-02 不要求启动 ROS、MuJoCo、CAN 或真实硬件。

## 5. 实现边界

规范实现入口：

| 路径 | 责任 |
| --- | --- |
| `portable_oscbf/work/obb_geometry_admission.py` | 纯逻辑点态评估、区域证书、域覆盖与身份检查；无 ROS I/O、无锁存状态。 |
| `portable_oscbf/scripts/certify_obb_region.py` | 从显式 JSON 输入生成确定性离线证据。 |
| `portable_oscbf/work/fcl_collision_mesh.py` | 离线 mesh 对照，包含 Link6，只排除直接相邻对。 |
| `portable_oscbf/scripts/generate_obb_calibration.py` | 生成 OBB 配置并准确标注在线子集、证书子集和排除策略。 |
| `portable_oscbf/tests/test_obb_geometry_admission.py` | OFF-02 行为与失效回归。 |
| `portable_oscbf/tests/test_obb_model.py` | 模型、mesh 和生成配置回归。 |

可微 DCOL 距离继续服务 CBF 内核；独立 SAT 服务准入事实。mesh/FCL 可解释 OBB 保守性，但不得静默替代本票选定的 OBB 准入判据。

## 6. 当前符合性证据

以下是 2026-09-15 本地证据快照，不是规格中的永久常量：

- 固定起点：`[0.2, -1.0, 1.2, 0.0, -0.7854, 0.35, 0.0, -0.47, 0.1]`。
- 蝴蝶任务首点 IK：`[0.145220506589, -1.125971556525, 1.298605201346, 0.0, -0.7854, 0.34987160919, 0.015356115966, -0.471678659574, 0.10801598214]`。
- 包含两点直线关节插值并逐轴外扩 0.0005 的局部盒，对 22 个省略对已认证；要求净空 1 mm，最小保守下界约 3.859286859 mm，瓶颈为 `Link5/Link7`。
- 模型 ID：`obb-model:v1:9e965e050aad0988a92974d62d952a55e74bdd2d05e5ec8e4f96272227ce20c2`。
- 配对策略 ID：`obb-pairs:v1:b0f9d288f60fc79239ce40f23d20368c1b2975104dc424a7fb2cfedd5d235f3c`。
- 证书 ID：`obb-region:v1:e7a2c4e62fa0f9e262d8a86e4ad1388231dd62afaaa96a1028ac5b2028caa763`。
- 相关测试结果：23 passed；隔离执行 36 对点查的 1000 次基准 p95 约 6.160 ms。

详细区域、逐对数据和输入哈希见：

- `docs/planning/oscbf-reuse/research/off02-local-certificate-20260915/region-input.json`
- `docs/planning/oscbf-reuse/research/off02-local-certificate-20260915/certificate.json`
- `docs/planning/oscbf-reuse/research/off02-local-certificate-20260915/REPORT.md`

## 7. 尚未覆盖与排除项

当前局部证书只负责固定起点至蝴蝶首点候选盒中的 22 个省略对。以下不属于已证明能力：

- 完整蝴蝶跟踪的连续九轴任务域；
- 生产 MoveIt 过渡是否始终位于当前局部盒内；
- 14 个在线对在没有相同运行约束的过渡阶段的连续覆盖；
- 实际状态误差、反馈延迟、反应和物理停车包络；
- 真实 OBB/mesh 制造、装配和标定误差；
- 未知或变化的附件几何；
- 完整控制回路 100 Hz 最坏时延；
- OFF-09 的求解后组合门、冻结锁存和人工恢复；
- OFF-13 的最终过滤命令复核与发送边界；
- sim/shadow/live 模式改变、CAN 通信和任何真实硬件动作。

`state_error_half_width_q=0`、`stop_half_width_q=0` 和 `geometry_error_m=0` 仅可用于明确标注的数字模型实验。它们不得解释为物理量已经测得，也不得据此开放 live。

## 8. 追溯来源

- `docs/planning/oscbf-reuse/handoffs/46-off02-geometry-decisions.md`
- `docs/planning/oscbf-reuse/research/off02-task-start-scope-20260915.md`
- `docs/planning/oscbf-reuse/research/off02-failure-routing/REPORT.md`
- `docs/planning/oscbf-reuse/research/off02-off09-interface-20260915.md`
- `docs/planning/oscbf-reuse/research/off02-local-certificate-20260915/REPORT.md`

以上来源中的早期“尚待实现”描述以本规格的当前证据和实际工作树为准；后期已接受决策优先于早期建议。
