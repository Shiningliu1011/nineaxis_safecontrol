# OFF-02 → OFF-09 几何接口决策研究

日期：2026-09-15。对应 [OFF-02 / #46](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/46) 与 [OFF-09 / #47](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/47)。研究问题：一个“当前构型重叠状态＋任务区域证据状态”的双轴接口，是否足以支持 OFF-09 的求解后准入和冻结锁存？

状态：研究完成；用户已接受修订建议，尚未实现产品代码。

## 结论

**双轴结构正确，但原提案需要收紧。** 不能只传两个裸枚举。推荐接口由两个互不替代的评估组成：

1. **点碰撞评估**：对明确绑定的关节状态判断全部非相邻 OBB 对是 `separated / overlap / indeterminate`。
2. **任务域证书评估**：判断调用方声明的动作与停车包络是否为 `covered / outside / indeterminate`。

两者都必须绑定查询对象、测量边界、配对策略和不可变证据身份。OFF-09 仅在 `separated + covered` 且绑定关系完整一致时把几何分门视为通过；其他组合均拒绝并按既定状态机锁存。拒绝动作可以相同，原因不能被压成同一个布尔值。

## 为什么必须分成两个评估

[OMPL 官方文档](https://ompl.kavrakilab.org/stateValidation.html)明确区分 `StateValidityChecker`（单状态）与 `MotionValidator`（状态之间的运动），并指出默认离散运动检查可能漏掉采样间的无效状态。一个点的“未重叠”因此不能推导路径段、误差邻域或停车可达集都安全。

[MoveIt Humble PlanningScene 文档](https://moveit.picknik.ai/humble/doc/examples/planning_scene/planning_scene_tutorial.html)同样把具体构型的碰撞结果、接触对、关节界限、运动学约束和最终 `isStateValid` 分开。其 [CollisionResult API](https://moveit.picknik.ai/humble/api/html/structcollision__detection_1_1CollisionResult.html)保存碰撞布尔值、接触对和距离；这支持“几何事实带明细，由更高层组合准入条件”的责任划分。MoveIt 的网格结果不能替代本票已选定的 OBB 判据，只用于说明接口分层。

最新 GitHub #47 工单（本轮回读，无评论）要求 OFF-09 区分 solver、弹性松弛、原约束残差、几何重叠和非有限输入，并将拒绝接到锁存。它没有授权 OFF-02 自己决定总准入或恢复。

## 推荐的语义契约

以下是语义，不提前钉死 Python 类名或 ROS 消息名。

### 请求必须说明什么

| 字段语义 | 要求 |
|---|---|
| 查询身份 | 单调序号或不可重用的 `query_id`，供返回值与本次候选严格配对 |
| 测量边界 | 至少区分 `measured_state / kernel_candidate / filtered_command`；名称可复用 OFF-15 已有边界词汇 |
| 点状态 | 九轴 q、关节顺序和其来源时刻/序号；非有限、维数或顺序错误不得进入计算 |
| 所需域 | 本次动作段及要求覆盖的误差/反应/停车可达集；若调用方只能提供一个点，域证书不能返回 `covered` |
| 配对策略 | 明确的非相邻配对集合身份；直接相邻 9 对按用户决定免检 |
| 运行身份 | 模型、OBB、运动学、附件声明、任务及证书身份 |

OFF-09 的求解后门应查询 `kernel_candidate`；OFF-13 若要对滤波后的最终命令复核，应以 `filtered_command` 发起另一份查询。两者不能共享一个没有边界标识的结果。实际调用次数和进程内/ROS 传输方式留给实现阶段按时延测试决定。

### 返回必须保留什么

#### A. 点碰撞评估

- `separated`：对本次明确 q 和配对策略，独立 OBB 判据确认全部必检非相邻对分离。
- `overlap`：至少一对相交，携带命中的连杆对及判据明细。
- `indeterminate`：输入非法、身份不符、数值边界、计算异常或配对覆盖不完整。不得降级成 `separated`。

建议点检查覆盖 **36 个非相邻对**，而不是只查当前 14 个 QP 约束对。原因是既有点实验在省略的 `Link3/Link5` 上发现 OBB 重叠；独立 SAT 点检查不等于把该对增加为 QP 约束。实现必须测量 100 Hz 路径的实际成本；若不能全查，返回值须带完整的被查集合身份，并由证书覆盖剩余对，OFF-09 必须验证两者并集无缺口，不能静默少查。

#### B. 任务域证书评估

- `covered`：本次所需动作/停车包络是证书已证明区域的子集，证书身份与当前运行身份一致，并覆盖其声明负责的全部省略非相邻对。
- `outside`：身份有效，但所需集合有部分明确落在证书范围外。
- `indeterminate`：证书缺失、身份不符、所需集合描述不完整、数值检查失败或证据不足。

`outside` 与 `indeterminate` 对准入都是否决，但前者通常需要换路径或缩域，后者通常需要补证据、修身份或修输入；保留区别能防止错误恢复。

### 稳定原因码

状态枚举回答“结果属于哪一类”，原因码回答“为什么”。首版至少需覆盖：

- `GEOMETRY_OVERLAP`
- `GEOMETRY_INPUT_INVALID`
- `GEOMETRY_NUMERICAL_INDETERMINATE`
- `PAIR_POLICY_INCOMPLETE`
- `DOMAIN_OUTSIDE_CERTIFICATE`
- `CERTIFICATE_MISSING`
- `CERTIFICATE_IDENTITY_MISMATCH`
- `REQUIRED_ENVELOPE_INCOMPLETE`

命名可以在实现时调整，但测试和诊断应断言稳定码，而不是解析自由文本。遇到多项问题时保留全部原因，同时给 OFF-09 一个确定的首要锁存原因；这与 `CommandSafetyGate` 保留首个停止原因的现有行为一致。

## 身份应绑定哪些内容

至少包括：

- schema/算法版本和数值策略；
- URDF/运动学链、mesh 与生成的 OBB 数据身份；
- 非相邻配对策略及省略对证书覆盖清单；
- 附件声明，必须区分“明确无附件”和“附件未知”；
- 蝴蝶任务、共享轨迹变换、允许起点/区域及单位；
- 证书 ID、生成输入及适用误差/停车参数状态。

项目已有可复用模式：`calibration_record.py` 对实际加载字节与规范化内容分别计算身份，并在身份不一致时不可准入；`tracking_contract.py` 将证据来源和测量边界作为独立字段；ADR-0009 使用不可变启动快照、固定错误码和未评估状态。这些模式支持复用设计原则，不表示几何证书已经实现。

## 组合真值表

| 点碰撞 | 任务域 | OFF-09 几何分门 | 典型原因 |
|---|---|---|---|
| separated | covered | 通过 | 查询、配对和身份均完整匹配 |
| overlap | 任意 | 拒绝并锁存 | 已发现具体非相邻 OBB 相交 |
| indeterminate | 任意 | 拒绝并锁存 | 输入、数值、身份或配对检查不完整 |
| separated | outside | 拒绝并锁存 | 当前点分离，但动作或停车包络超出证书 |
| separated | indeterminate | 拒绝并锁存 | 当前点分离，但没有足够区域证据 |

“当前点分离、域外”不能自动规划穿越域外进入证书；是否换路径由任务层重新提出候选，再做新查询。

## 传输与诊断建议

首版优先实现为纯逻辑、不可变的同步请求/结果对象，供 OFF-09 在同一候选上调用和测试。若以后跨进程，再定义版本化消息；此研究不提前选择服务、topic 或共享内存。

`TrackingStepData.overlap: bool | None` 可继续作为报告投影，映射为 `overlap / clear / unmeasured`，但不能作为控制门输入：它不表达查询边界、配对完整性、域外或证书身份。ROS 2 Humble 的 `DiagnosticStatus` 定义提供 `OK/WARN/ERROR/STALE` 及键值明细，适合监控投影；其通用级别也不应替代逐 tick 的确定性几何结果。项目中诊断已有 1 Hz 身份心跳模式，不能据此假设它与 100 Hz 候选逐条对应。

## 边界与仍待实现验证

- 本研究没有决定消息字段编码、缓存、进程边界或性能优化。
- 36 对 SAT 在本项目 100 Hz 控制路径上的最坏时延尚未实测；不能凭算法规模宣称满足 deadline。
- OFF-02 的连续区域证书尚未产生；物理误差、反应和停车界仍未知时，正式域证书不能声称覆盖实机停车集合。
- OFF-09 负责组合 solver/残差/几何并锁存，OFF-13 负责最终滤波/发送边界，不能用 OFF-02 的点检查宣称最终命令或物理停止安全。
- 本轮只读回查 GitHub #47、项目源码和官方资料；未启动 ROS、仿真或硬件，也未修改产品代码。

## 已接受的决策文本

用户接受“双评估＋强绑定”接口：点碰撞评估与任务域证书评估分开；每份结果绑定 query、测量边界、配对策略和不可变证据身份。OFF-09 仅接受 `separated + covered` 且绑定完整一致的结果，其余状态拒绝锁存。点检查原则上覆盖全部 36 个非相邻对，相邻 9 对免检；若实现不能全查，必须证明运行检查与证书覆盖的并集无缺口。具体类型、传输和性能优化由实现阶段验证。
