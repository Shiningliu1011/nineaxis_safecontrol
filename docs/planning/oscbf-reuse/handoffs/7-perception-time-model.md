# [10] 感知时间同步与延迟模型（点云融合 → CBF 输入）

- Tracker：https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/7
- 日期：2026-09-09；已认领：Shiningliu1011。
- 阶段记录：[GitHub 进度评论](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/7#issuecomment-5595568265)，非 resolution，不追加地图已完成索引。
- 状态：本地审计与测量契约已整理；整票保持 OPEN，尚未得到最终 age_warn / age_stop 或完成诊断接线。
- 起点：main `f3cc336b13f4435580200f54dd00b83197dca3cf`。保留已有驱动交接修改及未跟踪标定、日志、电机资料。

## 范围与前置

用户原话：“继续下一个ticket”。沿用[官方驱动交接](31-official-sensor-drivers.md)指定的下一窗口和[推荐窗口顺序](../DECISION-WINDOW-ORDER.md)。官方驱动票仍是本票的原生阻塞；仓库顺序规则允许先推进不依赖缺失证据的研究，不撤销依赖、不伪称前置通过。GitHub 本票旧正文的 `Blocked by: None` 已过时，实际关系以原生依赖为准。

本轮只分析已有采样、执行隔离离线审计、核验上游和整理交接；没有启动设备、修改运行代码/参数或授权真机运动。本轮没有新增用户取舍；双源逐源有效性、覆盖失效停车锁存、覆盖恢复后人工确认均继承 ADR 0003/0006。

## 本地时间流转与复现

源码起点与上述 commit 对齐；下列相对链接均相对于本交接文件。隔离脚本和原始结果在 [audit_time.py](../../../../.scratch/oscbf-reuse-wayfinder/7/audit_time.py) / [audit-results.json](../../../../.scratch/oscbf-reuse-wayfinder/7/audit-results.json)。脚本只导入纯 Python 融合引擎和读取历史 JSON，不构建或发送 ROS 消息。

1. [perception_bridge.py](../../../../src/robot_safecontrol_moveit/perception_bridge.py) 的 `_sensor_callback` 将 header 的 sec/nanosec 转为 float64 秒；只保留 xyz，丢弃 LiDAR 逐点时间。空输入或过滤后为空直接返回，不记录此次消息到达与失败原因。外参变换按配置帧及 header 时刻请求；静态外参不能修正动态环境在扫描期间的运动。
2. `_fusion_callback` 在处理前读取 ROS now，交给 [FusionEngine](../../../../portable_oscbf/work/fusion_engine.py)。默认相机/LiDAR 最大年龄各 0.5 s，配对差上限 0.1 s，占据过期 0.3 s，perception_timeout 1.0 s；这些是当前配置事实，不是已验收的时间预算。
3. 引擎以最新 LiDAR 选最近相机，超过配对差上限丢弃较旧一路；`camera_age` 报告最新相机而非实际选中的相机。复现 now=100，camera=99.91/99.99，lidar=99.92：实际使用相机 99.91（90 ms），报告 camera_age=10 ms。fusion_stamp=99.92，fusion_age=80 ms，低估最旧贡献的90 ms。
4. 年龄只检查 `< max_age`，没有负年龄拒绝。复现 now=100，camera stamp=101：age=-1 s，camera_alive=1，perception_valid=1。不能将负值钳为0然后继续使用；应先判定时钟映射可信性。
5. 重复配对调用返回 `_empty_result`：两源仍 alive，但 perception_valid=0，fusion_stamp=0。应区分“没有新帧”和“旧观测已无效”，否则直接接健康门会产生错误失效或错误缓存语义。
6. `fusion_stamp` 通过 Float32MultiArray status 发布。对历史样本 epoch 秒，float32 相邻可表示值相差128 s；它不能承担毫秒级绝对时间。短 duration 可以使用浮点，但绝对时间必须保留 sec/nanosec 或 int64 ns。
7. `_publish_tracks(tracks, now_s)` 的 now_s 未编码进消息；8×10 slots 只有几何、速度、enabled、d_safe、alpha。[oscbf_controller.py](../../../../src/robot_safecontrol_moveit/oscbf_controller.py) 的 `_tracks_callback` 缓存这些字段，未保存采集或接收时刻，也未订阅 perception/status。融合无源后不发布 tracks，因此旧 `_obs_state` 不会因时间过期而主动失效。
8. 融合引擎在计算聚类 dt 之前更新 `_prev_fusion_stamps`，随后减去当前组合的最早 stamp：单源会得到0后取1/30 s，双源可能得到源间时间差，而非上一融合周期的时间差。这是速度增强可信性缺口，后续 tracks 契约实施需一并核验；不能将当前估计速度当作可信最大相对速度。

复现命令：`python3 .scratch/oscbf-reuse-wayfinder/7/audit_time.py`。本轮执行成功，退出码0；解释器、平台和相关代码 SHA256 见 [environment.json](../../../../.scratch/oscbf-reuse-wayfinder/7/environment.json)。这是隔离审计，不是产品修复验收。没有修改代码，未运行全量测试。

## 历史实测能支持什么

输入来自[官方驱动在线采样目录](../../../../.scratch/oscbf-reuse-wayfinder/31/online-2026-09-09/)，每份输入 SHA256 已写入审计 JSON。旧 probe 仅记录 `[time.time(), header_stamp, point_count]`，没有单调时钟、设备原始时间、曝光/扫描区间和驱动发布时刻。

| 采样 | 帧数 / 订阅端 Hz | callback wall − header 中位数 / 最大值 | header 相邻间隔最大值 |
|---|---:|---:|---:|
| 相机单源 | 313 / 26.263 | 43.738 / 46.730 ms | 333.453 ms |
| 相机并发 | 244 / 20.628 | 43.567 / 46.397 ms | 500.585 ms |
| LiDAR 单源 | 121 / 10.000 | 100.511 / 101.947 ms | 101.329 ms |
| LiDAR 并发 | 121 / 10.001 | 101.098 / 101.991 ms | 100.511 ms |

这些差值是时间域尚未独立校验的残差，不是已证明的采集至回调延迟。采样约12秒，不能给长期上界。相机间隔异常可能来自发布、注册/同步、调度或传输丢帧，旧 probe 无法定位。0.5 s 间隔也不能单独证明在线年龄门已经触发，因为缺少门执行日志。LiDAR约100 ms残差不能据此断定网络传输耗时100 ms。

## 上游复用结论

固定源码核验见[官方时间语义研究](../research/perception-time-model.md)。研究在隔离分支 `codex/research/perception-time-model` 提交 `fe81eaf`，未推送；文档已复制至当前主工作区，未提交/合入产品分支。

继续复用 Livox driver2 1.2.6 / SDK2 1.3.1（自有部分 MIT，保留第三方通知）与 Orbbec ROS2 v2.9.3（wrapper Apache-2.0，SDK另核条款），许可证证据见[驱动研究](../research/official-sensor-drivers.md#license与验收边界)。保留现有融合管线作为对照；必要改动为时间/provenance/诊断薄适配，不重写驱动、配对或通用时钟设施。message_filters只是配对工具，本票没有证据要求替换现有配对算法。

Livox未同步时用主机 `high_resolution_clock` 处理包时间替代设备时间；该时钟的epoch与count单位须核本机构建。PointCloud2逐点timestamp是FLOAT64绝对ns，header取批首点，不能误读为逐点相对秒。Orbbec注册彩色点云取depth_frame时间；global是映射到主机域的采集时间，仍须实测误差及具体时钟选择。这些差异由适配记录并判有效性，无法证明采集年龄的源不得仅凭“时间接近当前”取得准入。

## 建议时间契约（后续实施输入）

本节是基于既有逐源/覆盖决议的工程细化建议；尚未实现或完成设备验收。

每路保留 source_id、设备身份、boot/session epoch、frame sequence、原始设备时间及单位、header时间和其语义、采集区间 start/end、时钟域、映射版本与误差上界、回调 ROS时间与单调时间、frame_id、标定ID和明确无效原因。没有可靠 frame sequence 时记录其缺失，不能用订阅计数冒充设备计数。相机注册点云应记录真正几何来源（深度）与配对彩色帧，不能由彩色帧刷新深度年龄。LiDAR 保留逐点时间或可信扫描区间；不能把整帧当作瞬时采样。

处理时刻另记录：预处理始末、选中各源帧、融合始末、观测发布、控制接收、QP开始/结束、最终命令校验与实际执行反馈。源时间不因重发、融合或预测而刷新；预测估计时刻与最后测量时刻分开。耗时/无新消息超时用单调时钟；ROS /clock 暂停或回跳时不能让真机看门狗停止计时。仿真时间和真实运行时间不能混用。

记映射后的最早有效采集时刻为 t_start,i，时钟与采集语义误差上界为 epsilon_t,i。在控制消费时刻，同一可信时间域内的保守年龄为：

`A_i = max(0, t_control - t_start,i) + epsilon_t,i`

此公式只用于已验证的时钟映射；未来时间超出映射误差界、零/非有限时间、会话变化或映射失锁先置无效，不通过 max(0, ...) 掩盖错误。只有 header 而无扫描区间时，须先证实 header 意义并补扫描/曝光宽度界，不能假设其为采集起点。

使用每项必要覆盖和几何实际贡献源的年龄；诊断 `max_obstacle_age_ms` 可汇总最大值，但不能代替逐源字段。融合内部仍保留来源及最旧有效贡献。跨源配对差只约束时间错位，不证明同步，也不独立决定覆盖是否足够。

用于推导的空间预算写为：

`B_i >= d0 + epsilon_geom,i + v_rel_bound,i * (A_i + T_remaining_bound) + D_stop_bound`

其中 B_i 是该动作及必要覆盖下可用的保守间距预算；epsilon_geom 包括以米表示的标定、深度/体素/几何/跟踪误差界；T_remaining 是消费后至停止反应起始的控制/命令时延界；D_stop 是此后机器人制动位移与障碍接近位移的保守合计。同一部分不可重复计入时间项与制动项。已对未来位置膨胀/预测的几何需声明基准，避免重复补偿。

当 v_rel_bound>0 且适用域明确，可推导 `A_stop <= (B_i-d0-epsilon_geom,i-D_stop_bound)/v_rel_bound,i - T_remaining_bound`。非正余量表示该工况不能准入；速度为0的退化情形单独评估，速度未知不等于0。有加速度界时改用相应可达位移界。此式只是预算核算框架，不是 CBF 安全性证明或完整人机协作标准验收。

`A_warn < A_stop`，两者的差必须容纳检测与已验证降进给响应；测量分位数用于性能比较，不能冒充故障时间上界。尚缺速度工况、几何误差、剩余控制时延及实际停止证据，故本票不提供虚构最终秒数，0.5/1.0 s 不升级为安全合同。超过使用域或必要覆盖失效且无有效保守替代时停止锁存，不删除过期约束后继续运行。

## 测量和编码交接

可以开始编码的范围：独立只读记录器、上述时间字段的离线校验/报告、合成时间故障回放。需要用户后续实施指令才进入产品代码；本轮没有新增运行程序。

正式诊断及控制接线尚不能按旧8×10 slots直接实现并宣称完成。应由本票提供字段/计时要求，与[观测契约升级](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/21)落实消息、[感知健康接线](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/23)落实有效性与锁存、[控制率与延迟预算](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/17)及[最终命令与执行保护](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/33)补消费后时间和执行证据。不要反向增加循环阻塞。

验收至少覆盖：两路独立/同时采样；预热与稳定阶段；CPU/GPU及录包组合负载；注册点云发布间隔异常定位；原始与映射时间同时记录；时钟偏移/漂移/跳变、设备重启、乱序、重复、空帧、停流；证明新源不能刷新旧源年龄，发布/处理不能刷新采集时间，控制端断流仍会自行过期。离线故障注入不得修改主机系统时钟或访问命令流。随后再在明确设备操作范围内补长期实测，保留样本量/时长/负载及分位数，不把一次通过当全工况上界。

回退：保留当前源码、配置及驱动版本快照；诊断先独立部署，停止诊断即可撤回。未来消息迁移须显式区分新旧版本并恢复匹配的生产者/消费者，禁止用旧无时间字段消息绕过准入。

下一推荐研究窗口：[自体过滤 bridge 接线：逐源 MoveIt 几何过滤](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/22)。其前置未闭合时仅推进独立研究；本窗口不自动启动下一票。

## 续研窗口：2026-09-11

阶段记录已发布：[本轮 GitHub 进度评论](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/7#issuecomment-5627868092)，非 resolution。

用户原始指令：“开始研究ticket#7”。继续本票研究，不启动下一票。当前起点为 `84797408ac0cbe344bcb9f06fc70da5ae29dd417`，开始时工作树干净；已通过 tracker 再次认领。正式地图要求重要的新安全阈值展示方案后由用户确认；本次没有作出新的数值决策。

### 当前证据复核

从上一审计起点到当前 HEAD，fusion_engine、perception_bridge、oscbf_controller 三文件无差异。重新运行原审计，未来时间戳放行、选中帧年龄低报、重复帧返回无效空结果及历史采样统计均复现。旧结果保留，新结果写入 [2026-09-11/audit-results.json](../../../../.scratch/oscbf-reuse-wayfinder/7/2026-09-11/audit-results.json)。未新增现场采样。

新增 [audit_incremental.py](../../../../.scratch/oscbf-reuse-wayfinder/7/2026-09-11/audit_incremental.py) 在独立进程中调用实际 FusionEngine，仅用 mock 包装聚类入口记录实参；[incremental-results.json](../../../../.scratch/oscbf-reuse-wayfinder/7/2026-09-11/incremental-results.json) 保存 Python、平台、HEAD 和四个源码 SHA256：

| 离线输入 | 观察结果 | 能支持的结论 |
|---|---|---|
| 单 LiDAR 两帧间隔100 ms | 两次传入聚类的 dt 均33.333 ms | 更新历史顺序使 dt 失真；若同一质心位移成功关联，速度除数会带来3倍比例误差 |
| 双源各100 ms周期、源间差10 ms | 两次 dt 仍33.333 ms | 不能把源间配对差用作跟踪周期；本项验证实参，不宣称真实目标速度误差已经实测 |
| camera stamp=+inf | alive门仍为1 | `< max_age` 缺少有限性拒绝；该项将降采样结果替换为空，只验证alive门，不声称完整链接受无穷值 |
| 启动 now=0.01 s、stamp=0 | perception_valid=1 | 未初始化时间在该合成启动场景可通过；真实模式应有零值策略，仿真合法原点另按模式处理 |
| now=100、stamp=99.5、max_age=0.5 | alive=0 | 现值使用严格小于，后续阈值边界必须明确，不能混用正文中的≤ |

[dynamic_clustering.py](../../../../portable_oscbf/work/dynamic_clustering.py) 的速度是关联质心差除以 dt；新 track 的零速度来自初始化，函数没有独立的物理 v_max 合同。关联距离门也不是障碍运动上界：快速物体可能无法关联而取得新的零速 track。故速度未知不能由新建槽的零值降格成静止；修复 dt 也不会自动建立物理速度界。

两个脚本执行均退出0。它们是隔离审计，不是产品测试通过声明；未修改产品程序/配置、未启动 ROS 节点或设备。原始输出目录为本地 scratch，未来共享时需显式纳入证据归档，不能假定远端能打开。

### 测量字段与验收判据细化（建议，尚未接线）

建议独立记录器按事件写入带 schema_version 的 JSONL，绝对时间使用整数 ns；每行带 source_id、设备 session、观测 ID、事件名和 clock_domain。采集来源与映射区间继承上文合同。计数器仅有订阅端序号时标注 subscriber_sequence，不能冒充设备帧号。关联失败或无序号也必须保留事实。

| 诊断 | 定义与缺失行为 |
|---|---|
| selected_source_age_upper_ns | 控制消费时，各实际贡献源最早采集时间的保守年龄；需要可信时间映射，未知为 null + reason |
| max_obstacle_age_ms | 实际障碍几何贡献的年龄上界最大值；空集合或任一必要贡献未知时不能以0代表安全，配套 availability/reason |
| max_required_coverage_age_ms | 必要空间覆盖证据的最大年龄；与有无障碍槽分开，即使零障碍也不能省略覆盖检查 |
| receipt_to_consume_ns | 同一主机、同一单调时钟域的消费减接收；它是链内延迟，不冒充采集年龄 |
| selected_pair_delta_ns | 实际选中两源在已校准公共域的采集参考时刻差；另报映射误差与采集区间宽度 |
| last_measurement_id / estimate_time_ns | 缓存、重发、预测都保留最后真实测量身份；估计时刻另列，不刷新测量年龄 |
| invalid_reason / required_coverage_valid | 区分无新帧、未初始化、时钟未知/跳变、乱序、重复、空输入、超龄、必要覆盖缺失；观测失效与锁存结果各自记录 |

不能只在回调到达时计算年龄；无新消息时，消费端或独立监督仍须按单调时钟推进接收超时与已锚定年龄。在可信映射成立后，可将该次接收时的年龄上界作为锚点，以单调经过时间加经过时间误差界继续累加；时钟映射/session改变则旧锚点失效，不能静默重基准。跨进程/跨主机记录须显式证明时钟可比性。

离线验收应覆盖：

1. 同一旧测量重发、较新另一源到达、预测更新、零障碍槽：均不能延长旧必要覆盖期限。
2. 未来越界/NaN/inf/时间未初始化、设备重启、ROS时间回跳或暂停：返回明确原因；硬件超时不随ROS时间暂停。
3. 等于阈值及其前后1 ns：边界一致；任何停止请求之后的锁存与人工恢复遵守已接受规则。
4. 停流后无回调：消费侧仍可到期；如果没有仍有效且已证明充分的替代覆盖，则进入停止锁存，不能仅清掉槽。
5. 非零帧间隔、多种帧率和非均匀间隔：聚类实际 dt 对应所比较的测量，关联失败速度为未知语义；记录器不得把速度估计当最大速度先验。
6. 数据缺字段时如实报 unknown，报告不能产出“年龄0/通过”；长期测量保留原始样本与统计分母，统计最大值与合同上界分列。

### 阈值推导和下一步边界

补充的一手依据与公式适用范围见[陈旧度预算研究](../research/perception-age-budget.md)。之前的相对速度预算只是统一的保守核算形式；计算具体裕度前必须先固定距离参考时刻，不能同时用当前机器人几何、完整历史相对位移和同一段机器人运动再收费。扫描/曝光误差、测量误差、帧间运动、控制保持及停止位移也分别记账。几何膨胀本身不等于已经证明离散执行的 CBF 安全性。

当前仍不能给 age_warn/age_stop 最终数值：缺逐源时钟误差、工况障碍运动界、几何误差、消费后延迟与经验证的停止轨迹。已有0.5/1.0 s参数继续仅为实现现值。当前新 track 零速度不能填补该缺口；“先取1秒，后面再调”不构成可审阅推导。

**可以开始编码：**独立只读事件记录器、离线契约校验和故障回放，输入未知时忠实输出未知。**尚不能直接完成的范围：**产品 max_obstacle_age_ms、观测消息迁移、健康锁存接线和最终阈值配置；需按已有原票协调消息与保护合同，并在实施指令后进入产品接线。实测也必须记录具体设备操作范围。

整票继续 OPEN；“双传感器官方驱动接入与版本冻结”原生前置保持，未加入反向循环依赖。新发现均属于本票及既有观测/健康/执行保护票，不另建同义票；没有已完成 resolution，不更新地图 Decisions so far。本窗口材料尚未提交或推送。
