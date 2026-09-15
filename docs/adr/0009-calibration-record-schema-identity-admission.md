# 标定记录的 schema、身份与准入接口

2026-09-14 定案（OFF-01 对齐决策；承 ADR 0004，来源票据 #45 / #27）：
ADR 0004 定了「谁是标定真源」（`config/sensor_extrinsics.yaml` 唯一权威），但没有规定
**记录长什么样、身份怎么算、未标定记录能不能进入避障**。本 ADR 定这三件事，它们是
T5（标定工具写入）、T7/OFF-01（运行时加载与诊断）、T6/OFF-17（自检比对）之间的契约面。
本文记录的是**决定**，不是实施或验收结果。

## 决定 1 — 记录为逐源分节

`config/sensor_extrinsics.yaml` 顶层带 `schema_version: 1`，每个传感器一个分节
（`camera` / `lidar`），分节内含 `frame_from`、`frame_to`、`matrix`（行主序 16 floats，
与既有 YAML 及 ROS 参数数组表示一致）与一条 provenance 子块（`calibrated`、
`sensor_serial`、`calibration_id`、`method`、`timestamp`、`operator`、`residual`、
`error_estimate`）。与标定无关的 `orbbec_launch` 块原位保留；旧键 `parent_frame`、
`child_frame`、`child_frame_lidar`、`calibration_method*` 随迁移删除，避免两套命名并存；
未实测字段写 `null`，不编造序列号/残差/日期（ADR 0004、交接文档 `27-calibration-ssot.md`）。

理由：上述字段绝大多数**逐源**而设，扁平命名加 `_lidar` 后缀会随源数量线性膨胀，
且容易只改一半（给 camera 加了字段忘了 lidar）。迁移时把 `perception_runtime.yaml:40`
那份假定相机位姿搬入本文件的 camera 分节，标 `calibrated: false`、`method: assumed`、
`operator: assignment`——这是**纠正**当前文件里与"未标定"事实矛盾的 `manual_measurement`
标注，不是能力降级（ADR 0004 的向后兼容硬约束：单 Camera 行为不得变化）。

## 决定 2 — 两种身份分开

- **文件身份** `file_sha256` 覆盖**实际加载的原始字节**（一次 `read_bytes()` 后同一份
  快照用于 hash、解析与校验），证明"节点读到的就是被检查的那份文件"。
- **记录身份** `calibration_id = "v1:" + sha256(canonical)[:16]`，其中 canonical 是对
  **显式白名单字段**（`schema_version`、`source`、`frame_from`、`frame_to`、`matrix`、
  `calibrated`、`method`、`operator`、`timestamp`、`sensor_serial`、`residual`、
  `error_estimate`）做 UTF-8 + `sort_keys` + 最短往返浮点 + 拒绝 NaN/Inf/重复 key 的
  规范化结果，**排除 `calibration_id` 自身**以避免循环。标定工具与运行时加载器必须
  **共用同一个规范化函数**，并留有固定测试向量（已知输入 → 期望 id）。
  `calibrated` 纳入 id：否则把 `false` 改成 `true` 不改变身份，一次"状态升级"可以沿着
  未变的 id 溜过 T6 比对。

理由：两个身份回答两个不同问题——文件 hash 证明"这份字节"，记录 id 证明"这份标定内容
没变"（即使注释、`orbbec_launch` 块或另一个源的字段改动，也不应让本源的标定身份变化）。
二者不可互相替代：只用文件 hash 会让任何无关改动都改变"身份"，且无法单独标识单源记录。

## 决定 3 — 准入与逐源豁免

加载器对每个源给出 `math_valid / provenance_valid / calibrated / identity_consistent`，
其中

    admission_ready = math_valid ∧ calibrated ∧ provenance 完整 ∧ 身份自洽

（实施期补充了第四个因子 `identity_consistent`：ADR 本文原写三者。理由是「声明的
`calibration_id` 与实际内容算出的不一致」意味着有人手改了矩阵却没更新身份——这正是
T6 比对要防的情形，而它既不是数学问题也不是标定状态问题。它的处理是**启动但不可准入**
+ WARN，不是拒绝启动：标定工具（OFF-16）若哪天改了规范化实现，不该让节点直接 brick。）

**provenance 完整的判据**（OFF-16 必须满足，否则 T6/准入必然失败）：

| 字段 | `calibrated: false` | `calibrated: true` |
|---|---|---|
| `calibrated` | 必填、必须是显式 bool | 同左 |
| `method` / `operator` | 必填、非空字符串 | 同左 |
| `timestamp` / `sensor_serial` | 可 null | 必填、非空字符串 |
| `residual` / `error_estimate` | 可 null | 至少一项非 null |

**逐源标定门**：任何**被启用**的源都必须通过标定门才能启动；唯一例外来自**部署 profile
的显式逐源声明**（如 `allow_uncalibrated_debug: [camera]`，默认不设）。
`perception_dual_sensor_real.yaml`（部署 profile）不设该声明，因此启用 LiDAR 前必须
先有已标定的记录。豁免**只影响能否启动，不改变准入结论**：被豁免的源仍然
`admission_ready = false`，诊断必须把它的身份标为未标定调试几何。

理由：豁免表达"本次部署能容忍什么"（部署属性），`calibrated` 表达"这个外参是什么"
（标定属性），两者不可混写；把豁免放进 profile 而不是代码，也让将来 T7 的模式接口
有现成落点，不必现在发明运行模式开关。**模式级拒绝仍不在本 ADR 范围**（见 OFF-17）。

## 决定 4 — 启动期失败分级与「未评估」

| 情形 | 行为 |
|---|---|
| 文件缺失 / 不可读 / YAML 解析失败 / 字节快照失败 | **拒绝启动** |
| 矩阵数学非法（元素非有限、`RᵀR≈I`、`det≈+1`（不取绝对值）、底行 `[0,0,0,1]`、平移超包络） | **拒绝启动** |
| 记录声明的帧与运行时配置的帧不一致（`frame_from` / `frame_to` 对不上） | **拒绝启动** |
| 启动命令仍传旧外参参数（`use_tf` / `camera_to_world_static` / `lidar_to_world_static` 的值 ≠ 内置默认） | **拒绝启动** |
| `calibrated: false` 且该源被部署 profile 豁免 | 启动 + WARN + 不可准入 |
| 记录身份不自洽（声明的 `calibration_id` ≠ 内容算出的） | 启动 + WARN + 不可准入 |

**不再保留 identity 兜底**：这是相对当前 `perception_bridge.py:326` 的有意收紧——原本
文件缺失时会静默按单位阵继续跑，之后会直接拒绝启动（ADR 0003:38「任一失败 → 拒绝进入
避障模式，不静默回退 identity」）。数学容差首版取正交残差 / det 偏差 / 底行 `≤ 1e-6`、
平移 `±10 m` 机械包络；这些是**软件合法性容差，不是标定精度**。

ADR 0003 里需要现场数据的两项（与 provenance 记录比对 `≤20mm/5°`、已知尺寸物体点云
误差 `<20mm` 验收）**本 ADR 不实现、不用占位数据充数**：诊断以固定错误码
（`CALIB_ACCEPTANCE_NOT_EVALUATED`）显式标为"未评估"，实测验收归 T5/T6 与 H6.1
（`docs/real_robot_execution_plan.md:177`；`OFFLINE-EXECUTION-ORDER.md:73,85` 写作
T6/H0，H0 实为仓库欠账清理与仿真基线，此处以 H6.1 为准）。

## Considered Options

- **schema 三选一**：扁平加后缀 / **逐源分节（选定）** / 扁平键把矩阵收二级。后者的问题是
  顶层既有 `camera` 块又有 `orbbec_launch` 块、语义不齐，且没有 `schema_version` 可比对。
- **身份三选一**：**规范化内容 hash（选定）** / 文件 hash 直接当 id / 不引入 id 只逐字段比。
  后两者让 T6 缺少单一比对键，或让无关改动改变"身份"。
- **豁免三选一**：**profile 逐源声明（选定）** / 代码里写死 camera / 按运行模式（sim 允许、
  shadow·live 不允许）。按运行模式会与 OFF-17「启动自检离线汇总与模式拒绝验证」重叠。

## Consequences

- **OFF-16**（标定工具）必须写入逐源分节并复用共享规范化函数，否则 T6 比对必然失败。
- **T6/OFF-17** 以节点上报的 `lookup_path` + `resolved_path` + `file_sha256` + 逐源
  `calibration_id` 为准，不另读源码树副本（否则 source/install 分叉时必然误报通过）。
- `perception_runtime.yaml` 与 `perception_dual_sensor_real.yaml` 的矩阵键随迁移删除；
  `package.xml` 需补 `diagnostic_msgs` 依赖（现未声明）。
- 记录不再热重载：启动期一次读取，运行期使用不可变快照。

## 实施契约（OFF-01 落地，2026-09-14）

以下是把上面四条决定变成代码时定下的接口细节。**全部条款都经用户逐条确认**：四条决定在
grilling 中确认；Q5（诊断发布机制）、Q7（旧参数与 TF 处置）、Q9（部署 profile 语义）在
实现完成后按同一格式重新提问、由用户在 2026-09-14 明确选定，三者都与当时已落地的实现一致，
因此没有产生代码改动。

**加载器**：`src/robot_safecontrol_moveit/calibration_record.py`，纯 Python、不 import rclpy，
路径由调用方注入；对外 API 为 `load_calibration_record(...) -> CalibrationRecord`
（含 `lookup_path / resolved_path / file_sha256 / schema_version / sources{...} / errors`）、
`decide_startup(...) -> StartupDecision`、`calibration_status_entries(...)`。
顶层键白名单 = `schema_version / camera / lidar / orbbec_launch`，多一个未声明块即报错
（防 `Camera:` 这类拼写错误被当成「该源不存在」）。

**启动失败语义**：任一拒绝条件命中即抛 `CalibrationStartupRefused`，节点在创建订阅与定时器
**之前**退出，进程退出码 `3`，stderr 打印 `PERCEPTION_BRIDGE_CALIBRATION_REFUSED` 与该次
全部错误码。被拒绝的实例**不发布**任何诊断——「当前实例没有报告」正是消费端可判定的信号。

**诊断接口（Q5，用户已确认）**：`/perception/calibration_status`，
`diagnostic_msgs/DiagnosticArray`，**volatile QoS**（不用 latched：latched 会把一个可能已退出
实例的身份交给晚加入者，「收到」就不再蕴含「刚发出」），启动时立即发一份 + **1 Hz 心跳**；
条目为 `perception_bridge:record`（lookup/resolved 路径、file_sha256、schema_version、
start、errors、warnings、acceptance）+ 每源 `perception_bridge:<source>`（enabled、exempt、
calibrated、math_valid、provenance_valid、identity_consistent、admission_ready、
calibration_id、declared_calibration_id、frame_from/to、method/operator/serial/timestamp、
issues、time_domain）。level 映射 `OK=0 / WARN=1 / ERROR=2`。`hardware_id` 与
`node_identity` 值都是 `<节点名>#<pid>`，供消费端判断「这份报告属于当前实例」。
**新鲜度由消费端用本机单调时钟判断**（ROS 时间会暂停/回跳/归零，消息时间戳不能当新鲜度依据）；
判据取「超过 2 个心跳周期收不到 = 报告陈旧、实例可能已不在」。记录在运行期不可变
（无热重载），所以「状态变化立即补发」在当前契约下退化为启动时那一次。

**逐源时间域**：只上报部署 profile **显式声明**的值（`source_time_domain_camera/lidar`，
默认空 = `unknown`，`time_domain_source` 标 `declared_by_profile`/`unknown`），
**不照抄驱动默认值**——同样的参数默认值在不同机型/固件上语义不同
（`research/sensor-time-alignment-20260914.md`）。OFF-01 只上报，判定语义归 OFF-06。

**旧参数与 TF 分支的最终处置（Q7，用户已确认）**：采用「删除语义 + 保留声明作探测器」。
三个旧名字仍被 `declare_parameter`（默认 identity / `false`），值不等于默认即拒绝启动；
TF 输入路径、`_tf_to_matrix` 与 identity 兜底全部删除，**没有任何回退矩阵**。依据有三条
独立证据：本机原型实测（旧参数会被静默忽略，矩阵不一致无法从值本身看出）、
T7 交接口径（`handoffs/27-calibration-ssot.md:74`「废除旧矩阵参数影响并明确报错旧覆盖配置，
拒绝 use_tf 绕过」）、外参真源调研（ROS 2 无「从外部 TF 反读配置」的官方模式，REP-105 要求
每段变换只有一个权威；Orbbec/Livox 驱动都不发布 `base_link` 变换）。被否的两个方案的代价
记录在案：纯删除会让旧参数**静默失效**；保留 TF 可选路径今天不提供任何能力，却会在有人
广播 `base_link → 传感器` 时变成一条独立于记录的外参来源，且静态 TF 被 latch 后不一致
不可观测。

**运行期帧绑定**：每帧校验 `msg.header.frame_id == 配置的源帧`，不符即丢弃并按秒限流告警
（不换用别的矩阵、不按 TF 补），符合「先校验 header.frame_id 再消费点云」的 T7 口径。

**部署 profile 语义（Q9，用户已确认）**：`perception_dual_sensor_real.yaml` 作为部署声明
**不设任何逐源豁免**，因此它现在**加载即拒绝启动**（它启用相机，而相机记录仍是
`calibrated: false`）。原语义「LiDAR 关闭 = 等价单相机」改为「真机部署不得用未标定几何
起步」；仿真/算法调试走 `perception_runtime.yaml`（显式声明
`allow_uncalibrated_debug: [camera]`，该 profile 下相机矩阵与迁移前逐元素相同，单相机行为
不变）。被否的方案：给部署 profile 也豁免 camera（把「容忍未标定相机」写进部署声明，
此后防线只剩诊断）；改用显式 `calibration_mode` 参数（会提前钉死 OFF-17 尚未设计的模式接口）。
注意：`source_topic_lidar` 为空时 LiDAR 不订阅，所以「用占位外参跑双传感器」在三种方案下
都不会发生——差别只在未标定相机能否起步，以及这个「能否」写在哪里。

**rclpy/Humble 实测坑（已按实测实现，勿凭直觉改回）**：① 空列表默认值会被推断成
`BYTE_ARRAY`，之后字符串数组覆盖在声明期就抛 `InvalidParameterTypeException`
（所以 `allow_uncalibrated_debug` 的默认值是 `[""]` + 过滤空串）；② params 文件里写
`key: []` 会让该参数变成「未初始化」，读 `.value` 抛 `ParameterUninitializedException`；
③ `DiagnosticStatus.level` 的 `byte` 字段在 rclpy 里绑成「长度 1 的 bytes」，赋 int 会在
assert 上崩（消费端/T6 读它时也要按 `level[0]` 取）；④ 节点未声明的 override 被静默忽略；
⑤ `/perception/status` 用 `qos_profile_sensor_data`（BEST_EFFORT）发布，默认 RELIABLE 的
订阅收不到**任何**消息，ROS 只在发现发布者时 WARN 一条 `incompatible QoS` 而**不报错**
（写消费端、含 OFF-17 汇总时必须先对齐 QoS）；⑥ 拒绝场景的命令行里 `-p` 必须是独立 argv
元素（`-p name:=value`），只传 `name:=value` 会被 rclpy 判为未知 ROS 参数并以 exit 1
结束——与真正的拒绝（exit 3）混在一起会让用例看起来穿过了拒绝门。

**验证**：两条缝各有一份常驻回归，都由 `run_all_tests.sh` 覆盖。① **纯加载器缝** ——
`tests/test_calibration_record.py`（69 项，含固定身份向量、镜像/非有限/底行/包络等数学拒绝、
provenance 判据、逐源豁免、软链接安装与复制安装两种部署形态、原始字节快照一致性）。
② **节点进程缝** —— `tests/test_perception_bridge_startup.py`（12 项，起真节点、隔离 ROS 域）：
演示 profile 起飞并逐条断言诊断内容（记录身份与解析路径、实例标识、被豁免相机的 WARN 与
不可准入、未启用源仍上报、严重级按单字节编码）；8 类拒绝都以 exit 3 结束并给出错误码；
被拒绝的实例不发任何诊断；运行期「对帧被融合（`camera_used` / `source_count` 上升）/
错帧被丢弃并留下限流告警」。没有构建产物时整文件跳过。
`.scratch/off01-extrinsics-source/` 下的 `verify_bridge_startup.sh`、`probe_bridge_params.py`、
`publish_test_cloud.py` 保留为交互式探针与上面那些坑的实测脚本，不再承担回归职责。
**仍未评估**：20mm/5° 与已知物体 <20mm 验收（固定错误码 `CALIB_ACCEPTANCE_NOT_EVALUATED`）。

## 仍待定（本 ADR 未覆盖）

1. T7 的**模式接口**（未标定调试 vs 正式避障）的命名、默认值与跨票分工：当前以
   profile 里的 `allow_uncalibrated_debug` 占位（Q9 已确认这一形态可用），是否升级为
   显式「模式」参数由 OFF-17 定。
2. 逐源**时间域的真实检测**（现在是「只报声明值」）：雷达包内 `time_type`、相机
   `time_domain` 与 `isGlobalTimestampSupported()` 的实际取值，归 OFF-06。
3. 相机 `input_frame` 与记录 `frame_from` 在真机上的实际取值核对（现按
   `camera_color_optical_frame` 对齐）：`depth_registration` 以外的启动参数组合会改变
   frame_id，届时运行期会丢弃点云并在诊断里可见，但需要现场确认一次。
