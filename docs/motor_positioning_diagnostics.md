# 电机定位诊断与原始记录回放

[`joint_table_readonly`](../src/robot_safecontrol_moveit/joint_table_readonly.py)提供设备静态读取及离线回放。设备读取需要对应现场授权；离线回放使用已有 JSON 或 ZIP，整个过程不创建串口连接。环境设置沿用[九轴定位入口](joint_positioning.md#运行入口)。

## 指定关节与字段

`--scope encoder` 使用 G 系列诊断字段，默认读取全部九轴。`--joints` 选择关节，`--fields` 明确选择属性；每个关节始终读取 `node_id`、`serial_number_l`、`hw_version`、`fw_version`。字段地址及类型以本机厂家 Python SDK v2.0 与《常用参数及地址表》为依据。

取得现场读取授权后，可按下面的命令采集 J1 计数与位置：

```bash
python3 -m robot_safecontrol_moveit.joint_table_readonly \
  --root "$PWD" --port /dev/serial/by-id/usb-Dr-Tech_DR-USB_CAN_87949380174B-if00 \
  --scope encoder --joints J1 \
  --fields gear_ratio cpr pole_pairs zero_offset_counts encoder_turn_abs encoder_pos_abs rotor_pos_turns output_pos_deg \
  --duration 3 --output .scratch/issue-184/j1-diagnostic-new.json
```

需要连续计数记录时，增加 `--sample-fields encoder_turn_abs encoder_pos_abs rotor_pos_turns output_pos_deg`。每批同时读取设备身份、传动、编码器参数、零位、多圈状态和设备限位，并与本次初始配置逐项比较。读到配置变化立即终止；`encoder_samples` 保存该批数值、主机时间区间及缺失字段。这些属性按顺序读取，设备采样时间未知。

输出文件必须尚不存在，父目录需要事先创建。字段或关节重复、字段无效及选项组合无效时，程序在打开设备前拒绝。读取连接只有属性读取权限。一次读取没有回复或回复无效时立即终止，保存已经取得的值、原始帧、具体失败字段和 `missing_properties`，随后重新抛出异常。

`property_observations` 为每个成功读取保存节点、地址、类型、值及主机请求与接收时间。`requested_properties` 保存本次完整请求范围。设备反馈的采样时间仍未知，不能把主机接收时间当作设备采样时间。

`encoder_mode`、`encoder_ready` 在厂家参数表中仅适用于 M 系列；`stored_turns_counts` 为保留字段，均不属于 G 系列 `encoder` 范围。`serial_number_h` 在已有现场记录中没有回复，也不列入本批次默认诊断范围。串口参数 `--scope identity`、`position`、`j1-recovery`、`multi-circle` 保持各自的读取定义。

`serial_number_l` 本次在九个节点均返回 1955，不能单独作为设备唯一身份。型号、铭牌及唯一身份仍需独立核查。报告中的 `hardware_qualified`、`zero_recovery_verified`、`physical_stop_independently_verified` 均保持 `false`。

`--scope operating` 读取完整身份、位置检查及运行参数，增加 `can_baud_rate`、`control_mode`、`input_mode`、`input_filter_bandwidth` 和 `trajectory_done`。该范围用于核对上电及模式切换后的配置，省略本批次没有回复的 `serial_number_h`。离线回放也核验这些运行字段与原始回复，报告数值不一致时立即拒绝。读取成功只说明本次取得的设备状态。

`--scope servo` 在 `operating` 基础上读取设备输入目标、内部位置／速度／扭矩目标、扭矩估计与限制、位置增益、速度增益、速度积分和电流环带宽。原始字段保留厂家单位，位置为输出轴 °、速度为 rpm、扭矩为 Nm；支持 `--joints J4` 等关节选择。该入口仅发送属性读取。厂家表限定为 M 系列的增益调度字段不属于本批 G 系列的读取范围。

## 离线回放

```bash
python3 -m robot_safecontrol_moveit.joint_table_readonly \
  --root "$PWD" --replay tests/data/motor_positioning_readonly_20261030.zip \
  --archive-member nineaxis-static-readonly.json \
  --output .scratch/issue-184/static-replay-new.json
```

直接回放 JSON 时省略 `--archive-member`。ZIP 使用 `zipfile` 读取成员，并核对 `provenance.json` 中的文件长度和 SHA-256；ZIP 不解压到文件系统。报告记录成员和归档身份。

回放从原始发送与接收帧重建属性及快速反馈，核验节点、属性地址、类型、时间顺序和报告中的属性值。连续采集还核对每批原始读取、时间区间、配置比较及属性记录。原始数据包含运动或参数写入命令时拒绝。`initial_nodes` 保存首次读取值，`nodes` 保存末次读取值，均按节点编号组织。

`replayed_completed` 表示所提供的读取记录解析完整；原采集状态另存于 `source_status`。原采集在最后一次回复缺失或帧不完整时终止的记录返回 `replayed_incomplete`，在 `incomplete_reads` 保留失败原因。未完成请求、成功报告中的损坏帧及不一致数据均不能得到完整回放结果。回放结论不提供真实运动、坐标恢复或机械精度证明。

## 本次证据范围

[`motor_positioning_readonly_20261030.zip`](../tests/data/motor_positioning_readonly_20261030.zip)保存本次授权下的三份原始记录，共有 378 项成功属性读取、1759 条快速反馈及一次无回复。全部发送帧均为 `0x1E` 属性读取。另存的 [`j1_encoder_configuration_20261029.json`](../tests/data/j1_encoder_configuration_20261029.json)与原始 Q14 静态记录字节一致，SHA-256 为 `3e89b6abd116b0b6562dd18521cceadd7d1ec8d759720dfa8641328eef621f92`。

当前 J1 回读为 `gear_ratio=36`、`cpr=16384`、`pole_pairs=14`、`zero_offset_counts=11145`；设备角度限位为 −36000°–1000°。该批只读采集没有改写设备参数。九个节点的 `can_timeout_ms=0`，设备采样时间未知，完整行程与独立停止仍需对应证据。同步接口的后续实际记录见[同步与断电实验](planning/motor-positioning/live-interface-experiments.md)。

[`j1_counter_sampling_20261030.zip`](../tests/data/j1_counter_sampling_20261030.zip)保存动作前九轴读取和 89 批 J1 静止连续计数，合计 1656 项属性读取与 1998 条快速反馈。

## 有人值守的计数动作记录

[`j1_bounded_counter_experiments_20261030.zip`](../tests/data/j1_bounded_counter_experiments_20261030.zip)保存 J1 的 0→1→0 mm、0→5→0 mm 四次实际动作、完整原始收发帧、连续计数、坐标配置、区间自身几何检查、现场说明及实验后九轴独立读取。四次动作只向 J1 各发送一条位置命令，共取得 5731 批计数与 51579 条九轴快速反馈。没有参数写入。

实验使用 0.05 mm/s、0.01 mm/s²，J1 运动前后采用 5.2 秒位置与速度窗口及至少 0.5 秒到位持续检查。其余八轴持续监测，动作范围限于本次已检查的 J1 区间及其余轴 ±0.01° 范围。外部空隙、急停按钮、持续看护和支撑由现场用户确认；停止时间、停止距离和完整场景尚无独立测量。实验结束后 J1 返回零位并保持使能，新的动作需要重新审查。

现场安装为 J1 沿基坐标系 Z 轴水平前后移动，基坐标系 +Y 向上；移动部件总质量与最大计划载荷待定，见[现场条件](planning/motor-positioning/field-conditions.md)。

已有 [`motion_log_replay`](../src/robot_safecontrol_moveit/motion_log_replay.py) 支持 `--encoder-capture`，同时核对位置命令、连续计数、九轴反馈覆盖和配置。提取归档中的对应三份原始文件后运行：

```bash
python3 -m robot_safecontrol_moveit.motion_log_replay \
  --log .scratch/issue-184-experiments/j1-outward-5mm.jsonl \
  --frames .scratch/issue-184-experiments/j1-outward-5mm.frames.json \
  --encoder-capture .scratch/issue-184-experiments/j1-outward-5mm.capture.json \
  --output .scratch/issue-184-experiments/j1-outward-5mm-replay-new.json
```

报告核对完整日志与收发帧摘要、单次 J1 命令的内容及时间、各节点反馈覆盖、每批属性及配置、原始速度窗口和到位状态。计数比例来自顺序主机读取的线性拟合，报告保留未知设备采样时刻及物理测量限制。卷尺读数为 500→495→500 mm，最小刻度 1 mm，不确定度未知；该人工记录支持 5 mm 相对位移核对，尚未完成独立连续精度验收。计数解释与完整行程所需依据见[设备记录](planning/motor-positioning/device-evidence.md)。

[`tests/test_joint_position_evidence.py`](../tests/test_joint_position_evidence.py)覆盖原始帧、归档校验、完整及部分回放、输入拒绝与主机读取时间。#184 的前置证据与现场条件见[设备依据](planning/motor-positioning/device-evidence.md)及[现场条件](planning/motor-positioning/field-conditions.md)。

## 研究补充证据

[`motor_positioning_research_20261030.zip`](../tests/data/motor_positioning_research_20261030.zip)保存当前连接的三份原始只读记录、现场条件、离线复算程序、结果和来源清单。新采集共 383 项成功属性、14967 条快速反馈及一次无回复；所有发送帧均为 `0x1E` 属性读取。完整静态采集和完整运行配置采集分别回放成功；`31003` 无回复的原始采集回放为 `replayed_incomplete`。

`analyze_research.py` 核对新数据的全部原始收发、地址、类型、数值和主机时间，并核对旧动作归档 36 个成员的长度与 SHA-256，再调用现有 `replay_encoder_probe` 复算四次真实动作。该程序没有创建设备连接；厂家属性验证使用本机纯枚举文件，资料路径及摘要列于来源清单。

使用 `zipfile` 或 `unzip` 将研究归档成员提取至一个新的 `.scratch/` 子目录，在[运行环境](joint_positioning.md#运行入口)下执行其中的 `analyze_research.py`。新目录需要保持 `.scratch/<名称>/analyze_research.py` 这一位置，并保留仓库中的原动作归档与本机厂家枚举文件；复算输出采用独占创建。归档内已有的三个派生 JSON 可以保留在原归档中，只提取三份 `*-readonly.json` 与 `analyze_research.py` 即可重新生成结果。`capture_operating_state.py` 为现场采集来源记录，运行它会连接设备。

派生结果分为 `operating-replay-summary.json`、`four-probe-replay-summary.json` 和 `derived-conclusions.json`，分别表达原始配置与主机时序、四次动作的完整复算、计数和通信容量计算。结果持续标明未知设备采样时刻、未验证真实同步及完整行程、未完成物理精度验收。数值与来源的解释见[设备依据](planning/motor-positioning/device-evidence.md#当前连接的复核与反馈时序)，待确认的设备规则见[设备确认事项](planning/motor-positioning/manufacturer-questions.md)。

## 同步接口与断电恢复的实际记录

[`motor_sync_live_validation_20261030.zip`](../tests/data/motor_sync_live_validation_20261030.zip)保存共同时间与 20 Hz 连续参考的六组实际实验、43290 条逐帧匹配并复算窗口的日志反馈，以及非零 J1 位置断电前后的 3744 条快速反馈。原生共同时间方式的正反向最大进度差约为 34.36%／32.50%，主机连续参考约为 2.32%／2.14%；这些结果限定于本次小范围及设备反馈。

用户报告断电前后卷尺测点到末端距离均为 499 mm，J1 的设备反馈坐标保持约 1 mm。旋转轴上电坐标及梯形运行参数发生变化，现有停止记录保持有效。完整方法、单位、源码版本、复算命令及验证限制见[同步接口与断电恢复记录](planning/motor-positioning/live-interface-experiments.md)。

## 九轴连续参考中止记录的回放

[`nineaxis_continuous_failure_20261030.zip`](../tests/data/nineaxis_continuous_failure_20261030.zip)保存全部九轴发生位移时的原始记录。J4 超出已检查范围，程序未能确认停止，用户随后确认已经急停、机械臂停止并有支撑。范围、发送方式及逐帧结果见[九轴连续参考实验](planning/motor-positioning/nineaxis-continuous-experiment.md)。此后的[原生单组定位](planning/motor-positioning/native-positioning-experiment.md)已单独完成实机验证。

现有 `motion_log_replay` 的 `--tracking-capture` 从采集文件读取内含的原始帧，使用 `--root` 指定坐标配置，不连接设备：

```bash
mkdir -p .scratch/nineaxis-tracking-replay
unzip -n tests/data/nineaxis_continuous_failure_20261030.zip \
  nineaxis-roundtrip-02.jsonl nineaxis-roundtrip-02.capture.json \
  -d .scratch/nineaxis-tracking-replay
python3 -m robot_safecontrol_moveit.motion_log_replay \
  --root "$PWD" \
  --log .scratch/nineaxis-tracking-replay/nineaxis-roundtrip-02.jsonl \
  --tracking-capture .scratch/nineaxis-tracking-replay/nineaxis-roundtrip-02.capture.json \
  --output .scratch/nineaxis-tracking-replay/result-new.json
```

回放核对原始日志身份、坐标配置、完整九轴预设／广播、逐轴自动回复、模式 1 保持命令、反馈及速度窗口。每条原始反馈与它之前最近一次实际主机广播的设备目标比较，包含尚未写入反馈日志的自动回复。主机发送、接收时间不能确定设备内部采用或采样时间。

本次结果为 `capture_status=aborted`、`continuous_roundtrip_completed=false`；`status=replayed_completed` 只表示离线核对结束。5949 条日志反馈全部匹配，全部原始快速反馈为 7542 条，524 批广播中首次范围违例来自 J4 的预设自动回复。`first_interval_violation` 保留原始帧、位置、最近广播目标及是否写入反馈日志。测试还使用原有四轴主要位移的成功记录，并拒绝与原始数据矛盾的终态、反馈和设备目标。独立机械精度、物理停止和设备资格字段保持 `false`。

当前 `tracking_probe` 研究连接还要求 `TrackingProbeBounds`，明确九个节点的已检查设备角度区间。区间缺失或无效时在打开串口前拒绝；收到范围外快速反馈或自动回复时保留原始帧并锁定连接，禁止随后发送。目标按实际 float32 编码检查范围。归档中的历史执行程序保留其执行版本；新的连续参考动作仍需重新核对当前姿态、参考采用、动态响应及现场条件。

采集在部分预设已发送、启动广播尚未发送时中止，回放将这部分保存为 `unbroadcast_presets`，并核对已收到的自动回复；完成终态不接受这种未完成批次。该状态没有提供设备采用或物理停止证明。

`response_dynamics` 保留每轴相邻原始反馈的最大位置变化、对应主机接收间隔、最近广播目标变化、设备速度和扭矩，以及最大跟踪误差的原始帧。由主机时间计算的位置变化率具有明确时间来源；设备采样时间和独立运动测量继续标记为未知。

[`motor_servo_diagnosis_20261030.zip`](../tests/data/motor_servo_diagnosis_20261030.zip)保存重新供电后的四份只读采集、原始响应分析及源码，共 918 项成功属性读取、4239 条快速反馈和一项没有回复的读取。全部发送均为 `0x1E`；完整采集与部分采集分别回放。J2–J6 读取的位置增益、速度增益和速度积分为 `10、0.2475、1.604`，用户确认调整历史未知。该参数差异与 J4 动态响应的因果关系尚未确定。
