# 九轴手动定位

[Python 程序](../scripts/joint_positioning.py)提供单组绝对目标和 CSV 逐组定位入口。每组全部九轴到位并停车后保持目标、持续监测，按 Enter 才开始下一组。真实执行采用有人值守的低速实验流程，要求全部可能运动范围内无人，操作者在范围外全程看护。每次启动核验设备零位配置、实际位置及连续静止反馈。

已确认的同步定位、整段路线检查、精度及断电保持要求见[九轴电机定位与 J1 全行程规格](planning/nineaxis-full-project/motor-positioning-spec.md)。本页说明现有入口、运行方式和设备记录。

设备静态属性、连续计数及小范围动作记录的离线核查见[电机定位诊断](motor_positioning_diagnostics.md)。共同时间、20 Hz 连续参考和非零位置断电的实际结果见[同步接口实验](planning/motor-positioning/live-interface-experiments.md)。

原生单组定位已完成[九轴实机验证](planning/motor-positioning/native-positioning-experiment.md)：目标 `[2, 1, -1, 1, -1, 1, 1, -1, 1]` 在约 13 秒后到位，并连续保持约 52 秒。J1 使用 mm，其余使用 °。[连续参考实验](planning/motor-positioning/nineaxis-continuous-experiment.md)的 J4 动态问题继续单独调查。

## 修改目标与参数

在程序顶部修改 `TARGET_JOINTS`、`MAXIMUM_SPEED` 和 `ACCELERATION`，每项完整包含 J1–J9。目标相对于已有机械零位；程序不设置设备零点。J1 使用 mm、mm/s、mm/s²，J2–J9 使用 °、°/s、°/s²。方向、零位与 J1 传动复用项目配置。

参数初值沿用现有低速入口：J1 为 0.25 mm/s、0.05 mm/s²，J2–J9 为 2.46°/s、0.3°/s²。J1 按此参数移动 50 mm 的名义时间约为 205 秒。输入转换为输出轴 rpm 和 rpm/s，再向下取到协议 0.01 步长；预览列出请求值、有效值及单位。现有运动限制继续检查，参数超出范围会拒绝；模型规定 J1 单次位移最多 50 mm，绝对目标范围为 0–500 mm，当前设备多圈计数尚未完成该行程验收。

也可以使用 `--joints` 直接输入九个值，覆盖程序内的 `TARGET_JOINTS`。该选项与 `--csv` 互斥；数量错误、非有限数值和目标超出模型范围时拒绝执行。预览的 `position_tolerance`、`position_unit` 分别显示到位误差阈值及单位：J1 为 1 mm，J2–J9 为 0.1°。

`SERIAL_PORT` 设置实际串口路径，或运行时提供 `--port`。`CSV_PATH` 保持 `None` 时使用文件顶部的单组目标；设置 CSV 路径或提供 `--csv` 时使用表格。相对 CSV 路径以项目根目录解析。

## 运行入口

在具备项目 Python、共同模型和硬件依赖的环境中，从仓库根目录运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src:$PWD/portable_oscbf:$PWD/.scratch/motor_tools/deps:$PWD/.scratch/nineaxis-execution/deps${PYTHONPATH:+:$PYTHONPATH}"
python3 scripts/joint_positioning.py --preview
python3 scripts/joint_positioning.py --preview --csv config/calibration_joint_table.example.csv
```

基本预览检查目标、参数、共同模型及单位转换，不连接设备。可通过 `--output .scratch/joint-positioning/preview.json` 保存新报告；现有文件不会覆盖。预览的 `parameters.limits` 列出默认检查参数；有人值守流程采用的反馈与静止检查参数记录在运行报告的 `limits` 中。

## 从原始起点生成共同进度预览

`--preview --start-readonly <JSON>` 从完整原始只读记录重建九轴末次反馈，核对节点、设备故障、九轴读取时间跨度、当前坐标配置摘要及模型范围，生成共同进度参考。输入文件只用于离线研究；设备上电坐标规则、当前实时反馈、静止资格及本批次协议能力仍需各自证据。

下面使用归档中的实际记录生成预览，不连接设备：

```bash
mkdir -p .scratch/motor-positioning-preview
unzip -n tests/data/j1_bounded_counter_experiments_20261030.zip \
  final-nineaxis-readonly.json -d .scratch/motor-positioning-preview
python3 scripts/joint_positioning.py --preview \
  --start-readonly .scratch/motor-positioning-preview/final-nineaxis-readonly.json \
  --output .scratch/motor-positioning-preview/reference.json
```

报告中的 `synchronized_plan` 包含实际记录的起点、目标、共同时长、各轴完整分段系数、位置／速度／加速度样本及共同模型计算的 TCP 路线。J1 使用 mm，J2–J9 使用 °，导数明确时间单位；完整系数的位置采用 m/rad，时间采用 s，各阶导数采用相应时间单位。`duration_s` 只累计运动参考，逐组等待和到位保持另行计时。

参考使用项目已有 Ruckig，按全部运动轴允许的归一化限制生成一个共同进度，再转换为九轴参考。检查全部运动轴的分段时间及归一化位置、速度、加速度和 jerk 相同，并验证进度单调、零位移轴保持、完整位置与导数范围。起终点速度及加速度为零，末段通过终点边界条件计算，CSV 各组在静止目标处衔接。参考 jerk 与最小时长采用 `config/oscbf_plant.yaml` 中的平滑设置，报告保存文件摘要；该设置不提供实际设备 jerk 能力证明。请求速度与加速度继续使用既有 0.01 协议量化后的限制。

原始反馈数值保持原值，超出模型范围的起点会拒绝，程序不把越界读数改为边界位置。无回复、部分读取、配置变化、缺少九轴、故障或未完成反馈均不能提供完整起点。到位后的速度资格、上电绝对坐标与位置有效性仍分别标记为未验证。

提供 `--scene-config <路径>` 时，对同一条完整参考调用既有连续运动检查。参数接受静态实验 YAML，或 `ExperimentConfig.scene()` 格式的 JSON 场景；JSON 必须明确 `frame_id`、工件、桌面、障碍物、安全距离及完整自碰撞来源。场景使用当前十个闭合凸 mesh、全部 36 个非相邻组合，外部和自身间距分别至少为 30 mm、10 mm。场景坐标、几何与输入文件身份保存于报告。

沿途碰撞或区间尚未证明时，保留报告并返回退出码 2；已证明路线记录检查区间和最小剩余裕量。报告保存检查程序摘要、最小检查区间 10 µs 与最多 4096 个区间的检查策略。检查范围为参考与名义终端停止，运动中任意时刻的实际停车、工具几何及现场场景资格保持各自的未验证状态。`reference_route_accepted=true` 表达声明场景中的参考区间结果，`hardware_executable` 继续为 `false`。

`--start-readonly` 和 `--scene-config` 只用于 `--preview`，不能用于启动真实运动。已有预览中的 `rows.device_commands` 描述单节点梯形接口；共同参考的完整定义保存在 `synchronized_plan.rows.reference_profiles`。本批次同步和连续接口已取得[小范围实机记录](planning/motor-positioning/live-interface-experiments.md)，公开入口的共同进度设备执行仍需对应资格与接入验证。

## 真实定位入口

真实入口为：

```bash
python3 scripts/joint_positioning.py --joints 2 1 -1 1 -1 1 1 -1 1
python3 scripts/joint_positioning.py --csv config/calibration_joint_table.example.csv
```

单组在设备读取、实际位置和静止检查通过后开始；CSV 第一组等待 Enter。绝对目标仍相对于既有机械零位，可以从已确认静止的当前位置开始。启动静止检查需要约 5.5 秒连续反馈。表格列格式沿用[标定姿态表](calibration_joint_table.md)，每行包含完整绝对目标。

本机现场记录位于 `.scratch/motor_control/attended-positioning.json`，绑定已审阅停止记录、串口、模型、坐标配置、J1 设备身份与设备零位，以及独立的静止原始记录。程序实际读取九轴闭环状态、故障、限位及轨迹模式，检查全部目标和回零目标。更换设备或坐标配置后，现场记录需要重新核查；缺失或身份不符会在创建真实 I/O 前拒绝。`--experiment-policy` 可指定经过相同现场检查的记录路径。

有人值守流程单独保存新的异常锁定 `.scratch/motor_control/attended-positioning-stop.json`。发生异常后必须根据运行报告处理原因，程序不会自动清除或重新开始。普通自动执行和 ROS live 保持各自的设备资格要求。

## 到位、等待与退出

整组完成前，各轴的静止状态参与共同稳定时间累计。暂时未满足静止条件时继续等待并重新计时；位置离开到位容差、反馈故障和速度违例仍中止。最后一轴到位之后，全组必须在报告中的稳定超时内完成共同检查。

各轴采用驱动器梯形位置模式，可重叠运动并先后到位。位置误差、速度、轨迹完成标志和共同稳定时间继续检查；单组定位的位置容差为 J1 1 mm、J2–J9 0.1°。J1 按已确认的 5 mm 导程转换为输出轴 72°，用于执行器的 `tolerance_deg`；关节行程限制独立生效。静止判定同时要求五秒输出轴位置范围不超过 0.01°、位置变化速度不超过 0.02 rpm、速度估计与独立静止记录的噪声范围相容、轨迹完成，并共同满足至少 0.5 秒。运动速度保护使用独立的一秒窗口、近期位置变化速度及相邻反馈位置连续性检查。本次有人值守实验的旋转轴反馈速度上限为 5°/s，位置指令速度初值为 2.46°/s；J1 反馈速度上限仍为 0.3 mm/s。这些数值用于编码器反馈判定；机械精度需要独立测量。

到位后程序显示每轴误差，并继续读取反馈。等待 Enter 期间没有新的运动目标，也没有测量文件要求。最后一组及单组到位后均继续保持和监测。

| 输入 | 九轴确认到位并停车后执行的操作 |
|---|---|
| Enter | 执行表格下一组；最后一组保持当前位置 |
| `r` 加 Enter | 确认正常实验结束，九轴共同开始回零，各轴沿用当前速度、加速度与梯形模式 |
| `q` 加 Enter | 退出程序，保持当前位置，不发送失能或回零命令 |

回零目标是相对已有机械零位的九轴全零绝对位置，不写入设备零点。回零属于本次实验的结束操作；回零后不再继续表格，九轴反馈确认到位及停车后进入 `holding_zero`，保持使能并持续监测，直到输入 `q` 退出。回零继续检查原有关节范围、J1 单次位移、反馈时效及运动限制；不满足条件时拒绝，不自动分段或重试。预览的 `return_to_zero` 列出全零目标及实际设备参数。

运动期间输入 Enter 或 `r` 会被清除，不能预约下一组或回零；提示出现前的输入也不能预约操作。零位保持期间的 Enter 或 `r` 不重复发送命令。正常退出只关闭通信连接，不发送失能或设备参数命令。

Ctrl+C、运动期间的 q 或任何执行异常会终止本次序列，不自动回零。通信正常且位置可信时请求当前位置保持，并用后续反馈检查停止；通信失效或停止未确认时如实报告，不自动重连、重试或继续下一组。驱动失能或断电后的重力转动无法由本程序消除，回零及通电保持均不能提供断电制动能力。运行报告保留这项设备能力缺口。

运动中请求当前位置保持仍使用梯形位置命令。观察期限考虑减速后返回保持目标所需时间，再加入完整静止窗口；命令返回与反馈确认停车分别记录。

运行记录默认使用 `.scratch/joint-positioning/` 中的新 JSONL，另存原始收发 `.frames.json` 和最终 `.report.json`。报告区分定位完成记录、测量记录及真实停止能力，按 Enter 不生成标定测量。回零尝试独立保存在 `return_to_zero`，同时写入 `attempts`，不增加表格完成组数；没有发起回零时该字段为 `null`。回零中止记录原因，回零反馈完成事件为 `return_to_zero_completed`，正常退出的 `termination.returned_to_zero` 表示此次退出前是否完成回零。

## 本地验证

`tests/test_joint_positioning.py` 使用真实管道输入、CSV 和 MuJoCo 的 qpos/qvel 检查手动推进、部分终端输入、持续等待、确认后从非零姿态回零、零位保持及故障拒绝；既有单组调试与设备原始日志继续回归。MuJoCo 使用已有的无重力、无接触场景，结果标识 `simulation_feedback`，不提供真实设备或负载停止证据。

## 当前设备试验

目标 `[50, 10, -20, 30, -30, 60, 10, 20, 30]` 已实际发送。J2–J9 到达目标并保持使能；J1 的反馈从约 0.746 mm 增至约 35.54 mm，随后驱动进入状态 1，轴故障 256、编码器故障 512。厂家 SDK 将其定义为 `AXIS_ERROR_ENCODER_FAILED` 和 `ENCODER_ERROR_EDR_OVER_RANGE`。运动前 `encoder_turn_abs=-42`，后续独立读取为 −2047；设备零位计数仍为 7733。

运行程序在一次 J1 状态读取没有回复时中止，后续独立非运动读取确认上述故障。该次定位没有自动重试或回零。原始记录保存在 `tests/data/attended_positioning_j1_fault_20261029.zip`。

厂家《DrEmpower 系列电机使用手册 v1.0》物理第 13–14 页要求此类编码器错误先重新设置零点，再清除错误。资料尚未明确本台固件的多圈计数范围，50 mm 目标和完整行程仍未通过设备验收。

用户明确要求回零后，程序仅向状态正常的 J2–J9 发送八条原机械零位位置命令，沿用 2.46°/s、0.3°/s² 以及原反馈和静止规则。约 38.1 秒后八轴通过到位与持续静止检查，保持使能；程序没有向故障中的 J1 发送运动或参数命令。

用户随后自行将 J1 回到原机械零位并重置零点。独立六秒读取确认九轴均为闭环状态 8，轴与编码器故障均为 0；J1 反馈约 0 mm，新的 `zero_offset_counts=11145`，多圈使能为 1。有人值守现场记录已同步该零位计数。J2–J9 均接近 0°。回零操作及用户恢复后的原始读取归档为 `tests/data/requested_zero_recovery_20261029.zip`。

零位计数 11145 的配置下，J1 已完成 0→1→0 mm、0→5→0 mm 双向计数实验；卷尺往返读数为 500→495→500 mm，最小刻度 1 mm。后续实际移动并保持约 32.0068 mm，再次执行 50 mm 目标时，最后有效位置约 35.538 mm，随后通信中断。用户急停并重新供电后，J1 多圈使能为 1、计数为 −2047、编码器错误 512、轴错误 257、状态 1。详细条件与原始帧见[较大目标实机验证](planning/motor-positioning/larger-positioning-experiment.md)。

用户再次将九轴恢复机械零位并重置电机零点后，两份六秒只读采集确认九轴均为状态 8、五类故障为零，J1 的 `zero_offset_counts=11078`、`encoder_turn_abs=0`、多圈使能为 1。该次原始记录与配置见[初始零位核验](planning/motor-positioning/zero-reset-verification.md)。

后续授权的[多圈计数实验](planning/motor-positioning/j1-multi-circle-experiment.md)在保留限位、关闭多圈时到达并保持 40 mm，独立回程完整返回零位。重新启用多圈时错误 512 再次出现，设置零点、清错并恢复闭环后，九轴均为状态 8、五类故障为零。最终 J1 零位计数为 `11045`、多圈计数为 `0`、多圈使能为 `1`，本地定位配置采用该零位计数。当前停止记录仍要求启动审查；完整行程、断电坐标保持、J4 连续运动动态原因和独立停止能力仍待验证。
