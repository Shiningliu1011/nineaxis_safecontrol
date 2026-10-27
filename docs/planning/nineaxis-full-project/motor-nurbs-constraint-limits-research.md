# 电机与 NURBS 约束数值依据

核验日期：2026-10-27。单位遵守[项目单位约定](../../units.md)：J1 使用 mm，J2–J9 使用 °；控制接口使用 m/rad。本页记录厂家资料、代码读取与独立数值计算，实际设备能力只采用有对应条件和验证记录的结论。

本轮用户确认研究对象为曲面跟随与避障，NURBS 依据师兄提供的最新 `NURBS曲线.zip` 程序。源路径的运动限制用于研究条件；电机资料按各自电压、传动和负载条件登记。运行配置尚未修改。本文为 [M07 公共限制](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/138)的数值证据，尚待选择的模型参数继续保持其研究假设身份。

用户确认当前电源显示约 24 V，研究速度上限采用 J2–J6：120 °/s，J7–J9：150 °/s；两组数值均按厂家 36 V 额定速度的 50% 选取。这是人为选择的研究参数，24 V 下的额定负载速度仍缺少厂家资料和本机验证。

## 电机型号与厂家资料

当前项目映射为 J2–J6 使用 `B25-PD-36-G`，J1、J7–J9 使用 `B06-PA-36-G`，来源为 [`joint_module_mapping`](../../../portable_oscbf/config/actuator_modules.yaml)。现场记录仍要求人工核对节点与关节的安装对应关系；2026-10-16 的通信试验取得节点 1–9 的回复，记录的型号与固件信息尚不足以确认具体产品版本。来源：[通信记录](../../drempower_communication_20261016.md)、[设备信息现状](../../hardware_laptop_quickstart.md)。

本机厂家原始资料：

| 资料 | 本次使用的位置 | SHA-256 |
|---|---|---|
| [DrEmpower 系列产品选型手册 v1.2](<../../../大然电机/大然电机DrEmpower系列产品选型手册 v1.2.pdf>) | 印刷页码 37、47、55；对应型号参数表 | `7914dc69394a190c46d0f6dd57eb8d0dd024ff18ab558631d7a6bd80e70a7dec` |
| [DrEmpower 系列电机使用手册 v1.0](<../../../大然电机/大然电机所有资料/drempower-wiki-master/1-产品手册及使用说明书/DrEmpower系列电机使用手册 v1.0.pdf>) | §1.2.1 运动模式、§1.3 待机阻尼、使用注意事项 10、13 | `64fd61aacc537fd62c798320c5d98d1acd250990831e8570b121305d1e8ac1b8` |
| [DrEmpower 电机库函数说明 v2.1](<../../../大然电机/DrEmpower电机库函数说明 v2.1.pdf>) | §2.1 参数与示例、§2.5 速度控制、§2.10 急停、附录 | `b389fdb64593281a63b9c336354942efdedefdb7464a92970023278192b6ff4f` |
| [DrEmpower 系列电机常用参数及地址表](<../../../大然电机/大然电机所有资料/drempower-wiki-master/6-其他资料/DrEmpower系列电机常用参数及地址表.pdf>) | 印刷页码 41–45 | `8ac76339e93f475cf9e24ee76546e756a4791563114239164375277f06a998fc` |
| [DrEmpower 电机 CAN 通讯协议说明 v2.0](<../../../大然电机/大然电机所有资料/drempower-wiki-master/5-通信协议/DrEmpower电机CAN通讯协议说明 v2.0.pdf>) | 表 1-3、§2.1 | `464e65de8a4c6fdd00d3609e5052782777170da5a382946a88b7f14562f7ab98` |

厂家线上入口已核验：[DrRobotTech/drempower-wiki](https://github.com/DrRobotTech/drempower-wiki)列出上述版本的资料；[DrEmpower 智能一体化关节 Python 库函数详解](https://www.daran.tech/nd.jsp?id=6)为厂家官网发布的接口说明，发表日期为 2024-04-25，本次取得并阅读完整正文。数值表以下述本机 PDF 为依据，保存文件身份可供后续核查。

## 厂家额定能力与电压条件

| 项目 | `B25-PD-36-G` | `B06-PA-36-G` |
|---|---:|---:|
| 厂家输入电压范围 | 24–48 V | 12–36 V |
| 内置减速比 | 1:36 | 1:36 |
| 36 V 额定输出转速 | 40 rpm = 240 °/s = 4.188790205 rad/s | 50 rpm = 300 °/s = 5.235987756 rad/s |
| 36 V 最大空载输出转速 | 46 rpm = 276 °/s = 4.817108736 rad/s | 71 rpm = 426 °/s = 7.435102613 rad/s |
| 额定扭矩 | 25 Nm，表中注明 36 V | 6 Nm，表中未另注电压 |
| 峰值扭矩 | 81 Nm，表中未另注电压 | 28 Nm，表中注明 36 V |
| 额定电流 | 3.2 A | 3 A |
| 峰值电流 | 4.5 A | 12 A |
| 转子惯量 | 2670 g·cm² | 7.83 g·cm² |

来源：选型手册 v1.2 印刷页码 47、37/55。`B06-PA-36-G` 在手册中有多圈计数与单编码器产品说明，上表的转速、电流、扭矩数据一致；实际设备版本需要对应铭牌和反馈资料。峰值扭矩持续时间、重复占空比、负载运动的扭矩—转速曲线、温度与连续工作时间条件，在本次核查的对应型号表中没有给出。

换算采用 `ω_deg_s = 6 × n_rpm`、`ω_rad_s = π × n_rpm / 30`。使用手册注意事项 10 说明运动控制接口采用输出轴参数；`axis0.output_shaft.*` 的速度单位为 rpm。因此将这组输出转速换算到直接连接的转动关节时，内置的 `1:36` 已包含在给定能力中。来源：使用手册注意事项 10、常用参数表的 `axis0.output_shaft.vel_estimate` 与 `vel_limit`。

最近已有的供电回读记录来自 2026-10-16：两组电压范围为 23.99–24.26 V、24.02–24.25 V。上述额定转速与空载转速均附带 36 V 条件；本轮没有取得新的硬件回读。现有材料没有这两个型号在 24 V 下的额定负载转速，不能用电压比例或转速常数直接确认 24 V 的关节能力。来源：[通信记录](../../drempower_communication_20261016.md)第 28 行、选型手册型号参数表。

## 当前程序读取的转动关节速度

当前 [`control_limit_profile`](../../../portable_oscbf/config/actuator_modules.yaml) 的 `rated_speed_safety_factor=0.70`，[`load_actuator_limit_profile`](../../../portable_oscbf/work/actuator_limits.py)按额定输出转速乘此系数。本轮直接调用该函数，得到以下值：

| 关节 | 采用 36 V 资料和当前系数的速度上限 | 数值身份 |
|---|---:|---|
| J2–J6 | 168 °/s = 2.932153143 rad/s，各轴相同 | 240 °/s × 0.70 |
| J7–J9 | 210 °/s = 3.665191429 rad/s，各轴相同 | 300 °/s × 0.70 |

`0.70` 是项目已有的运行策略。厂家资料没有规定这个系数能覆盖当前整臂的负载、温度、安装方式、制动及控制时延。在实际供电、安装映射和负载能力确认前，上表可作为带来源的仿真或 shadow 候选值。当前函数的 `hardware_validated` 对转动轴仅表示转速换算来源已记录，其注释未把整机能力验证包含在内；本轮函数返回 `hardware_executable=false`、`unverified_joint_names=("J1",)`。来源：`actuator_limits.py` 第 56–67、148–155 行。

### 约 24 V 供电条件下的研究速度上限

用户确认的研究角速度绝对值上限如下：

| 关节 | 建议角速度上限 | 输出转速 | SI 数值 |
|---|---:|---:|---:|
| J2–J6 | 120 °/s | 20 rpm | 2.094395102 rad/s |
| J7–J9 | 150 °/s | 25 rpm | 2.617993878 rad/s |

选择依据为厂家 36 V 额定输出速度分别为 240 °/s、300 °/s，本轮研究采用其 50% 作为运行限速。百分比属于研究设置，不提供额外的负载能力保证。本次补查仍未取得对应型号在 24 V 下的额定负载转速或扭矩—转速曲线，因此这组值不标记为厂家 24 V 额定值，也不标记为已验证的真机限速。

表内数值已经是建议上限，后续采用该配置时直接使用表内绝对值，单位转换后保持数值含义一致。关节限制与源 NURBS 的 60 mm/s 末端进给上限共同作用，受关节限制时实际可行进给可以降低。J1 保留独立的线速度及传动换算。本轮只完成数值建议和独立单位换算，运行代码与配置保持原内容。

## J1 丝杠与额外传动

现有文档记录单头丝杠导程为 5 mm/rev，测量日期为 2026-09-06；输出轴到丝杠的额外传动比、效率、行程、方向和回零条件尚待确认。来源：[实机执行资料](../../real_robot_execution_plan.md)的决策 9 与 P2、`actuator_modules.yaml` 第 99–103 行。

令 `r` 为电机模组输出轴转数除以丝杠转数，保持与 [`TransmissionSpec.ratio_motor_rev_per_lead_rev`](../../../src/robot_safecontrol_moveit/unit_conversion.py)相同定义，则：

\[
v_{J1}=\frac{n_{output}\,5}{60r}\quad\text{mm/s}.
\]

因此在厂家 36 V 转速条件下：

| 量 | 包含未知额外传动比的结果 | 输出轴与丝杠直接 1:1 连接时的条件计算 |
|---|---:|---:|
| 额定线速度 | `4.166666667/r mm/s` | 4.166666667 mm/s |
| 最大空载线速度 | `5.916666667/r mm/s` | 5.916666667 mm/s |
| 额定线速度乘项目 `0.70` | `2.916666667/r mm/s` | 2.916666667 mm/s |

第三列依赖 `r=1`，现有测量记录尚未确认该条件。仅知道丝杠自身每圈移动 5 mm，还不能确认电机模组输出轴每圈对应的移动距离；`TransmissionSpec` 的默认 `r=1` 同样需要设备依据。速度换算不依赖效率，力换算需要效率与传动比；丝杠和导轨的允许载荷还需要各自资料。来源：`unit_conversion.py` 第 84–102 行。

当前仿真 J1 上限为 500 mm/s。若 `r=1`，该线速度要求输出轴转速 6000 rpm，相当于厂家 36 V 额定转速的 120 倍。该仿真数值没有现有电机与丝杠资料支持其用于真机。来源：`actuator_modules.yaml` 第 100 行及上述独立计算。J1 的实际速度上限仍需确认 `r`、当前供电条件及丝杠本身允许转速。

## 加速度、jerk 与制动能力

厂家资料中找到了目标加速度的设置入口和通信表达范围。对上述两个具体型号，本轮核查的选型手册、使用手册、库函数说明、常用参数表、CAN 协议及厂家官网接口全文，均未给出带负载条件的最大机械加速度或 jerk 上限。以下逐项保留已有证据。

| 厂家资料项目 | 已找到的内容 | 可确认的含义 |
|---|---|---|
| `set_angle(..., param, mode=1)` | `param` 为启停目标加速度，单位 rpm/s；`speed`、`accel` 要求大于零 | 轨迹指令的设定值 |
| `set_angle(..., param, mode=0)` | `param` 为角度输入滤波带宽，要求小于 300；`speed` 不起作用 | 输入滤波参数 |
| 库函数说明 §2.1 的示例 | 10 rpm 运动、20 rpm/s 加速度，即 120 °/s² = 2.094395102 rad/s² | 示例命令的选择 |
| `axis0.output_shaft.trap_traj_accel_limit` / `trap_traj_decel_limit` | 地址 `38015` / `38016`，单位 rpm/s，允许修改 | 输出轴梯形轨迹的加速与减速设定 |
| `axis0.output_shaft.vel_ramp_rate` | 地址 `38017`，单位 rpm/s，允许修改 | 速度模式的目标变化速率 |
| `axis0.output_shaft.torque_ramp_rate` | 地址 `38018`，单位 Nm/s，允许修改 | 扭矩模式的目标变化速率 |
| `axis0.trap_traj.config.accel_limit` / `decel_limit` | 地址 `35102` / `35103`，单位 turn/s² | 电机内部坐标的轨迹参数 |
| CAN `0x1A` 加速度字段 | `Signed short`、倍率 `0.01`；最大可表达正数为 327.67 rpm/s = 1966.02 °/s² = 34.313522160 rad/s² | 消息数值表达范围 |
| S 形速度轨迹 | `axis0.trap_traj.config.traj_mode=2`；加速度从零逐渐增加到给定值 | 轨迹类型功能；材料未给型号 jerk 上限 |

来源：库函数说明 v2.1 §2.1、§2.5、附录；使用手册 v1.0 §1.2.1；常用参数表印刷页码 43–44；CAN 协议 v2.0 表 1-3。官网[接口说明](https://www.daran.tech/nd.jsp?id=6) §3.1 同样说明加速度与滤波带宽的模式含义。接口范围、示例数值、可设置属性和机械能力分别记录；当前材料不支持把示例 120 °/s² 或字段上界 1966.02 °/s²确认为整臂加速度限制。

厂家官网接口说明 §3.5–3.6 说明：给定速度与力矩组合不能提供充分减速能力时，会发生目标角度过冲。这表明需要同时考虑运动状态和可用力矩。转子惯量只描述电机自身，整臂加速度还取决于负载与连杆惯量、重力、摩擦、传动效率和其他关节运动。厂家常用参数表印刷页码 45 也要求把输出轴负载惯量按减速比平方换算，再与转子惯量相加。

关于电流和热限制，常用参数表列出 `axis0.motor.config.current_lim`、`axis0.output_shaft.torque_lim`、温度与保护阈值；其中 `axis0.motor.effective_current_lim` 在表中标为 M 系列适用，需要核对 G 系列的对应读取接口。使用手册注意事项 13 说明默认温度保护范围为 95–100 °C，高于下限会限制电流并减少输出扭矩。本次材料没有当前九个设备这些属性的有效回读与负载验证记录，额定或峰值扭矩无法单独给出整臂逐轴的加速度数值。

关于停车，使用手册 §1.3 明确说明“电机不带机械抱闸”；待机阻尼在输出轴转动时提供阻力。库函数说明 §2.10 说明 `estop()` 切换到 `IDLE`、卸载电机并设置错误标志。常用参数表还列有再生电流与电源吸收电流等配置。上述材料未给出这两个型号在本机器人负载下的最小制动减速度、停车时间或停车距离，现有材料不足以指定对应数值。来源：上述厂家资料及[实机执行资料](../../real_robot_execution_plan.md)的 P4、H4.4。

## 研究加速度与加加速度建议

针对已确认的研究速度上限，建议采用以下角加速度与角加加速度绝对值上限。它们按变速时间与平滑程度选择，厂家当前材料未提供本机器人在 24 V 和实际负载下的能力保证值。

| 关节 | 已确认速度上限 | 建议加速度上限 | 建议加加速度上限 |
|---|---:|---:|---:|
| J2–J6 | 120 °/s | 120 °/s² = 2.094395102 rad/s² | 600 °/s³ = 10.471975512 rad/s³ |
| J7–J9 | 150 °/s | 150 °/s² = 2.617993878 rad/s² | 750 °/s³ = 13.089969390 rad/s³ |

数值选择采用 `a_max = v_max / (1 s)`、`j_max = a_max / (0.2 s)`。对应当前 [`MotionPlanner`](../../../portable_oscbf/work/static_motion.py) 的 `max_jerk = max_acceleration / jerk_time`，该研究建议使用 `jerk_time=0.2 s`。从零初始加速度开始，以最大允许 jerk 建立最大加速度需要 0.2 s；这只是所选模型的加速度变化时间。

本轮使用项目已有环境中的 Ruckig 0.12.2 实际计算单轴启动、停车、反向，以及双轴共同启动和停车；接口与现有程序相同，为 `ControlInterface.Velocity` 和 `Synchronization.Time`。初始与目标加速度为零、其余约束不限制该变速过程时，两组从静止达到速度上限均需 1.2 s，阶段时间分别为 0.2 s 增加加速度、0.8 s 保持加速度、0.2 s 减小加速度。SI 换算、完整分段及边界状态见[实际计算结果](../../../.scratch/constraint-limits/check_research_acceleration_jerk.json)，[复核脚本](../../../.scratch/constraint-limits/check_research_acceleration_jerk.py)仅进行本地运动数学计算，没有设备 I/O。

相同初始加速度条件下，从速度上限停车需要 1.2 s，模型停车转角分别为 72°、90°。J7–J9 的名义总行程约 170°，因此静止到静止的完整运动会受行程进一步限制：独立 Ruckig 位置模式的全行程计算得到约 145.390149 °/s 的峰值；保留现有两侧 CBF 余量时约为 144.853410 °/s。实际获准运动还要检查初始状态、剩余行程、其他轴的同步与完整停止过程，速度上限并不要求每段运动都达到该值。

上述检查验证了这组研究参数对应的 Ruckig 变速曲线；未验证最新 NURBS 路径的整臂闭环、真实负载或物理停车。J1 的线加速度和线加加速度需要根据实际传动、负载和对应运行速度单独确定。

## 当前仿真与规划数值

[`actuator_modules.yaml`](../../../portable_oscbf/config/actuator_modules.yaml)第 91–104 行将加速度明确标为仿真假设。`StaticExperimentConfig.execution_limits()` 从该文件读取数值，按 `max_jerk = max_acceleration / jerk_time` 构造 jerk；当前 [`oscbf_plant.yaml`](../../../config/oscbf_plant.yaml) 的 `jerk_time=0.08 s`。本轮调用配置读取和换算得到：

| 关节 | 当前仿真速度上限 | 当前仿真加速度上限 | 当前仿真 jerk 上限 |
|---|---:|---:|---:|
| J1 | 500 mm/s | 5000 mm/s² | 62500 mm/s³ |
| J2–J6 | 168 °/s | 1799.087477 °/s² = 31.4 rad/s² | 22488.593459 °/s³ = 392.5 rad/s³ |
| J7–J9 | 210 °/s | 1799.087477 °/s² = 31.4 rad/s² | 22488.593459 °/s³ = 392.5 rad/s³ |

这些加速度与 jerk 尚无厂家负载能力或实际整臂测量依据。`31.4 rad/s²` 与 CAN 字段可表达范围接近，也不能据此推定机械能力。来源：[`execution_limits()`](../../../src/robot_safecontrol_moveit/static_experiment_config.py)第 252–270 行、[`MotionPlanner`](../../../portable_oscbf/work/static_motion.py)第 98–109 行。

项目还保存有其他数值，分别由其使用入口决定作用范围：

| 来源 | J1 速度 / 加速度 | J2–J9 速度 / 加速度 | effort |
|---|---|---|---|
| [MoveIt `joint_limits.yaml`](../../../models/ninezzhou_moveit_config/config/joint_limits.yaml) | 150 mm/s / 200 mm/s² | 45.836624 °/s / 85.943669 °/s² | 未在该文件设置 |
| [URDF](../../../models/ninezzhou/urdf/ninezzhou.urdf) | 200 mm/s；未给加速度 | 85.943669 °/s；未给加速度 | J1 为 1000 N；J2–J9 为 50 Nm |

上述 URDF 与 MoveIt 数值属于模型和规划配置，现有资料没有把这些数值验证为具体设备的持续运动能力。厂家表中 J2–J6 的额定扭矩为 25 Nm，J7–J9 的额定扭矩为 6 Nm、峰值为 28 Nm。长期约束与短时峰值需要保留各自的适用条件；J1 的力限制还需要额外传动比、效率与丝杠允许载荷。

## 本轮电机数值核验

- 用 `pdftotext -layout` 读取上述五份厂家 PDF，仅采用文本提取。
- 实际运行 `.scratch/constraint-limits/inspect_motor_limits.py`，读取生产 `load_actuator_limit_profile()`，独立换算 rpm、°/s、rad/s，读取 URDF 与 MoveIt YAML，计算条件 J1 速度、CAN 字段表达范围及当前仿真 jerk。转速配置与独立换算的差值小于 `1.8e-11 rad/s`。
- 数值输出保存于 `.scratch/constraint-limits/motor-limits-calculation.json`；官网完整正文保存于 `.scratch/constraint-limits/official-python-library.txt`，原始网页保存于同目录的 `official-python-library.html`。
- 没有修改控制代码、运行配置或设备状态；没有执行硬件命令。本轮没有验证真实关节加速度、jerk 或停车能力。

## 当前模型的位置限制

当前 URDF 提供以下位置边界，本轮用 XML 库读取原值并换算。它们可以作为该机构研究模型的位置限制，实际行程、装配干涉和零位仍按设备资料核验。来源：[URDF](../../../models/ninezzhou/urdf/ninezzhou.urdf)的各关节 `limit` 元素；独立计算输出中的 `urdf_position_project_units`。

| 关节 | 当前模型的位置范围 |
|---|---:|
| J1 | 0–585 mm |
| J2–J4 | −90.000210° 至 +90.000210° |
| J5 | −180.000421° 至 +180.000421° |
| J6–J9 | −85.000008° 至 +85.000008° |

模型中的边界余量另由 [`joint_limit_contract.py`](../../../portable_oscbf/work/joint_limit_contract.py)定义。`JOINT_LIMIT_CBF_MARGIN=0.01` 对 J1 为 10 mm，对转动关节为约 0.572958°；该控制参数与电机能力分开记录。

## 最新 NURBS 程序中的研究限制

源文件为本机 `/home/lsn/robot/NURBS曲线.zip`，SHA-256 为 `907fbfc97e961aca04e83b69b8a38839aff2e4f6e32e98c8e89f7a2c1d06e58d`。下文 `nurbs_nihe/` 指包内：

`NURBS曲线/DiYiJieDuan/shiyanjianmo/jianmo_guiji/yilaixiang/jiemian/lujing/nihe_yilai/nurbs_nihe/`。

`sudu_yilai/sudu_fenpei/A_2_lixianjisuan.m` 第 9–19 行设置参数，`nurbs_blocks.mat.limits` 保存相同数值。`A_3_shishichabu.m` 第 40–48 行读取该结构并检查运动；`pipe_pose_schedule.m` 第 42–44 行继续使用 `offline.limits` 检查加入姿态后的轨迹。源位置与姿态数据的身份见下表。

| 源文件（相对 `nurbs_nihe/`） | SHA-256 |
|---|---|
| `sudu_yilai/sudu_fenpei/nurbs_blocks.mat` | `7f7295cb7b7bf2b73b2b0e07f4e555a4cdbec4815cdd516982edd7dc8c63a4d0` |
| `sudu_yilai/sudu_fenpei/ik_input.mat` | `cd724fcfb13dfcdc86a8f21de4bdd69a3ab88a97e66f6eff613774ba1d08d341` |
| `sudu_yilai/sudu_fenpei/zitai_yilai/zitai_gouzao/ik_input.mat` | `74acea6a2033e72e0be182d07e800c414d453ed3477170f9cb7cfe2976bc0942` |
| `sudu_yilai/sudu_fenpei/zitai_yilai/zitai_gouzao/pose_reference.mat` | `0deb8a83cb47f3397085eede4b84765933c1d08acf404189521f71c18a3f85b9` |

采用该源程序作为曲面跟随研究条件时，路径限制使用以下数值：

| 参数 | 研究数值 | 约束对象与定义 | 控制接口中的 SI 数值 |
|---|---:|---|---:|
| `F` | 60 mm/s | 路径进给上限；源验证同时检查瞬时速度与相邻采样点的弦长速度 | 0.060 m/s |
| `At` | 100 mm/s² | 进给时间律的加速度 `dv/dt` 的绝对值；源验证还检查阶段解析峰值及弦长速度的变化 | 0.100 m/s² |
| `Jt` | 16400 mm/s³ | 进给时间律的 jerk `d²v/dt²` 的绝对值；源验证还检查相应阶段峰值与离散变化 | 16.4 m/s³ |
| `An` | 300 mm/s² | 沿给定曲线的法向加速度 `κv²` | 0.300 m/s² |
| `Jn` | 16400 mm/s³ | 同时限制 `κ²v³` 和 XYZ jerk 相对曲线切向的法向投影模长 | 16.4 m/s³ |
| `delta` | 0.001 mm | 插补曲线相对相邻位置连线的弦误差 | 0.000001 m |
| `Ts` | 0.002 s | 源插补采样周期，即 2 ms | 0.002 s |

定义来源：`sudu_yilai/pipe_feed_profile.m` 第 569–654 行。`At`、`An` 分别限制切向与法向量，XYZ 加速度总模长需要另外记录。`Jt` 对应进给速度的二阶时间导数；XYZ jerk 的切向投影还包含曲率项，因此 XYZ jerk 模长也要单独记录。以上数值分别适用于对应物理量，不直接作为九个关节的加速度或 jerk 限制。

`Fmin=10 mm/s` 的源码注释明确限定为几何扫描。`limits.position_tol=0.000001 mm` 用于分段边界位置、弧长反解及长度闭合的数值检查。实际最低加工进给和机器人跟踪位置容差需要各自依据。源轨迹的起点和终点速度均为零。来源：`A_2_lixianjisuan.m` 第 19 行，`pipe_feed_profile.m` 第 618、635–640 行。

## 位置运动的本轮独立核验

本轮实际运行 [独立计算脚本](../../../.scratch/constraint-limits/inspect_limits.py)，用 SciPy `BSpline` 求曲线及前三阶参数导数，结合源进给时间律独立重算 XYZ 速度、加速度和 jerk；用分节点数值积分重算弧长。MAT 文件由 SciPy 和 `pymatreader` 读取，没有执行 MATLAB，也没有修改源 ZIP。

数据包含 4756 个位置点、4 个运动段、总时长 9.51 s；独立弧长积分为 478.791363842698 mm。A3 与 A4 中的位置、速度、加速度、jerk、参数、时间和进给数组逐元素相同。位置、速度、加速度和 jerk 的重算最大差分别为 `1.74e-13 mm`、`4.66e-12 mm/s`、`4.28e-10 mm/s²`、`6.07e-8 mm/s³`。来源：[本轮数值结果](../../../.scratch/constraint-limits/limits-analysis.json)。

| 采样指标 | 本轮独立结果 | 对应限制 |
|---|---:|---:|
| 最大速度 | 57.899189 mm/s | `F=60 mm/s` |
| 最大切向加速度 | 99.938768 mm/s² | `At=100 mm/s²` |
| 最大法向加速度 | 288.287289 mm/s² | `An=300 mm/s²` |
| 最大 XYZ 加速度模长 | 302.135512 mm/s² | 独立记录；高于单独的法向分量 |
| 最大进给时间律 jerk | 2037.853187 mm/s³ | `Jt=16400 mm/s³` |
| 最大 XYZ jerk 法向投影模长 | 2470.568637 mm/s³ | `Jn=16400 mm/s³` |
| 最大 `κ²v³` | 3031.314644 mm/s³ | `Jn=16400 mm/s³` |
| 最大 XYZ jerk 模长 | 3361.116859 mm/s³ | 独立记录 |
| 最大弦误差（每段内七个检查位置） | 0.000144019 mm | `delta=0.001 mm` |

上述七项有对应源限值的采样检查全部通过；位置及前三阶时间导数的一致性检查也通过。源 `validation.schedule.position` 另存更密集位置及阶段检查的峰值，其中法向加速度为 288.350629 mm/s²、法向 jerk 为 2484.030931 mm/s³、`κ²v³` 为 3032.646743 mm/s³。本轮读取了这些保存值，未重新运行 MATLAB 的完整规划与连续极值检查。采样核验、源保存的检查结果与机器人执行能力保持各自的证据范围。

## 姿态、最低进给与任务误差

最新源程序采用位置三维与平滑执行轴二维，完整滚转为名义参考。源工具轴为工具坐标 `+Z`，`R_WT` 将工具坐标映射到工件坐标；仓库既有任务使用工具 `+X`，接入时需要显式说明轴变换。`progress=u`，`arc_mm` 为独立的弧长字段。来源：`zitai_yilai/pipe_pose_reference.m` 第 32–39 行、`pipe_pose_export.m` 第 10–30 行和 `zitai_gouzao/运行说明.txt` 第 5 节。

`normal_raw_W` 为外法向，`tool_axis_raw_W=-normal_raw_W` 为原始内法向，`tool_axis_ref_W` 为平滑执行轴。本轮重新计算全部采样点两条轴的夹角，最大值为 0.762096944°；源极值检查保存值为 0.762101317°。机器人相对平滑执行轴的跟踪误差与执行轴相对原始内法向的差异分别检查。来源：源运行说明第 5 节、`axis_smoothing_error` 与 `validation.geometry.fidelity.max_rad`、本轮独立结果。

| 源配置字段 | 当前源值 | 可确认的状态 |
|---|---|---|
| `minimum_cruise_speed_mm_s` | 空数组 | 未配置最低巡航进给 |
| `hard_axis_error_rad` | 空数组 | 未配置执行轴相对原始目标轴的工艺误差上限 |
| `soft_axis_error_rad` | 空数组 | 未配置相应误差目标 |
| `omega_max_rad_s` | 空数组 | 未配置完整空间角速度上限 |
| `alpha_max_rad_s2` | 空数组 | 未配置完整空间角加速度上限 |

来源：`zitai_yilai/pipe_pose_options.m` 第 2–15 行，源 `pose_reference.mat.reference.config`；`pipe_pose_schedule.m` 第 23–40 行仅在这些字段有值时加入相应限制。完整空间角 jerk 已计算，源配置没有提供对应上限。

按源计划运动时，全部采样点的完整空间角速度、角加速度、角 jerk 模长最大值分别为 84.506506 °/s、446.754060 °/s²、4908.221979 °/s³。这些数值描述该条参考轨迹的运动需求，不能直接替代关节能力或工具工艺上限。来源：源 `omega_W`、`alpha_W`、`angular_jerk_W` 的本轮独立模长计算。

当前项目[跟踪评价](../../tracking_evaluation.md)另有历史模型评价要求：位置 cross-track RMS ≤0.15 mm、p95 ≤0.5 mm、max ≤2 mm；相对给定执行轴的真实夹角 RMS ≤0.05°、max ≤0.5°。这些是跟踪评价指标，源 `position_tol` 与轴平滑误差分别保留原用途。若需要评价工具相对原始内法向的总误差，还需要独立设定相应判据。

## 插补周期与运行周期

`Ts=2 ms` 是源插补参数。预测更新、实时控制、关节执行及状态反馈的周期分别由其接口与实际验证确定。更改采样或插值方式后，原始弦误差检查的结果需要重新验证。

作为数值检查，采样点中最大曲率为约 `0.383518 mm⁻¹`，对应局部曲率半径约 2.607441 mm。采用源程序的局部圆弧弦误差公式

\[
v_{chord}=\frac{2}{T}\sqrt{2\rho\,\delta-\delta^2}
\]

并保持 `delta=0.001 mm` 时，该局部半径下的速度上界在 `T=10 ms` 时约为 14.441442 mm/s，在 `T=20 ms` 时约为 7.220721 mm/s。该计算仅说明弦误差与连线时间间隔的关系，尚未证明实际执行运动的误差。连续 NURBS 求值、关节运动生成及实际 TCP 路径需要按各自表达进行验证。来源：源 `A_2_lixianjisuan.m` 第 63–64 行及本轮数值结果。

## 当前可采用的值与待确定项目

| 对象 | 数值与采用条件 |
|---|---|
| 源路径研究上限 | `F=60`、`At=100`、`An=300`、`Jt=Jn=16400`，单位分别为 mm/s、mm/s²、mm/s³ |
| 源插补 | `delta=0.001 mm`、`Ts=0.002 s`，更改执行表达后重新检查 |
| 当前机构位置范围 | 使用本页列出的 URDF 边界，保留模型身份和控制余量 |
| 转动轴额定速度资料 | J2–J6 为 240 °/s，J7–J9 为 300 °/s，均为 36 V 资料条件 |
| 当前程序的转动轴速度配置 | 额定速度乘 `0.70`，得到 168 °/s 与 210 °/s；本轮研究速度尚未写入运行配置 |
| 已确认的研究速度上限 | J2–J6：120 °/s，J7–J9：150 °/s；用户确认的人为研究设置，尚无对应负载能力验证 |
| J1 速度 | 36 V 额定条件下为 `4.166666667/r mm/s`；额外传动比 `r` 尚待确认 |
| 关节加速度与 jerk | 研究建议 J2–J6：120 °/s²、600 °/s³；J7–J9：150 °/s²、750 °/s³；已检查 Ruckig 变速曲线，厂家负载能力仍待取得 |
| 最低巡航进给与新增工艺容差 | 源程序未配置，当前研究不把几何扫描值或数值检查精度改作工艺要求 |
| 路径与工具轴 | TCP 三维位置与给定平滑执行轴二维共同跟随，绕工具轴转动自由；最新源路径尚待完成整臂接入与验证 |
| 跟踪精度 | 现有评价要求为位置 RMS ≤0.15 mm、p95 ≤0.5 mm、max ≤2 mm；执行轴夹角 RMS ≤0.05°、max ≤0.5°；本轮尚未全部确认为最新路径的统一判据 |
| 碰撞间距 | 现有 S0/S1 场景为外部间距 30 mm、自碰撞间距 10 mm；本轮尚未确认全项目共用数值 |
| 运动与停止 | 关节边界、环境、工件、工作台和自碰撞检查覆盖实际运动与完整停车过程，结合剩余行程及多轴同步计算 |
| 奇异性 | 现有控制内核具有相关 CBF；当前检查使用位置 Jacobian 的奇异值乘积，本轮尚未新设阈值或确认五维任务的相应判据 |
| 执行与时间 | 检查反馈有效性、计划与命令有效期、计算期限，以及实际采用运动与参考推进的一致性；各周期和期限按正式任务另行核定 |

现有场景数值来源为 [`static_experiment_s0.yaml`](../../../config/static_experiment_s0.yaml)和 [`static_experiment_s1.yaml`](../../../config/static_experiment_s1.yaml)；跟踪评价来源为[跟踪评价说明](../../tracking_evaluation.md)；奇异性检查来源为 [`oscbf_velocity_config.py`](../../../portable_oscbf/work/oscbf_velocity_config.py)的 `h_2`；完整研究要求见[总体项目规格](../../specs/predictive-coordination-control.md)。这些入口保留各自配置和验证范围。

末端空间路径和电机参数还需要通过机构运动学共同判断可执行进给。源 A4 输出的说明明确其尚未验证机器人 IK 或关节约束；关节速度、加速度和 jerk 需求会随具体构型及冗余运动变化。本轮没有执行源路径的整臂闭环、实际作业或硬件运动，因此不报告完整机器人任务通过。

本轮 Python 环境为 3.13.12，NumPy 2.4.6、SciPy 1.17.1、pymatreader 1.3.2、h5py 3.16.0。脚本、解压来源副本和 JSON 结果位于 Git 忽略的 `.scratch/constraint-limits/`；本页保存其主要数值、原始文件身份与验证范围。厂家资料目录同样为本机保留目录，跨机器复算需要传输对应原文件。
