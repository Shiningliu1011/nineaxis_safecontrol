# 标定所需关节与模型接口核查

核查日期：2026-10-26。源码基准：`6a9705260f12db502eef23867ddf0c60febb8aad`，并读取当前工作目录中的文件。范围为关节身份、编码器单位换算、名义运动学和反馈接口；当前没有用户提供的激光跟踪仪测量数据或精度数值。

## 关节与单位

项目公共关节顺序为 `J1, J2, J3, J4, J5, J6, J7, J8, J9`。J1 是 `prismatic`，坐标单位为 m，沿 `base_link` 的局部 Z 方向运动；J2–J9 是 `revolute`，坐标单位为 rad。URDF 解析及本次离线检查均确认这一组成。关节限位属于名义模型数据，现场采集可用范围仍需要依据实物确认。

证据：[robot_spec.py:9](../../../../src/robot_safecontrol_moveit/robot_spec.py#L9)、[ninezzhou.urdf:87](../../../../models/ninezzhou/urdf/ninezzhou.urdf#L87)、[kinematics_data.py:7](../../../../portable_oscbf/work/kinematics_data.py#L7)、[kinematics_data.py:21](../../../../portable_oscbf/work/kinematics_data.py#L21)。

`UnitConverter` 的全部编码器输入，以及全部 `zero_offset_deg`，均使用编码器输出轴 degree。每个关节的换算为：

\[
q_i=s_i(r_i-z_i)k_i,
\qquad r_i=s_iq_i/k_i+z_i.
\]

其中 `s_i = sign`，`r_i = reading_deg`，`z_i = zero_offset_deg`；J2–J9 的 `k_i = π/180` rad/degree。J1 的换算系数为：

\[
k_1=\frac{\texttt{lead\_mm\_per\_rev}}
{1000\times\texttt{ratio\_motor\_rev\_per\_lead\_rev}\times360}
\quad\text{m/degree}.
\]

`ratio_motor_rev_per_lead_rev` 表示电机输出轴转数除以丝杆转数。J1 先减去 degree 单位的零位参数，再进行传动换算。`TransmissionSpec.efficiency` 不参与位置换算。

证据：[unit_conversion.py:83](../../../../src/robot_safecontrol_moveit/unit_conversion.py#L83)、[unit_conversion.py:156](../../../../src/robot_safecontrol_moveit/unit_conversion.py#L156)、[unit_conversion.py:165](../../../../src/robot_safecontrol_moveit/unit_conversion.py#L165)。

当前 `hardware_joint_zero.yaml` 记录了导程 5 mm/rev，同时注明传动比例、效率、行程、正方向及回零方式待测。文件中的九组 `sign: 1` 和 `zero_offset_deg: 0.0` 均标为待验证。代码没有从这条注释自动创建有效的 `TransmissionSpec`；未传入该对象时，J1 换算会抛出 `J1TransmissionMissingError`。

证据：[hardware_joint_zero.yaml:8](../../../../config/hardware_joint_zero.yaml#L8)、[unit_conversion.py:146](../../../../src/robot_safecontrol_moveit/unit_conversion.py#L146)。导程记录属于仓库已有资料，本次没有重新进行物理测量。

## 求解结果与零位参数的符号

若离线求解器明确定义 `q_corrected = q_old + Δq`，并保持编码器读数、方向及传动系数不变，则：

\[
z_{\mathrm{new}}=z_{\mathrm{old}}-\frac{s\,\Delta q}{k}.
\]

旋转关节为 `new_zero_offset_deg = old_zero_offset_deg - sign * Δq * 180 / π`；J1 为 `new_zero_offset_deg = old_zero_offset_deg - sign * Δq / k1`。这里 J1 的 `Δq` 使用 m，其他关节使用 rad。

这一结果由 `UnitConverter.encoder_to_joint` 的当前实现直接移项得到。求解结果应同时保存 `Δq` 的定义、旧参数、候选新参数和换算系数，保证结果可复核。若直接以 `zero_offset_deg` 作为待求参数，输出应声明其为绝对参数值。

证据：[unit_conversion.py:165](../../../../src/robot_safecontrol_moveit/unit_conversion.py#L165)。本次没有生成实物零位参数。

## 名义 URDF 与 FK 来源

几何来源为 `models/ninezzhou/urdf/ninezzhou.urdf`。`generate_kinematics_data.py` 使用 `urdf_parser_py` 读取该文件，保留关节 origin、axis、类型与限位，将 `tool0` 命名为 `ee_link`，并增加一个与 `base_link` 重合的 `world` 坐标系。`NineaxisKinematics` 从生成的 `JOINT_CHAIN` 计算零位变换及 POE 螺旋轴；该类使用的 `world` 在这一模型中与 `base_link` 重合。

证据：[generate_kinematics_data.py:9](../../../../portable_oscbf/scripts/generate_kinematics_data.py#L9)、[generate_kinematics_data.py:62](../../../../portable_oscbf/scripts/generate_kinematics_data.py#L62)、[generate_kinematics_data.py:110](../../../../portable_oscbf/scripts/generate_kinematics_data.py#L110)、[nineaxis_kinematics.py:191](../../../../portable_oscbf/work/nineaxis_kinematics.py#L191)。

可供离线标定使用的纯计算接口如下。输入 `q` 必须按 J1–J9 排列，并使用上述 m/rad 单位。

| 接口 | 输入与输出 | 使用条件 |
|---|---|---|
| `UnitConverter.encoder_to_joint` / `encoder_to_joints` | 编码器输出轴 degree → 关节坐标 | 完整核实关节身份、方向、零位参数与 J1 传动 |
| `UnitConverter.joint_to_encoder` / `joints_to_encoder` | 关节坐标 → 编码器 degree | 使用同一份参数快照 |
| `NineaxisKinematics.forward_kinematics(q)` | 九个关节坐标 → 各 link 的 4×4 变换 | `ee_link` 对应 `tool0`；`world` 与 `base_link` 重合 |
| `NineaxisKinematics.ee_pose(q)` | 九个关节坐标 → 位置 `(3,)`、旋转矩阵 `(3,3)` | 名义 tool0 位姿 |
| `NineaxisKinematics.compute_position_jacobian(q)` | 九个关节坐标 → 3×9 位置 Jacobian | 对应 tool0 原点 |
| `NineaxisKinematics.point_jacobian(q, link_idx, point_world)` | 关节坐标、link 索引、模型坐标系中的点 → 3×9 Jacobian | 最末端刚性安装点使用 `link_idx=9` |

证据：[unit_conversion.py:183](../../../../src/robot_safecontrol_moveit/unit_conversion.py#L183)、[nineaxis_kinematics.py:294](../../../../portable_oscbf/work/nineaxis_kinematics.py#L294)、[nineaxis_kinematics.py:318](../../../../portable_oscbf/work/nineaxis_kinematics.py#L318)、[nineaxis_kinematics.py:375](../../../../portable_oscbf/work/nineaxis_kinematics.py#L375)。

这些计算接口不需要启动 ROS、MoveIt 或仿真。`NineaxisKinematics` 构造过程读取执行器 YAML 以取得速度与加速度资料，其 FK 几何仍来自 `JOINT_CHAIN`。本次计算只调用 FK 与位置 Jacobian。

证据：[nineaxis_kinematics.py:191](../../../../portable_oscbf/work/nineaxis_kinematics.py#L191)、[actuator_limits.py:107](../../../../portable_oscbf/work/actuator_limits.py#L107)。

`src/aeb_rrtstar/robot_model.py` 中的 FK 明确用于独立 benchmark 的近似距离检查：实现只累计 `_LINK_OFFSETS` 和各关节 Z 旋转。离线标定的名义几何应使用已核对 URDF 的 `NineaxisKinematics` 接口。

证据：[robot_model.py:98](../../../../src/aeb_rrtstar/robot_model.py#L98)、[robot_model.py:149](../../../../src/aeb_rrtstar/robot_model.py#L149)。

## J9 与 tool0 的名义几何

URDF 定义 J9 的局部轴向为 `[0, 0, 1]`，其子 link 为 `Link9`；`tool0_fixed` 把 tool0 放在 Link9 坐标系的 `[0.235, 0, 0]` m，固定旋转为零。因此 tool0 原点到 J9 轴线的垂直距离为 0.235 m。

证据：[ninezzhou.urdf:551](../../../../models/ninezzhou/urdf/ninezzhou.urdf#L551)、[ninezzhou.urdf:569](../../../../models/ninezzhou/urdf/ninezzhou.urdf#L569)。URDF 注释说明 0.235 m 来自已有规划工作点的定义；该数值的来源没有提供本次靶球安装几何的实测证明。

本次离线计算结果：

- 零位 tool0 在 `base_link` 中的位置为 `[-0.0000063546528432, 0.34300218187920345, 1.3869999999796934]` m。
- tool0 对 J9 的位置导数模长为 `0.23499999999999974` m/rad。
- 仅将名义 J9 坐标增加 0.1 rad，tool0 的位置变化长度为 `0.023490209557218818` m，与 `2 × 0.235 × sin(0.05)` 一致。

这些结果证明当前模型中 tool0 位置随 J9 变化。实际靶球的响应取决于其球心在末端刚体坐标系中的安装位置。离线模型应使用已知或参与求解的球心安装坐标，不能直接把球心等同于 tool0 原点。九个零位参数能否同时确定，还需要完整测量模型与参数可观测性检查。

计算来源：[verify_project_interfaces.py](verify_project_interfaces.py)。全部数值均为名义几何结果。

## 反馈格式与当前采集条件

纯解码函数 `decode_feedback(frame_id, data)` 返回 `MotorFeedback`，字段为 `node_id`、`pos_deg`、`vel_rpm`、`torque_nm`、`traj_done`、`axis_error`。位置采用 float32 degree；速度和转矩分别为有符号 16 位数乘以 0.01。该数据对象自身没有时间戳。

证据：[drempower_can.py:74](../../../../src/robot_safecontrol_moveit/drempower_can.py#L74)、[drempower_can.py:188](../../../../src/robot_safecontrol_moveit/drempower_can.py#L188)。

`ReceivedFrame` 保留底层 `frame_id`、`data`、`timestamp_s`。`PythonCANBackend.receive` 使用收到的 `python-can Message.timestamp`；`poll_node_state` 校验帧来源与时间，并把同一时间保存为 `CANNodeState.stamp_s`。模块说明明确将接收时间定义为 transport wall-clock；实际时间检查与 `time.time()` 比较，因此接口使用主机墙钟体系。读取超时通过 `time.monotonic()` 管理，两个时间体系需要分别记录。当前接口没有电机设备侧的采样时间或采样序号，不能据此声称九轴同时采样。

证据：[python_can_backend.py:1](../../../../src/robot_safecontrol_moveit/python_can_backend.py#L1)、[python_can_backend.py:14](../../../../src/robot_safecontrol_moveit/python_can_backend.py#L14)、[python_can_backend.py:76](../../../../src/robot_safecontrol_moveit/python_can_backend.py#L76)、[socketcan_backend.py:193](../../../../src/robot_safecontrol_moveit/socketcan_backend.py#L193)。

现有 `hardware_probe` 会读取 `current_state`、`axis_error`、`output_position`、`output_velocity`，为每个属性保存 `value` 与 `rx_wall_time_s`，并明确输出 `device_sample_age_known: false`。这是可参考的通信诊断格式，采集方案仍需定义测量组、时间窗口、静止判定及激光跟踪仪记录的配对方法。

证据：[hardware_probe.py:19](../../../../src/robot_safecontrol_moveit/hardware_probe.py#L19)、[hardware_probe.py:94](../../../../src/robot_safecontrol_moveit/hardware_probe.py#L94)。本次没有调用 CAN 或串口接口。

代码中的时间限制也需要与现场条件区分：`hardware_probe` 默认每轮属性读取等待上限为 0.2 s；`poll_node_state` 默认接收等待为 0.01 s，反馈有效时间上限为 0.2 s。这些默认值来自通信实现，尚未经过本次标定采集的设备验证。跟踪仪测量期间各关节保持静止的时间范围及允许变化量，仍需在采集方案中确定。

证据：[hardware_probe.py:94](../../../../src/robot_safecontrol_moveit/hardware_probe.py#L94)、[hardware_probe.py:149](../../../../src/robot_safecontrol_moveit/hardware_probe.py#L149)、[socketcan_backend.py:185](../../../../src/robot_safecontrol_moveit/socketcan_backend.py#L185)。

当前 `HardwareBridge` 的 `live` 模式直接拒绝启动；`sim`、`shadow` 不创建 CAN 总线、轮询定时器或状态发布器。`/mujoco_joint_states` 的项目生产者 `OscbfPlant` 从 `SCurveDriverSimulator` 产生状态。因此，现场标定需要取得明确来源的真实编码器记录。

证据：[hardware_bridge.py:67](../../../../src/robot_safecontrol_moveit/hardware_bridge.py#L67)、[hardware_bridge.py:96](../../../../src/robot_safecontrol_moveit/hardware_bridge.py#L96)、[ros_conventions.py:15](../../../../src/robot_safecontrol_moveit/ros_conventions.py#L15)、[oscbf_plant.py:44](../../../../src/robot_safecontrol_moveit/oscbf_plant.py#L44)、[oscbf_plant.py:190](../../../../src/robot_safecontrol_moveit/oscbf_plant.py#L190)。

2026-10-16 的记录显示九个电机节点在 500 kbps 下完成两次静止属性通信检查，各为 120 轮、1080 次有效回复。本次读取了两份原始结果文件，确认其 `complete: true` 和相应计数。该次连接使用厂家串口协议；节点 1–9 与实际 J1–J9 的安装对应关系仍待核对，`can0` 也尚未作为该设备的连接入口验证。

证据：[通信记录:17](../../../../docs/drempower_communication_20261016.md#L17)、[通信记录:19](../../../../docs/drempower_communication_20261016.md#L19)、[第一份结果](../../../../.scratch/motor_check_20261016/stability_20261016T122305_449305/summary.json)、[第二份结果](../../../../.scratch/motor_check_20261016/stability_20261016T122657_839629/summary.json)、[drempower.yaml:12](../../../../config/drempower.yaml#L12)。这属于历史通信证据，未验证当前连接或本次测量条件。

## 离线数据接口需要保存的内容

依据以上接口，采集与求解之间至少需要明确以下记录：

- 每组测量的 `sample_id`、测量顺序、靶球身份和安装位置身份，以及跟踪仪坐标系与单位。
- 每个节点的原始读数 `pos_deg`、节点身份、关节身份、原始 CAN 帧或厂家记录、接收时间及其时间体系；还需保存可用的速度、转矩、状态位与异常信息。
- 同一测量组内各节点读数的时间跨度、静止判定依据、跟踪仪测量时间及配对规则。数值阈值待设备与验收要求确定。
- 当次使用的 URDF、参数文件、节点映射、J1 传动参数与 `sign`/`zero_offset_deg` 快照，以及由其计算的 `q`。
- 明确标识模型计算值、原始实测值和候选标定结果，保留各自来源。

现有 `JointCalibrationTable.for_joint` 在缺少关节时会提供默认 `sign=1`、`zero_offset_deg=0`。离线采集文件的读取规则应主动要求九个关节资料完整，并保留每个参数的确认状态。原始 `pos_deg` 应持续保留，以便参数调整后重新计算 `q`。

证据：[unit_conversion.py:121](../../../../src/robot_safecontrol_moveit/unit_conversion.py#L121)、[unit_conversion.py:183](../../../../src/robot_safecontrol_moveit/unit_conversion.py#L183)。这一节属于准备方案的接口要求，尚未实现新的采集器或求解器。

`scripts/calibrate_zero.py` 执行的是机械零位标记流程：打开 SocketCAN、使能电机、执行 `set_zero`、发送方向检查命令并写入参数 YAML。激光跟踪仪求解与数据配对需要独立定义。准备阶段应保留当前编码器零位的来源，现场执行 `set_zero` 时必须建立新的数据与参数版本关系。

证据：[calibrate_zero.py:77](../../../../scripts/calibrate_zero.py#L77)、[calibrate_zero.py:97](../../../../scripts/calibrate_zero.py#L97)、[calibrate_zero.py:121](../../../../scripts/calibrate_zero.py#L121)。本次没有运行该脚本。

## 实际验证

运行目录为 `/home/lsn/robot/robot_safecontrol`。环境为 Python 3.13.12、NumPy 2.4.6、SciPy 1.17.1；`urdf_parser_py` 来自已配置的 ROS Humble Python 路径。验证脚本通过成熟 URDF 解析库读取文件，以 SciPy 旋转矩阵进行名义 FK 计算，并直接调用项目 FK 接口核对。

1. `PYTHONDONTWRITEBYTECODE=1 python3 portable_oscbf/scripts/generate_kinematics_data.py --check`：退出码 0，生成数据与当前 URDF 一致。
2. `PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$PWD/src:$PWD/portable_oscbf${PYTHONPATH:+:$PYTHONPATH}" python3 .trellis/tasks/10-26-laser-tracker-calibration-preparation/research/verify_project_interfaces.py`：退出码 0。检查三个合法名义关节配置，每个配置核对 11 个 link 变换；最大矩阵元素绝对差为 `8.881784197001252e-16`。J9 几何关系及 Jacobian 检查通过。
3. `PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q -p no:cacheprovider tests/test_unit_conversion.py`：退出码 0，`13 passed in 0.08s`。检查内容包括节点映射、旋转单位、方向与零位参数、J1 传动换算及缺少传动时拒绝换算。

全部运行均为隔离的离线数值检查。未进行实物运动、编码器读取、激光测量、零位参数写入或精度验收；名义 FK 一致性不能证明实物几何精度。
