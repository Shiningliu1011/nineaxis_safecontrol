# 标定姿态表

相机标定和激光跟踪仪标定分别建立会话。两次标定可以使用同一份关节姿态表，各自保存执行反馈及采集文件，不要求同时采集两种设备。

姿态表每行给出一组完整的绝对关节目标，列顺序为 `pose_id,J1_mm,J2_deg,J3_deg,J4_deg,J5_deg,J6_deg,J7_deg,J8_deg,J9_deg`。J1 使用新坐标 0–500 mm；J2–J9 使用模型角度，方向和设备零位继续由已有标定换算。`pose_id` 必须非空且唯一。可从 [示例 CSV](../config/calibration_joint_table.example.csv) 复制格式；示例数值仅用于输入检查，尚未检查环境碰撞。

当前主机可以检查整表、导出 SI 目标和名义 TCP：

```bash
source install/setup.bash
python3 -m robot_safecontrol_moveit.joint_table_cli \
  --root "$PWD" --input config/calibration_joint_table.example.csv \
  --measurement-kind camera --output .scratch/camera-targets.json
```

激光跟踪仪会话使用 `--measurement-kind laser_tracker` 和独立输出路径。输出文件必须尚不存在。报告保存表格 SHA-256、共同模型身份、关节顺序、项目单位目标和 SI 目标。`targets_validated` 表示输入与模型位置范围通过检查；报告明确记录 `collision_checked=false`、`execution=not_started`、`hardware_executable=false`。

标定采用关节绝对目标定位。每组目标经 J1 传动、各轴方向和零位换算，交给驱动器的 `set_angle(mode=1)`，由驱动器按指定速度和加速度完成运动。各轴可以重叠运动，全部关节持续到位后才允许采集。报告的 `device_commands` 保存九条单节点绝对位置参数、CAN ID 和实际协议载荷，同时记录节点映射、零位与传动配置 SHA-256。当前只导出这些命令，不发送设备。J1 使用 0.25 mm/s、0.05 mm/s²，旋转轴使用既有可编码的 0.41 rpm、0.05 rpm/s；这些参数属于已有低速检查范围。

`JointTableProgress` 提供离线逐组流程：调用 `begin()` 选取当前目标；`observe()` 只接受来源明确的仿真反馈，逐项检查身份、时间、速度、位置连续性和范围。全部九轴连续满足位置及静止条件至少 0.5 秒后，进入 `awaiting_measurement`。采集期间继续观察反馈，姿态变化或反馈中断会拒绝继续。`record_measurement()` 关联本次已经保存的非空文件并记录 SHA-256，之后才允许 `begin()` 选取下一行。`write_report()` 将当前阶段、逐组实际反馈和文件关联保存到新的 JSON 文件。反馈与记录明确标识 `simulation_feedback`，不能作为物理标定证据。

普通 CSV 自动执行的 `--execute` 保留固定拒绝，在创建真实 I/O 前检查运动锁定及资格。其默认低速参数为 J1 0.25 mm/s、旋转轴 0.41 rpm（2.46°/s）。现场明确授权的单组调试由 `joint_table_commissioning` 管理，采用独立的参数来源、目标身份和一次性请求，详见下文。参数依据和完整真实记录见[运动参数与反馈证据](joint_motion_limits_evidence.md)。

`JointTableExecutor` 提供九轴设备观察流程，复用每轴的 `VelocityWindow` 和 `SingleJointMonitor`。每轮必须包含完整 J1–J9，各轴保留独立请求与接收时间。开始运动前连续检查真实来源的反馈；同一组全部关节共同持续满足位置、速度和 `traj_done` 至少 0.5 秒，才进入 `awaiting_measurement`。已经到位的关节继续接受监控。运动、停车、反馈或测量等待超时会锁存 `aborted`，不能自动重试或开始下一组。测量等待期限默认 300 秒，通过 `measurement_timeout_s` 明确设置。

测量请求必须提供当前 `session_id`、`task_id`、`measurement_kind`、`pose_id` 和已经保存的非空采集文件。runtime 在后台计算文件 SHA-256，主线程继续检查九轴反馈；`record_prepared_measurement()` 接受记录前重新核对文件身份与当前反馈时效。直接 `record_measurement()` 仅供离线流程使用。来源身份不符、过期反馈及同一会话重复使用文件内容均拒绝。报告逐组保存目标、实际反馈、运动耗时、项目单位误差、停车条件、拒绝原因及文件 SHA-256；完成的组保存 `completed_s`，后续组拒绝时保留其完成结果。相机和跟踪仪使用各自的任务与会话。

`joint_table_runtime.py` 接通九轴位置发送、全部反馈轮询、测量请求和异常保持观察；`DrEmpowerSerial(position_nodes=...)` 只允许完整九轴的单节点位置模式 1，禁止该连接写入设备配置。普通执行仍保留资格拒绝；单组调试核对当前请求和参数范围。停止请求只使用新取得且可信的位置，保持命令返回和反馈停车观察分别记录。通信失效或停车未确认时报告 `external_stop_required`。

相关 CLI 参数为 `--port`、`--task-id`、`--measurement-dir` 和 `--measurement-timeout-s`。运行记录使用指定输出路径的 JSONL，以及同名 `.frames.json`、`.report.json`。测量请求按行使用 `0001.json`、`0002.json` 等文件，字段为 `session_id,task_id,pose_id,measurement_kind,files`；文件需要完整保存后再交给运行程序读取。当前固定关闭判断保持启用，这些参数不能取得运动许可。

九轴动态 MuJoCo 测试检查同时运动、共同停车、测量等待及下一组推进，报告标记 `simulation_feedback`。完整设备记录回放标记 `recorded_device_feedback`。现场执行采用 `hardware_feedback`；当前尚无九轴真机到位、停车、物理加速度和快速运动验收。报告的 `hardware_qualified`、`physical_stop_independently_verified`、`acceleration_feedback_validated` 保持为 `false`。

首组目标 `[50 mm, 10°, -20°, 20°, 60°, -20°, 30°, -30°, 30°]` 已按本会话明确授权实际发送。采用上位机轨迹参数的 0.75：J1 为 2.0825 mm/s、1.875 mm/s²；J2–J6 为 108°/s；J7–J9 为 149.94°/s；旋转轴加减速度为 135°/s²。运动约 0.5205 秒时，J2 反馈 10.686097°，超出目标 10° 的允许区间，整组中止。随后发送九条可信位置保持请求；全部停车观察未通过，用户已操作急停并报告停止，之后恢复急停。首组未完成，没有标定测量，没有自动重试或下一组。

急停恢复后的只读回读显示 J5 从保持记录约 +18.12° 变为 −79.05°，J6 从约 −15.51° 变为 −59.22°。用户确认驱动失能后关节受重力自行转动，现场没有机械支撑或制动。运动锁定因此增加 `hardware_blocker`，单组入口在创建 I/O 前拒绝，人工 acknowledgement 也不能清除此阻塞。用户另行要求的全部关节回零目标已保存为全零位置表，尚未执行；该请求不包含重写设备零点。当前需要建立并验证能够阻止重力转动的停止条件。

J1 新坐标仍为 0–500 mm，用户设置的设备零位由独立回读确认过，多圈开关 1、零位计数 7733；首组执行前实际设备限位为 −36000°–0°。程序没有改写设备零位或限位。完整阶段记录见[现场证据](joint_motion_limits_evidence.md#首组九轴真机调试与急停恢复)。

用户归零前的 J1 后退约 28 mm 请求已经结束。停车、断电保持及九轴快速到位验收仍未完成；ROS live 与普通 CSV 自动执行保持关闭。历史记录见[后退检查](joint_motion_limits_evidence.md#已授权的-j1-后退检查)。

## 单组调试边界

`python3 -m robot_safecontrol_moveit.joint_table_commissioning` 接受 `--root`、`--input`、`--request`、`--port` 和 `--output`。请求绑定一行完整目标、模型、配置、串口、上位机参数来源、停车噪声原始证据和旧锁定 SHA-256，最长十分钟有效，尝试一次后即消耗。现场人员、独立急停和当前零位确认均明确记录；这些确认不能替代真实反馈或消除硬件阻塞。运动前重新读取设备状态、故障、限位与轨迹参数，原锁定只在共同停车通过后归档并恢复。成功与故障结束均重新锁定。

停车使用五秒完整位置范围、最近位置变化速度、五秒原始速度平均值及 `traj_done` 共同检查，再要求九轴共同持续至少 0.5 秒。位置范围上限为 0.01°，位置速度阈值为 0.02 rpm；原始速度估计的噪声范围来自另一份完整静止记录的单秒时间平均值，并保存原始反馈及来源。运动超速仍独立检查一秒速度和位置变化，不采用停车噪声范围。到位误差继续为 0.1°。J1 的单次 50 mm 检查计入已测 0.01° 位置测量范围，实际目标严格限定 0–500 mm。

调试用途标记 `positioning`，不产生相机或激光跟踪仪测量记录。调试完成状态为 `positioning_completed`，仅表示该组反馈检查完成；`hardware_qualified` 等完整资格字段保持 `false`。当前重力停车阻塞尚未解决，该入口不能运行运动。

后续实验研究直接在 OSCBF 求解中加入速度、加速度和加加速度约束，使用共同模型与限制来源；这项研究独立于标定的绝对关节定位流程。

组间 `ready` 阶段继续接收反馈并检查上一组保持位置，人工等待不清除时间或位置历史。开始下一组时重新累计该组的到位持续时间。没有持续反馈时，原有 0.2 秒间隔检查仍会拒绝继续。
