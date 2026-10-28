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

多轴真实执行入口仍关闭，`--execute` 会检查运动锁定并拒绝创建真实 I/O。完整的现场执行还需要多轴反馈时间基准、同步与故障处理、碰撞路径、停车能力及断电恢复验证。现有低速检查参数保持原值：J1 最高 0.25 mm/s，旋转轴指令使用可编码的 0.41 rpm，即 2.46°/s。按当前 J1 参数移动 50 mm 需要约 205 秒，不能据此承诺快速到位。提高运动速度需要现场测量依据及新的明确动作授权。

本次加入了表格输入、模型检查和离线流程，传感器外参求解与多轴真机执行尚未完成。现场每组测量应保存目标关节值、实际反馈、模型与零位身份、时间基准、传感器原始文件及对应的测量编号。

后续实验研究直接在 OSCBF 求解中加入速度、加速度和加加速度约束，使用共同模型与限制来源；这项研究独立于标定的绝对关节定位流程。

组间 `ready` 阶段继续接收反馈并检查上一组保持位置，人工等待不清除时间或位置历史。开始下一组时重新累计该组的到位持续时间。没有持续反馈时，原有 0.2 秒间隔检查仍会拒绝继续。
