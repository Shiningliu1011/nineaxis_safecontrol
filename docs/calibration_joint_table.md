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

多轴真实执行入口仍关闭，`--execute` 会检查运动锁定及固定关闭判断，在创建真实 I/O 前拒绝。现有低速检查参数保持原值：J1 最高 0.25 mm/s，旋转轴指令使用可编码的 0.41 rpm，即 2.46°/s。按当前 J1 参数移动 50 mm 需要约 205 秒。提高运动速度需要现场测量依据及新的明确动作授权；参数来源和完整真实回放见[运动参数与反馈证据](joint_motion_limits_evidence.md)。

`JointTableExecutor` 提供九轴设备观察流程，复用每轴的 `VelocityWindow` 和 `SingleJointMonitor`。每轮必须包含完整 J1–J9，各轴保留独立请求与接收时间。开始运动前连续检查真实来源的反馈；同一组全部关节共同持续满足位置、速度和 `traj_done` 至少 0.5 秒，才进入 `awaiting_measurement`。已经到位的关节继续接受监控。运动、停车、反馈或测量等待超时会锁存 `aborted`，不能自动重试或开始下一组。测量等待期限默认 300 秒，通过 `measurement_timeout_s` 明确设置。

测量请求必须提供当前 `session_id`、`task_id`、`measurement_kind`、`pose_id` 和已经保存的非空采集文件。runtime 在后台计算文件 SHA-256，主线程继续检查九轴反馈；`record_prepared_measurement()` 接受记录前重新核对文件身份与当前反馈时效。直接 `record_measurement()` 仅供离线流程使用。来源身份不符、过期反馈及同一会话重复使用文件内容均拒绝。报告逐组保存目标、实际反馈、运动耗时、项目单位误差、停车条件、拒绝原因及文件 SHA-256；完成的组保存 `completed_s`，后续组拒绝时保留其完成结果。相机和跟踪仪使用各自的任务与会话。

`joint_table_runtime.py` 接通九轴位置发送、全部反馈轮询、测量请求和异常保持观察；`DrEmpowerSerial(position_nodes=...)` 只允许完整九轴的单节点位置模式 1，禁止该连接写入设备配置。执行入口与 runtime 均保留资格拒绝。停止请求只使用新取得且可信的位置，保持命令返回和反馈停车观察分别记录。通信失效或停车未确认时报告 `external_stop_required`。

相关 CLI 参数为 `--port`、`--task-id`、`--measurement-dir` 和 `--measurement-timeout-s`。运行记录使用指定输出路径的 JSONL，以及同名 `.frames.json`、`.report.json`。测量请求按行使用 `0001.json`、`0002.json` 等文件，字段为 `session_id,task_id,pose_id,measurement_kind,files`；文件需要完整保存后再交给运行程序读取。当前固定关闭判断保持启用，这些参数不能取得运动许可。

九轴动态 MuJoCo 测试检查同时运动、共同停车、测量等待及下一组推进，报告标记 `simulation_feedback`。完整设备记录回放标记 `recorded_device_feedback`。现场执行采用 `hardware_feedback`；当前尚无九轴真机到位、停车、物理加速度和快速运动验收。报告的 `hardware_qualified`、`physical_stop_independently_verified`、`acceleration_feedback_validated` 保持为 `false`。

本会话收到的首组目标为 `[50 mm, 10°, -20°, 20°, 60°, -20°, 30°, -30°, 30°]`，已完成输入检查和导出，尚未发送。用户测得当前 J1 距原 585 mm 端点 472 mm，按其零位定义推算当前约为 28 mm，零位基准为近似确定，测量不确定度未知。用户另行授权开启 B06-PA-36-G 的多圈功能：J1 开关已回读为 1，保存命令已发送，设备零位、限位及运动锁定保持。启用后输出角度仍接近 0°，不能直接将其作为模型坐标。当前设备坐标对应、停车与断电恢复的证据见[运动参数与反馈证据](joint_motion_limits_evidence.md#当前现场检查与零位)。

后续实验研究直接在 OSCBF 求解中加入速度、加速度和加加速度约束，使用共同模型与限制来源；这项研究独立于标定的绝对关节定位流程。

组间 `ready` 阶段继续接收反馈并检查上一组保持位置，人工等待不清除时间或位置历史。开始下一组时重新累计该组的到位持续时间。没有持续反馈时，原有 0.2 秒间隔检查仍会拒绝继续。
