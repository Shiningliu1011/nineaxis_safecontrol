## Architecture

**ROS 2 工作区 + 嵌套包**：根目录是 `robot_safecontrol_moveit`（ament_python），
其 `src/aeb_rrtstar_ompl` 是嵌套的 ament_cmake 包（MoveIt2 OMPL 插件），普通
`colcon build` 发现不了它，必须用 `build_aeb_moveit.sh` 指定 `--base-paths`。

**仿真闭环（`run_demo.sh` 配置）**：脚本启用 `oscbf_plant`，它订阅
`/oscbf_command`，通过带加速度/jerk 限幅的执行器仿真积分并持续发布
`/mujoco_joint_states`。`oscbf_controller` 订阅该状态流，运行 JAX 内核后
发布命令。viewer 只订阅状态做 MuJoCo 显示，不发布关节状态。
过渡服务器经 `trajectory_execution.py` 将回放发送到 `/transition_replay_viz`，
并向命令流发送过渡命令；回放及收敛完成后交接给控制器。

**默认 launch 与 demo 的差别**：直接启动最终 launch 时 `start_oscbf_plant=false`，
`transition_replay_topic=/mujoco_joint_states`；过渡回放会向此状态话题发布，
但没有被控对象持续积分形成上述闭环。不能把 demo 的状态 ownership 套到所有配置。
当前 containment 的 hardware bridge 在 sim/shadow 都不发布硬件状态。

**自主过渡流程**：`transition_planning_server` 监听开始信号后，调用 move_group
的 AEB-RRT* 规划器从随机位姿规划无碰撞过渡到轨迹起点，经 Ruckig 平滑后回放；
回放结束 → OSCBF 控制器接管（`oscbf_wait_for_start`）。

**真机目标与当前实现**：未来目标是 command → 安全网关 → CAN → 真实反馈 →
硬件状态流。当前 `hardware_bridge.py` 未接通该链路：sim 不创建控制订阅、
polling timer 或状态 publisher；shadow 订阅命令并记录请求/拒绝，不创建 CAN
backend、不收发 CAN；live 在创建硬件控制实体之前无条件失败。
无真实反馈时 `feedback_ok`、`watchdog_ok` 均为 false，acknowledge 不能制造健康状态。
final launch 在启动任何节点前拒绝 `hardware_mode=live`；`hardware_bridge` 自身也保留独立拒绝，保护 direct node 入口。未来解除 live containment 时，两层必须独立审查。
剩余实现及真机准入见 [GitHub #13](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13)。

**感知管线**：`perception_bridge` 接收点云 → `obstacle_extractor` 聚类拟合球/
圆柱几何 → 发布到 `/perception/tracks` → `oscbf_controller` 经 `obs_*` 接口注入
控制内核做动态避障。

**标定记录（外参唯一真源）**：感知外参只来自 `config/sensor_extrinsics.yaml`
（经 ament share 路径加载；逐源分节含矩阵与 provenance）。`perception_bridge` 启动时校验
「记录可读 / 矩阵是合法 SE(3) / 帧绑定与配置一致 / 每个被启用源过标定门」，任一条不满足即
**拒绝启动**（退出码 3，不发布任何诊断）；运行期每帧校验 `header.frame_id`，不符即丢弃。
旧参数 `use_tf` / `camera_to_world_static` / `lidar_to_world_static` 只保留声明作探测器
（非默认值即拒绝启动），**没有 TF 输入路径，也没有 identity 兜底**。身份与准入结论发布在
`/perception/calibration_status`（`DiagnosticArray`，1 Hz 心跳 + 启动即发一份）。
心跳使用单调时钟，`use_sim_time=true` 且 `/clock` 停止时仍持续发布；消息时间戳
仍采用 ROS 时间，消费端不得用该时间戳判断报告新鲜度。记录规范化中的映射键必须为
字符串（禁止把数字键转为字符串后覆盖同名键）；超出浮点范围的数值和非法 YAML 日期
按标定错误拒绝启动，不绕过退出码 3 契约。修复与复现见
[OFF-01 交付记录](../planning/oscbf-reuse/handoffs/45-calibration-record.md)。
未标定几何只允许在部署 profile 显式逐源声明 `allow_uncalibrated_debug` 时启动，
且不构成准入。契约见 ADR 0004 与 ADR 0009。

**坐标系**：URDF 为 Y-up，MuJoCo 为 Z-up，程序通过 `display_frame` body 的
euler 旋转自动转换。圆柱轴心拟合口径在轨迹生成端、过渡端、控制器端必须一致
（默认最小二乘圆拟合轨迹自动求轴心）。
