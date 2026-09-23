## Request Lifecycle（闭环数据流）

1. `run_demo.sh` 启用被控对象并设置随机起始位姿；`oscbf_plant.py` 持续发布仿真状态。
2. `transition_planning_server.py` 调用 MoveIt/AEB-RRT* 与 FCL 规划过渡，经 Ruckig
   平滑并回放命令；回放后等待被控对象收敛，再交接给 OSCBF 控制器。
3. `oscbf_controller.py` 订阅状态，经 `portable_oscbf/work` 的 JAX facade 计算并
   发布 `/oscbf_command`；被控对象积分后继续发布 `/mujoco_joint_states`。
4. `mujoco_viewer_with_cylinder.py` 订阅状态驱动显示；它不拥有控制状态 publisher。
5. `tracking_evaluator.py` 可订阅状态与命令计算跟踪指标；查看器、过渡服务器和
   控制器共用 `oscbf_trajectory.py` 的轨迹变换。

**当前 QoS**：[ros_conventions.py](../../src/robot_safecontrol_moveit/ros_conventions.py)
分别提供 `state_stream_qos()`（KEEP_LAST / depth 20 / BEST_EFFORT / VOLATILE）和
`command_stream_qos()`（KEEP_LAST / depth 5 / BEST_EFFORT / VOLATILE）。被控对象与
过渡回放的状态发布，以及控制器、过渡服务器、查看器和感知桥的关节状态订阅使用
状态流设置。控制器与过渡回放的指令发布，以及被控对象和 shadow bridge 的指令订阅
使用指令流设置。感知和 LiDAR 话题继续使用各自的 QoS。两条流的测量数据与选择依据见
[QoS 时延验证记录](../validation/qos-latency-2026-09-23.md)。
