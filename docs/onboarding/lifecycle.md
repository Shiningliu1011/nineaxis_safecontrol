## Request Lifecycle（闭环数据流）

1. `run_demo.sh` 启用被控对象并设置随机起始位姿；`oscbf_plant.py` 持续发布仿真状态。
2. `transition_planning_server.py` 调用 MoveIt/AEB-RRT* 与 FCL 规划过渡，经 Ruckig
   平滑并回放命令；回放后等待被控对象收敛，再交接给 OSCBF 控制器。
3. `oscbf_controller.py` 订阅状态，经 `portable_oscbf/work` 的 JAX facade 计算并
   发布 `/oscbf_command`；被控对象积分后继续发布 `/mujoco_joint_states`。
4. `mujoco_viewer_with_cylinder.py` 订阅状态驱动显示；它不拥有控制状态 publisher。
5. `tracking_evaluator.py` 可订阅状态与命令计算跟踪指标；查看器、过渡服务器和
   控制器共用 `oscbf_trajectory.py` 的轨迹变换。

**当前 QoS**：canonical [ros_conventions.py](../../src/robot_safecontrol_moveit/ros_conventions.py)
的 `state_stream_qos()` 是 KEEP_LAST / depth 20 / BEST_EFFORT / VOLATILE。
被控对象的状态发布和命令订阅、控制器的状态订阅及 shadow bridge 的命令订阅使用它。
viewer、过渡服务器的状态订阅、过渡回放发布和控制器的命令发布仍使用
`qos_profile_sensor_data`（depth 5 / BEST_EFFORT / VOLATILE）。深度差异仍存在，
这里记录当前实体，不代表全面 QoS 已统一；不存在 viewer 发布 transient-local 状态的链路。
