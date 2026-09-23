## Key Entry Points

- `src/robot_safecontrol_moveit/oscbf_controller.py` — OSCBF 安全控制节点
- `src/robot_safecontrol_moveit/oscbf_plant.py` — jerk 限幅执行器仿真节点
- `src/robot_safecontrol_moveit/transition_planning_server.py` — 过渡规划服务器（薄 ROS 壳）
- `src/robot_safecontrol_moveit/transition_executor.py` — 过渡管线相位机（纯逻辑，无 ROS）
- `src/robot_safecontrol_moveit/mujoco_viewer_with_cylinder.py` — MuJoCo 查看器/仿真
- `src/robot_safecontrol_moveit/oscbf_trajectory.py` — 统一轨迹变换（三端共享）
- `src/robot_safecontrol_moveit/hardware_bridge.py` — containment 入口（sim inert / shadow 记录 / live disabled）
- `src/robot_safecontrol_moveit/perception_bridge.py` — 感知桥接节点
- `src/robot_safecontrol_moveit/obstacle_extractor.py` — 点云→几何障碍物提取
- `src/robot_safecontrol_moveit/tracking_evaluator.py` — 跟踪评价指标
- `src/robot_safecontrol_moveit/drempower_can.py` — DrEmpower CAN 协议编解码
- `src/robot_safecontrol_moveit/socketcan_backend.py` — SocketCAN 抽象与替身测试入口，当前 bridge 不接入
- `portable_oscbf/work/jax_control_facade.py` — JAX 控制内核主机端入口
- `src/aeb_rrtstar_ompl/src/AEBRRTstar.cpp` — C++ AEB-RRT* 插件实现
- `launch/mujoco_transition_final.launch.py` — 完整闭环 launch
- `run_demo.sh` / `build_aeb_moveit.sh` — 一键运行/构建
