## Tech Stack

| Layer | Technology | Version |
|-------|-----------|---------|
| ROS 2 | Humble (rclpy / rclcpp) | Humble |
| Python 包 | robot_safecontrol_moveit (ament_python), setup.py | 0.1.0 |
| 运动规划 | MoveIt 2 + pymoveit2 + OMPL 插件 | — |
| 物理仿真 | MuJoCo | — |
| 控制内核 | JAX 0.6.2 (JIT), qpax 弹性 QP | — |
| 碰撞 | FCL, OBB 包络, DCOL, 17-球模型 | — |
| C++ 插件 | aeb_rrtstar_ompl (ament_cmake) | 0.1.0 |
| 真机通信 | python-can / SocketCAN, DrEmpower 协议 | — |
| 数值 | numpy, scipy, yaml | — |
