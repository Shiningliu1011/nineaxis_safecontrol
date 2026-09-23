## 已排除的文件 (ROS 依赖)

以下文件**未包含**在本移植包中, 因为它们依赖 ROS 2 (`rclpy`, `sensor_msgs`, `geometry_msgs` 等):

| 文件 | 原因 |
|------|------|
| `newaxis/run_oscbf_rviz_newaxis.py` | ROS 2 主节点 |
| `newaxis/rviz_publisher.py` | RViz 可视化 |
| `newaxis/transition_executor.py` | ROS 消息类型 |
| `newaxis/obstacle_scene_builder.py` | ROS 节点状态 |
| `newaxis/oscbf_controller.py` | ROS 节点引用 |
| `newaxis/tracking_execution.py` | ROS 节点引用 |
| `newaxis/cbf_constraint_builder.py` | ROS 节点引用 |
| `newaxis/avoidance_state_machine.py` | 为 ROS runner 设计 |
