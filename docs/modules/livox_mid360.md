# MID-360 / MID-360S 驱动

项目使用 `src/robot_safecontrol_moveit/livox_mid360/` 作为 MID-360 / MID-360S 驱动的维护来源。该模块提供设备发现、诊断、点云接收和可选 ROS 2 节点。来源许可、历史真机验证与接收约束见[模块 README](../../src/robot_safecontrol_moveit/livox_mid360/README.md)。

## 安装与运行

从仓库根目录执行 `bash build_aeb_moveit.sh`，然后加载环境：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_safecontrol_moveit livox_mid360_tool --help
ros2 run robot_safecontrol_moveit livox_mid360_node
```

节点发布 `/livox/lidar` 的 `PointCloud2`。启动节点默认只监听设备，不写入设备配置。`livox_mid360_tool` 的设备写入命令会修改设备状态，具体命令和已验证范围见模块 README。

Python 接收 API：`from robot_safecontrol_moveit.livox_mid360 import CloudReceiver`。模块的协议与接收部分使用 NumPy，ROS 节点另外依赖 ROS 2 Humble 的 `rclpy` 和 `sensor_msgs`。

## 验证

```bash
python3 -m pytest tests/test_livox_mid360_protocol.py tests/test_livox_mid360_device.py tests/test_livox_mid360_stream.py tests/test_livox_mid360_ros_node.py -q
```

这些测试检查协议、接收和节点行为。历史真机记录见[研究资料](../planning/oscbf-reuse/research/tmbs-mid360-python-driver-reuse.md)；本地测试通过不代表当前设备已完成真机验收。
