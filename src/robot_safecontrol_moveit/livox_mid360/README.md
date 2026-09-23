# livox_mid360

本模块提供 Livox MID-360 / MID-360S 的设备发现、控制面、连续点云接收和 ROS 2 节点。驱动协议与接收代码位于本目录，主包通过 `livox_mid360_tool` 和 `livox_mid360_node` 安装入口提供命令。

## 来源与适用范围

代码最初参考用户提供的 `tmbs-main.zip` 中 `backend/app/drivers/mid360_driver.py`，复用与适配记录见[研究记录](../../../docs/planning/oscbf-reuse/research/tmbs-mid360-python-driver-reuse.md)。原始压缩包根目录没有 LICENSE，只有第三方 vendor 条款；用户已经确认可用于本项目。对外分发或再许可前仍需确认许可范围。

设备类型 35 对应 MID-360S，类型 9 对应 MID-360。广播发现按设备类型报告型号；unicast 探测不推断型号。

历史真机验证使用 MID-360S `192.168.1.115`、固件字节 `[35,1,1,8]`：广播发现、10 Hz 连续流、200,318 点/s、14 s 零丢包，点云内容与 `livox_ros_driver2` 逐帧一致，`PointCloud2.point_step=26`。PTP/GPS 同步、长期稳定性、断线重连和写设备命令尚无真机验收记录。

## 入口

构建并加载项目环境后，使用 `ros2 run robot_safecontrol_moveit livox_mid360_tool --help` 查看诊断命令，使用 `ros2 run robot_safecontrol_moveit livox_mid360_node` 启动 ROS 节点。节点发布 `/livox/lidar`，`configure_on_start` 默认 `false`。

`discover`、`sn`、`info`、`mode get`、`ip get`、`fov get`、`pattern get` 读取设备信息；`stream` 接收设备已经推送的点云。`configure`、`mode set sampling|standby|ready`、`ip set`、`fov set`、`pattern set`、`reboot` 会修改设备状态，须在明确安排设备操作时执行。

无需 ROS 节点时可以直接接收点云：

```python
from robot_safecontrol_moveit.livox_mid360 import CloudReceiver


def on_frame(frame):
    print(frame.size, frame.base_time_ns, frame.src_ip)


receiver = CloudReceiver(host_ip="192.168.1.5", on_frame=on_frame)
receiver.start()
```

`CloudReceiver` 帧包含 `x/y/z`、`intensity`、`tag`、`line` 与逐点时间。设备需要已经把推流目标配置为本机地址与端口。

## 接收约束

- 与 `livox_ros_driver2` 使用相同 UDP 接收端口和 `/livox/lidar` 话题，同机不能同时运行。
- Fast-DDS 共享内存单样本约 512 KiB 的限制对应 `MAX_FRAME_POINTS=20,000`。切帧上限来自已有真机丢帧问题的修复记录。
- 设备向最近配置的主机与端口推流。点云为 56300 → 56301，状态为 56200 → 56201，IMU 为 56400 → 56401；发现和命令使用 56000/56100。
- 设备发现响应可能发送至广播地址；接收 socket 需要绑定 `INADDR_ANY`。

模块测试位于仓库根目录的 `tests/test_livox_mid360_*.py`，使用合成 UDP 包与回环网络验证协议和接收行为。运行入口与安装步骤见[驱动说明](../../../docs/modules/livox_mid360.md)。
