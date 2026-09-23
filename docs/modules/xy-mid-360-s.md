# xy-mid-360-s

纯 Python 的 Livox MID-360 / MID-360S 驱动：设备发现、设备控制面、连续点云接收，以及可选的 ROS 2 节点。不依赖厂商 SDK（Livox-SDK2）也不需要 colcon 构建；除 ROS 节点外不依赖 ROS。

以下安装、测试和相对路径命令均在仓库的 `xy-mid-360-s/` 目录执行。

## 来源与许可边界

本包是 robot_safecontrol 项目内模块 `src/robot_safecontrol_moveit/livox_mid360/` 的独立副本，原始来源为用户提供的 `tmbs-main.zip`（`backend/app/drivers/mid360_driver.py`）。完整的复用与适配记录见 robot_safecontrol 仓库的 `docs/planning/oscbf-reuse/research/tmbs-mid360-python-driver-reuse.md`。

原始 zip 根目录没有 LICENSE（只有第三方 vendor 条款）；用户已确认该来源可用于本项目。**对外分发或再许可前仍需确认许可范围。**

## 兼容性与验证状态

- 型号前提：**MID-360S（设备类型 35）**，同时兼容 MID-360（类型 9）。广播发现按设备类型映射型号名，unicast 探测不臆断型号。
- 已在真机 MID-360S（`192.168.1.115`，固件字节 `[35,1,1,8]`）验证：广播发现、连续流 10 Hz、200,318 点/s、14 s 零丢包、与官方 `livox_ros_driver2` 逐帧内容一致（`point_step=26`）。
- 真机修复过的两个缺陷：Fast-DDS 共享内存单样本 512 KiB 上限导致的丢帧（`MAX_FRAME_POINTS=20,000` 硬切帧），以及发现应答是广播目的地址、只有 `INADDR_ANY` 绑定的 socket 能收到。
- 未验证：PTP/GPS 同步实测、长时间稳定性与断线重连、写命令（`configure`/`mode`/`reboot`）的真机演练。

## 安装

```bash
pip install -e .            # 只装 numpy
pip install -e '.[test]'    # 另装 pytest
```

ROS 2 节点需要已 source 的 ROS 2 环境提供 `rclpy` 与 `sensor_msgs`（它们不在 PyPI 上），本包的 pip 依赖里不含它们。

## 命令行（不需要 ROS）

```bash
xy-mid-360-s discover --host-ip 192.168.1.5
xy-mid-360-s info --ip 192.168.1.115 --host-ip 192.168.1.5
xy-mid-360-s stream --host-ip 192.168.1.5 --seconds 10
```

未安装时用 `PYTHONPATH=. python3 -m xy_mid_360_s.cli <命令>` 代替。

只读命令（不改设备状态）：`discover`、`sn`、`info`、`mode get`、`ip get`、`fov get`、`pattern get`。

写设备命令（会改设备状态，按需显式执行）：`configure`（把推流目标设为本机）、`mode set sampling|standby|ready`、`ip set`、`fov set`、`pattern set`、`reboot`。

## Python API（无 ROS 抓流）

```python
from xy_mid_360_s import CloudReceiver

def on_frame(frame):
    print(frame.size, "points", frame.base_time_ns, frame.src_ip)

receiver = CloudReceiver(host_ip="192.168.1.5", on_frame=on_frame)
receiver.start()
```

完整示例见 `examples/capture_no_ros.py`。帧是 numpy 数组（`x/y/z`、`intensity`、`tag`、`line`、逐点时间），没有 ROS 话题；需要 PointCloud2 就用下面的节点。

## ROS 2 节点（可选）

```bash
source /opt/ros/humble/setup.bash
xy-mid-360-s-node        # 发布 /livox/lidar，PointCloud2，point_step=26
```

`configure_on_start` 默认 `false`：只启动节点不会写设备配置。要一次性把设备配好，用 `xy-mid-360-s mode set sampling --ip <addr>`。

## 关键约束

- **与官方驱动互斥**：官方 `livox_ros_driver2` 与本包绑定同一组 UDP 端口（点云 56301、状态 56201）并发布同一话题 `/livox/lidar`，不能同时运行。
- **512 KiB 规则**：Fast-DDS（`rmw_fastrtps_cpp`）共享内存单样本上限 512 KiB，按 0.1 s 切帧会产出约 20,000 点的帧，因此 `MAX_FRAME_POINTS=20,000` 是硬上限，不是可调建议。
- **推流目标**：设备只向最近一次配置的主机:端口推流（设备源端口 56300 → 主机 56301）。`stream` 前需要先 `configure`，或设备已被官方驱动配置过。
- 端口约定：发现/命令 56000/56100，状态 56200 → 56201，点云 56300 → 56301，IMU 56400 → 56401。

## 测试

```bash
python3 -m pytest tests/ -q                              # 无 ROS 部分
source /opt/ros/humble/setup.bash
python3 -m pytest tests/test_livox_mid360_ros_node.py -q  # 需要 rclpy/sensor_msgs
```

测试全部基于合成 UDP 包与 loopback 假设备，不需要硬件，也不改设备状态。
