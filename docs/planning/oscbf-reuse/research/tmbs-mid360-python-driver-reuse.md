# tmbs-main MID-360 驱动：复用与适配记录

日期：2026-09-09。对应 [P0 双传感器官方驱动接入与版本冻结](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/31)。用户提供 `~/robot/tmbs-main.zip`（Tunnel Model Building System：FastAPI 后端 + 前端 + 平台层，1480 文件约 10.9 MB），本记录只评估其中 `backend/app/drivers/mid360_driver.py`（2327 行）对项目激光雷达驱动的可用性，并给出已落地的适配实现。**型号前提为 Mid-360S（设备类型 35），不是 MID-360（类型 9）。** 2026-09-09/10 已在真机（MID-360S、type 35、`192.168.1.115`、固件字节 `[35,1,1,8]`，见 [#31 在线窗口](../handoffs/31-official-sensor-drivers.md#2026-09-09-在线验证进行中)）完成发现、连续流与 `perception_bridge` 端到端验证；过程中修复两个只在真机暴露的缺陷（Fast-DDS 单样本 512 KiB 上限、发现应答的广播目的地址），见下"真机验证"。

## 结论

该驱动是一个**不依赖厂商 SDK 的纯 Python Livox 以太网协议实现**，正补上项目当前的空缺：官方 C++ 驱动只做采集发布，不提供设备控制（发现/SN/IP/FOV/工作模式/重启），且需要 SDK2 + colcon 构建。已适配为项目内模块 `src/robot_safecontrol_moveit/livox_mid360/`（7 文件 2060 行，57 个新测试全过），保留其协议层与设备命令面，替换其批次采集模型为连续流，并对齐官方 PointCloud2 字段布局；真机上发现并修复一个 Fast-DDS 单样本 512 KiB 上限导致的丢帧缺陷。**按 #31 的定位，官方 C++ 驱动仍是主路径；本模块是设备控制工具 + 免 SDK 的备用/诊断通道，不是替代品。**

## 来源与许可边界

zip 根目录**没有 LICENSE**（仅 `vendor/victorialogs/LICENSE.txt` 等第三方条款），README 只描述系统用途。复用的内容大部分是 Livox 公开以太网协议 v1.4.x 的常量与帧格式（与官方 SDK2 头文件、协议文档一致），不属于该项目的独创表达；但整段实现不应在确认来源与许可前对外分发。解压副本仅存于 `.scratch/tmbs-driver-review/`（未入库）。**是否确认可用于本项目，见文末待决项 1。**

协议事实以官方源码为准复核，副本在 `.scratch/oscbf-reuse-wayfinder/31/online-2026-09-09/upstream/`（`livox_ros_driver2` 1.2.6、`Livox-SDK2` v1.3.1）。zip 的字段偏移在 SN/状态/温度/IP/pattern 上与官方一致；差异集中在解析封装与采集模型，见下。

## 复用了什么

| 能力 | zip 出处 | 项目落点 |
|---|---|---|
| 控制帧 24B 头、CRC-16/CCITT-FALSE、CRC-32、请求/应答帧构建与校验 | `_build_ctrl_frame` / `_parse_ctrl_frame` | `protocol.build_ctrl_frame` / `parse_ctrl_frame` |
| key_value_list 构建（config/inquire）与解析 | `_build_param_config` / 各命令内联解析 | `protocol.build_param_config` / `build_param_inquire` / `parse_key_value_list` |
| 点云 36B 包解析、14B 点格式、tag 置信位、CRC 策略 | `_parse_pcl_packet` | `protocol.parse_cloud_packet` / `CloudPacket` |
| 端口、命令 ID、参数 Key、工作模式/状态常量 | 文件头常量区 | `protocol` 常量区 |
| 设备命令面：SN、状态、温度、IP 读写、FOV 双 profile、pattern、工作模式、重启 | `Mid360Device`（384–795 行） | `device.Mid360Device` |
| 广播发现与 unicast 探测 | `discover_subnet` / `discover_device` | `discovery` |
| 点云接收线程、状态推送线程 | `_pcl_recv_loop` / `_state_recv_loop` | `stream.CloudReceiver` / `StateMonitor` |

## 适配改了什么

1. **采集模型：批次 → 连续流。** zip 是单次采集原语（`create_task`/`start_batch`/`stop_current_batch`，含握手 worker 状态机与 1e8 点级缓冲）；项目侧 `perception_bridge` 订阅的是连续话题，官方驱动也连续发布，因此改为 `FrameAssembler`（按 0.1 s 时间跨度 / 40 万点上限 / 时间回退切帧）+ `CloudReceiver`（含停滞看门狗，短流也能出帧）。
2. **点云保真：不丢点。** zip 固定丢弃 tag 低置信点并只输出 `x,y,z,refl` 四列 float32。项目默认 `keep_all_points=True`，保留 tag/line、毫米整型与包时间；置信度过滤交由消费侧决策，驱动不替下游丢数据。
3. **字段布局对齐官方驱动。** 发布 `PointCloud2` 采用官方 `lddc.cpp` 布局：x/y/z/intensity FLOAT32 @ 0/4/8/12、tag/line UINT8 @ 16/17、timestamp FLOAT64 @ 18，`point_step=26`。因此 `perception_bridge` 无需任何改动即可解码（已有测试用 `_points_xyz` 验证）。
4. **时间语义对齐官方。** 新增 `packet_time_base_ns`（time_type 为 PTP/GPS 时用设备时间，否则用主机接收时间）与 `point_interval_ns`（`time_interval*100/dot_num`），与 `pub_handler.cpp` 的 `GetEthPacketTimestamp`/逐点重建一致。**复核时发现并修正一处常量命名错误：** `TIME_TYPE_PTP/GPS` 初版与上游枚举顺序（NoSync=0, GptpOrPtp=1, Gps=2）颠倒，已改正并加回归测试（行为原本不受影响，因为两种同步类型走同一分支）。
5. **应答解析按 SDK2 结构区分两种形状。** 查询 ACK 为 `ret_code(1)+param_num(2)+{key,len,value}`（无 reserved），设备推送为 `key_num(2)+rsvd(2)+{...}`；`parse_key_value_list` 显式区分并对两种前缀回退探测，各查询改为按键取值，不再依赖魔数偏移。
6. **去平台耦合。** 移除 FastAPI 模型、平台日志、驱动注册表与会话状态依赖；模块只依赖 `numpy`（ROS 节点另需 `rclpy`）。
7. **Mid-360S 修正。** zip 常量已含 `DEV_TYPE_MID360S=35`，但发现路径默认断言为 MID-360。已改为：unicast 探测不臆断型号（返回 `device_type=None`），广播发现按 9/35 映射名称，文档与注释统一写明 S 型语义（READY 0x09、PTP/GPS 时域）。
8. **运行时不改设备状态。** ROS 节点 `configure_on_start` 默认 `False`——仅启动节点不推送流目标、不切换模式，符合项目"运行不得改变设备状态"的规则；QoS 默认 best-effort，与 `qos_profile_sensor_data` 订阅兼容。
9. **支持 data_type=2**（int16 厘米）并在版本字段异常时降级解码，而非直接丢弃。
10. **切帧上限对齐 RMW 传输上限（真机发现）。** `MAX_FRAME_POINTS` 从“失控保护”升为切帧主规则并定为 **20,000**：Fast-DDS 共享内存单样本上限 512 KiB，而 0.1 s 间隔加 96 点包粒度会让帧长到 20,064–20,544 点（521,664–534,144 B），越过上限后订阅端只剩几 Hz。详见下文“真机验证”。
11. **广播发现 any-bind（真机发现）。** 设备对发现广播的应答是广播目的地址，只有 `INADDR_ANY` 绑定的 socket 收得到；已改为 any-bind，`host_ip` 只用于 unicast 回退探测。

## 未复用（明确放弃）

- 批次/任务 API 与握手 worker 状态机（与项目连续流模型冲突，且引入大量并发状态）。
- FastAPI 路由与模型、前端、驱动注册表、会话状态、平台日志（项目无此技术栈）。
- 1e8 点级缓冲与"预热丢首帧"启发式（应由消费侧按任务决定，驱动不应内置）。
- 模拟驱动（项目已有自己的测试替身与合成数据生成器）。

## Mid-360S 专属注意

- 设备类型 **35**（MID-360 是 9），`protocol.DEV_TYPE_NAMES` 已区分；广播发现按此映射。
- `READY(0x09)` 是 MID-360/HAP 可写模式：电机运行、激光关闭，切到 SAMPLING 快于 STANDBY(0x02)；S 型同样支持。
- 官方 launch 是 `msg_MID360s_launch.py` 且默认 `xfer_format=1`（CustomMsg），与官方路径联调时必须显式 `xfer_format=0`（详见 [#31 研究](./official-sensor-drivers.md)）。
- 具备 PTPv2/GPS 能力不等于现场已同步；未同步时本模块与官方驱动一样退回主机时间，不能当采样时刻用。

## 真机验证（2026-09-09/10）

设备：MID-360S，type 35，SN `ARMCP7D****`，IP `192.168.1.115`，主机 `192.168.1.5/24`（网卡 `enx6c1f******`）；实测 ~200,318 点/s、96 点/包、~2,096 包/s、4 线。证据目录 `.scratch/livox-hw-test/`（脚本、日志、逐帧统计 JSONL）。

### 发现：应答的目的地址是广播

广播发现此前零应答，只有 unicast SN 探测可用。根因：设备对发现广播的应答把目的地址写成广播地址，内核只把这类报文投递给绑定 `INADDR_ANY` 的 socket，绑定 `host_ip` 收不到。已改为 any-bind（子网定向广播仍按路由从正确网卡发出，多网卡主机不受影响），保留 unicast 探测作为回退。修复后 `discover_subnet` 连续 5/5 次、CLI `discover` 2/2 次返回 SN `ARMCP7D****`；期间出现过一次空 SN，未复现，未改代码。

### 连续流：Fast-DDS 单样本 512 KiB 上限

首轮采集：节点内部组装与发布都正常（组装 0.67 ms、发布 0.19 ms/帧，帧间隔均值 101.3 ms），但订阅端只有 4.6–7.25 Hz。C++ `ros2 topic hz` 同样丢帧（排除 rclpy 转换/发布路径），16 核 96% 空闲（排除算力），RELIABLE QoS 反而更差（0.6 Hz）。

尺寸扫描定位根因：**Fast-DDS（本项目 RMW `rmw_fastrtps_cpp`）共享内存传输的单样本上限为 512 KiB（524,288 B）**，超过后回落到分片 UDP，绝大多数帧丢失。best-effort 订阅端实测（`sweep_*.log`）：

| 每帧点数 | 载荷字节 | 订阅端 |
|---|---|---|
| 20,000 | 520,000 | 10 Hz 稳定（每秒 10 帧） |
| 20,100 | 522,600 | 10 Hz 稳定 |
| 20,160 | 524,160 | 3.57 Hz，最大间隔 3.4 s |
| 20,200 | 525,200 | 6.50 Hz |
| 21,000 | 546,000 | 2.21 Hz |

悬崖正好在 512 KiB：20,160×26 B = 524,160 B 加上 CDR 开销即越过 524,288 B。修复是把 `MAX_FRAME_POINTS` 定为 **20,000** 并作为切帧主规则（此前 0.1 s 间隔加 96 点包粒度会溢出到 20,064–20,544 点，正好跨在悬崖两侧），同时与官方驱动每帧 ~20,000 点一致；回归测试锁定“点数×26 B + 4 KiB 序列化开销 < 512 KiB”。`publish_freq < 10` 时节点给出告警：此时由点数上限而非频率决定速率。

修复后同机复测（`hz_final.log`、`sub_rate_final.log`）：C++ 订阅端 9.969 Hz 稳定（窗口 111 帧、std dev 0.5 ms、间隔 98–102 ms），Python 订阅端 12 s 收 120 帧 = 10.00 Hz（间隔均值 100.3 ms、max 101.6 ms）。采集侧 14 s 无丢包：29,213 包 / 2,804,448 点 / 138 帧，点速率 200,318 点/s。与官方驱动的 10.00 Hz 差 0.3%，来自按 0.1 s 时间跨度切帧的边界粒度，不是丢帧。

### 与官方驱动的逐帧内容对比

同机交替运行官方 `livox_ros_driver2` 1.2.6（`xfer_format=0`）与本模块，各取 20 帧（`stats_official.jsonl` / `stats_mynode.jsonl`）：

| 指标 | 本模块 | 官方驱动 |
|---|---|---|
| 每帧点数 | 20,064（恒定） | 19,872–20,064 |
| point_step / frame_id | 26 / `livox_frame` | 26 / `livox_frame` |
| 零返回值占比 | 34.96% | 35.48% |
| 平均 / 最大距离 | 4.95 / 18.10 m | 4.94 / 18.46 m |
| 平均强度 | 43.36 | 42.91 |
| 帧内时间跨度 | 99.29 ms | 99.90 ms |
| 4 线直方图 | 5016×4（均衡） | 4992×4（均衡） |
| tag 直方图均值 | 18460/93/16/0 | 18348/96/16/0 |

帧首/帧尾零返回串 ≤44 点（`edge_zeros.py`）。无系统性内容差异；帧边界不同只影响首点位置，不影响点集。

### 与官方驱动的能力对比

上节比逐帧内容（等价），这里比能力面。对比对象为官方 `livox_ros_driver2` 1.2.6（`xfer_format=0`，源码见 `.scratch/oscbf-reuse-wayfinder/31/online-2026-09-09/upstream/`）。

| 维度 | 本模块 | 官方驱动 |
|---|---|---|
| 部署形态 | 纯 Python（依赖 numpy；节点另需 rclpy），无 SDK、无 colcon 构建 | C++，需 Livox-SDK2 + `build.sh humble`，运行需把 SDK 的 `lib` 加入 `LD_LIBRARY_PATH` |
| 话题 | 仅 `/livox/lidar`（PointCloud2） | `/livox/lidar` + `/livox/imu`（启动时使能 IMU）；`multi_topic=1` 时按设备 IP 分话题 |
| 消息格式 | 仅 PointCloud2，`point_step=26`，布局与官方 `lddc.cpp` 一致 | `xfer_format` 0（Livox PointCloud2）/ 1（CustomMsg）；2（PCL PointXYZI）在 ROS 2 分支被 `#if 0` 关闭（`lddc.cpp:538-544`），实际只有 0/1 |
| 多雷达 | 单设备（单 `FrameAssembler`） | `multi_topic` / `data_src` 支持多设备 |
| 设备控制 | CLI + `Mid360Device`：discover / sn / info / ip / fov / pattern / mode / configure / reboot | 无控制工具；启动时按 JSON 配置写设备（点云类型、扫描模式、安装外参、工作模式、IMU 使能） |
| 启动副作用 | `configure_on_start=false` 默认只读：不推送流目标、不切模式 | 启动即写设备配置 |
| 切帧规则 | 0.1 s 时间跨度 + `MAX_FRAME_POINTS=20,000` 硬上限 | 纯时间间隔 `publish_interval_ = 1 s / publish_freq`（`pub_handler.cpp:65-66`），无点数上限 |
| QoS | 默认 best-effort（depth 5），可切 reliable | rclcpp 默认 reliable（全局话题 depth 256，`lddc.cpp:643-658`）；两种默认都与 `qos_profile_sensor_data` 订阅兼容 |
| 发布速率（真机） | 9.97 Hz（切帧边界粒度所致，非丢帧） | 10.00 Hz |
| 维护与许可 | 项目内自维护；许可来源待确认（待决项 1） | 厂商维护；MIT + 第三方通知 |

互补关系与共同边界：

- **官方独有**：`/livox/imu`、CustomMsg（`xfer_format=1`，官方 S 型 launch 的默认值）、多雷达——这三项本模块不提供。
- **本模块独有**：设备控制面（官方无对应工具）、免构建部署、只读启动、无 ROS 环境下的裸机抓流（`livox_mid360_tool stream`）。
- **512 KiB 上限对官方同样成立**：官方切帧只看 `publish_freq`，没有点数上限。默认 10 Hz 时每帧 ~20,000 点，恰好压在悬崖下（真机 19,872–20,064 点）；README 推荐的 5 Hz 会让单帧涨到 ~40,000 点（~1.04 MB），越过 512 KiB 后预计出现与本模块修复前相同的丢帧。此条为源码推断，**未实测**——改 `publish_freq` 本身不写设备，但再启官方节点会写设备配置，需显式授权后做。
- **互斥**：两者绑定同一 UDP 端口（56300/56200）并发布同一话题，不能同时运行；接入时必须二选一或改用不同话题名（待决项 4）。

### 端到端集成（perception_bridge）

本轮相机未运行（LiDAR-only），`perception_bridge` 以 `source_topic_lidar:=/livox/lidar` 运行 8 s 以上（`hz__*.log`、`status_once.log`）：`/livox/lidar` 9.97 Hz、`/perception/status` 19.99 Hz、`/perception/tracks` 10.00 Hz、`/perception/instant_occupancy` 9.92 Hz、`/perception/cloud_world` 3.64 Hz、`/perception/esdf` 2.14 Hz、`/collision_object` 29.90 Hz。`/perception/status` 采样（10 元素，顺序见 `fusion_engine.py` 头注释）：`camera_alive=0, lidar_alive=1, lidar_age=0.14 s, lidar_used=1, fusion_stamp=1788971264, source_count=1, perception_valid=1`。融合 20 Hz 对雷达 10 Hz 的相位差会让 `lidar_used` 每两拍为 0，这是 FusionEngine 第 6 步“重复帧保护”（同一 stamp 组合重复即返回空结果）的既定行为，不是掉流；`lidar_alive` 全程为 1。

## 验证状态

- **57 个新测试全过**（`tests/test_livox_mid360_{protocol,stream,device,ros_node}.py`），全部基于合成 UDP 包，无需硬件。
- 全量回归：主包 `tests/` **307 通过**；`portable_oscbf/tests` 146 通过 / 34 跳过 / **1 失败**——该失败为既有 JAX 浮点容差问题（8.6e-08 超出 atol 1e-08），`portable_oscbf` 无本地改动，与本模块无关。
- 新增入口点：`livox_mid360_node`（ROS 2 节点）、`livox_mid360_tool`（discover/info/sn/configure/mode/ip/fov/pattern/reboot/stream）。
- **已做（本模块维度）**：真机发现（广播 + unicast）、只读查询（sn/info）、连续流采集与发布、与官方驱动的逐帧内容对比与能力对比、`perception_bridge` 端到端集成。**未做**：PTP/GPS 同步实测（设备未同步，两驱动都退回主机时间）、长时间稳定性与断流重连、`configure` 类写命令的真机演练（会改设备状态，需显式授权）。注意官方节点启动会写入设备配置（点云类型、扫描模式、外参、工作模式、IMU 使能，见 #31 在线记录），本模块的只读命令（sn/info/stream）可用于对照而不改状态。

## 与官方驱动的分工（建议）

| 场景 | 用哪个 |
|---|---|
| 生产点云话题（供 `perception_bridge`） | 官方 `livox_ros_driver2`（#31 主路径，`xfer_format=0`） |
| 设备发现/SN/IP/FOV/工作模式/重启 | 本模块 CLI 或 `Mid360Device` |
| 官方驱动不可用时的诊断与备用采集 | 本模块 `livox_mid360_node` |
| 无 ROS 环境的裸机排查 | 本模块 `livox_mid360_tool stream` |

## 待决项

1. **许可**：zip 无根 LICENSE，需用户确认来源与可用范围，之后该模块才能入库/外发。
2. **定位**：仅作控制工具（推荐），还是允许作为官方驱动不可用时的采集备用路径？
3. **配置接入**：是否把 `livox_mid360_node` 写入 launch/config？当前两份感知配置的 `source_topic_lidar` 仍为 `""`（LiDAR 默认关闭，启用受 T5/T6/T7 门控），本改动未触碰。
4. **并行冲突**：真机确认设备只向最近一次配置的主机:端口推流（本轮官方驱动写入的 `192.168.1.5:56301` 在官方进程退出后仍持续有效，本模块正是靠它免配置收流），两个驱动同时运行会抢同一 UDP 端口并重复发布同一话题，接入时必须二选一或改用不同话题名。
