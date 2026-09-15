# 双源时间对齐：Livox Mid-360S 与 Orbbec Gemini 335L 在 ROS 2 Humble 下的时间戳一手研究

日期：2026-09-14。范围：只查一手来源（厂商文档、厂商固定版本源码、规范、ROS 2 Humble 官方源码/设计文档），回答「两个传感器各自的时间戳来自哪里、官方支持哪些同步手段、ROS 2 提供哪些时间设施、以及本仓库现有融合逻辑会怎样失效」。
本文不实施、不接线、不启动设备、不修改任何已有文件；不含任何未标出处的数值。
证据等级标记：**【一手文档明确】**（厂商文档/规范/官方设计文档明文）、**【一手源码可见】**（固定 commit 的官方源码可查）、**【推断（需实测确认）】**（由已核实源码/规范推出的工程结论，未在任何设备上验证）、**【未知】**。

来源读取方式：本次在 2026-09-14 直接拉取固定 commit 的源码与厂商页面（命令与 URL 见第 9 节）。仓库侧结论来自本仓库当前工作区文件与 GitHub issue #7/#45/#51 正文。

---

## 1. 问题与范围

用户现状（既定事实，不是待确认项）：**当前雷达和相机的时间还没有做对齐**——两路传感器时间戳尚未建立可信的共同时间基准。

本项目的下游用途：`perception_bridge` 把 LiDAR + 相机点云融合为 `/perception/esdf`、`/perception/tracks`、`/perception/instant_occupancy`，交给九轴 OSCBF 安全控制器当障碍输入。时间对齐一旦失真，「障碍年龄」`age = now - sensor_stamp` 就失真，安全间距预算与静态/动态分层判据随之失真。因此本文只回答「时间戳从哪来、能不能对齐、对齐到什么程度可判定」，不讨论外参标定（另见 `sensor_extrinsics.yaml` / ADR 0004）。

涉及的两个驱动（本仓库既有冻结候选，见 `docs/planning/oscbf-reuse/research/official-sensor-drivers.md`）：

- Livox **livox_ros_driver2 `1.2.6`** @ `13eb05e4e6dd7a765b934d0c5fd6236676a57b49`，依赖 Livox SDK2 `v1.3.1` @ `f5d9375f84efe2b15bc0a052d3e18482ed13adf4`；`xfer_format=0`（PointCloud2），`frame_id=livox_frame`。
- Orbbec **OrbbecSDK_ROS2 `v2.9.3`** @ `ce08bce25f7a0a6fe939ece87ec945447109581f`（包内自带 SDK 2.9.3）；`depth_registration:=true`，注册彩色点云话题 `/camera/depth_registered/points`，`frame_id` 默认 `camera_color_optical_frame`。

---

## 2. 当前事实：两源时间戳来源与时间域

### 2.1 Livox Mid-360S（经 livox_ros_driver2 1.2.6，`xfer_format=0`）

**设备协议层事实【一手文档明确】**（Livox wiki《激光雷达通信协议–Mid360》点云数据协议）：

- UDP 包头的 `timestamp`（偏移 28，8 字节）**代表该包第一个点的时间**；包内 `time_interval`（偏移 3，2 字节，单位 0.1 µs）的定义是「这帧点云数据中最后一个点减去第一个点时间」，包内 N 个点等间隔。
- `time_type`（偏移 11，1 字节）三种取值：`0` = 无同步源，**时间戳为雷达开机时间**；`1` = gPTP/PTP 同步，时间戳为 master 时钟源时间；`2` = GPS 同步。单位均为 ns（uint64）。
- 同一页脚注：GPS 时间同步的有效范围为 2000-01-01 到 2037-12-31；PTP 不支持 IEEE1588v2.1；不建议 IEEE1588v2.0 与 gPTP 并存。

**驱动层事实【一手源码可见】**（driver `13eb05e4...`）：

- `src/comm/comm.h:94-99` 定义驱动侧的 `TimestampType`：`kTimestampTypeNoSync=0`、`kTimestampTypeGptpOrPtp=1`、`kTimestampTypeGps=2`。**这三个常量定义在驱动仓库内**，不在 SDK2 v1.3.1 的公开头文件里（本次在 SDK2 全树 grep `kTimestampType` 无匹配）。
- `src/comm/pub_handler.cpp:105-109`：按包的 `time_type` 置全局 `is_timestamp_sync_`；`143-144`：`packet.time_stamp = GetEthPacketTimestamp(...)`。
- `src/comm/pub_handler.cpp:265-275` 是**决定性的**一段：

  ```cpp
  if (timestamp_type == kTimestampTypeGptpOrPtp ||
      timestamp_type == kTimestampTypeGps) {
    return time.stamp;            // 用设备包内时间戳
  }
  return std::chrono::high_resolution_clock::now().time_since_epoch().count();
  ```

  即：**设备未同步（`time_type=0`）时，驱动丢弃设备时间戳，改用主机 `high_resolution_clock::now()` 的 count 作为包时间**；设备已 PTP/GPS 同步时用设备时间戳。
- `src/comm/pub_handler.cpp:142`：`point_interval = time_interval * 100 / dot_num`（ns）；`388`：`point.offset_time = pkt.time_stamp + i * pkt.point_interval`。**逐点时间是「包时间 + 序号×间隔」重建出来的，不是每点独立硬件打点。**
- `src/comm/pub_handler.cpp:281-286`：`GetLidarBaseTime()` 返回本批第一个点的 `offset_time`；`178` / `209` 把它写进 `frame_.base_time`，再经 `src/lds.cpp:134,144` → `src/comm/ldq.cpp:139` 存为 `StoragePacket::base_time`。
- `src/lddc.cpp:298-333` `InitPointcloud2Msg`：`timestamp = pkg.base_time`（`310`），`cloud.header.stamp = rclcpp::Time(timestamp)`（`316`），逐点 `point.timestamp = (double)pkg.points[i].offset_time`（`328`）。
  → **`PointCloud2.header.stamp` = 该批点云第一个点的时间；逐点 `timestamp` 字段（FLOAT64，offset 18）是同一时间基准下的绝对 ns 值，不是相对 header 的秒数。** 字段布局见 `src/lddc.cpp:284-295`（x/y/z/intensity FLOAT32 offset 0/4/8/12；tag/line UINT8 16/17；timestamp FLOAT64 18；`point_step=26`）。
- 发布节奏：`src/comm/pub_handler.h:121-123` 默认 `publish_interval_=100 ms`，由 `publish_interval_ = (kNsPerSecond/(publish_freq*10))*10`（`pub_handler.cpp:66-68`）从 `publish_freq` 推出。`pub_handler.cpp:166-194`（已同步分支）用**设备时间戳**取模 `publish_interval_ms_` 作为发布闸门；`195-207`（未同步分支）用主机 `high_resolution_clock` 计时发布。**两条分支的时间来源不同**，未同步时发布节奏完全由主机时钟决定。

**「未同步」时到底是什么时间域【一手源码可见 + 本机实测】**：`high_resolution_clock` 在 libstdc++ 中被定义为 `system_clock`（注释：「Alias to std::system_clock until higher-than-nanosecond definitions become feasible」，`bits/chrono.h:1274-1279`，本次核对 gcc-12.3.0 源码），而 `system_clock` 是「wall time from the system-wide clock」。本机（Ubuntu 22.04 / g++ 11.4.0）实测：`high_resolution_clock == system_clock` 为真，`period` = 1/1e9，`count` ≈ 1.7893e18 ≈ Unix epoch 纳秒。所以**在本机、本驱动构建下，未同步的 Livox header stamp 名义上就是主机 CLOCK_REALTIME 的 Unix epoch 纳秒，取自驱动处理包的时刻**（不是激光采样时刻）。C++ 标准不保证这一点；换平台/标准库需重测（见第 8 节）。

**与时间相关的驱动参数（逐项，`launch_ROS2/msg_MID360s_launch.py:8-14` 声明，全部按原样传给节点）**：

| 参数 | 类型 | 官方 MID360s launch 默认 | 语义与时间相关性 |
|---|---|---|---|
| `xfer_format` | int | `1`（CustomMsg） | `0` 才发 `sensor_msgs/PointCloud2`；`1` 发 `livox_ros_driver2/CustomMsg`（含 `timebase` + 逐点相对 `offset_time`）。本仓库必须显式置 `0`。 |
| `publish_freq` | double | `10.0` | 决定 `publish_interval_`（`pub_handler.cpp:66`）。**没有「每帧点数上限」参数**；点数随切帧时长增长。 |
| `multi_topic` | int | `0` | 0 = 所有雷达共用 `/livox/lidar`；1 = 每雷达一个话题。与时间无关，但与话题身份相关。 |
| `data_src` | int | `0` | 0 = 雷达；其他值无效。 |
| `output_data_type` | int | `0` | 0 = 发布到 ROS。 |
| `frame_id` | string | `'livox_frame'` | 可配置值，非厂商固定常量；只影响 header.frame_id，不影响 stamp。 |
| `lvx_file_path` | string | `/home/livox/livox_test.lvx` | 离线回放路径。 |
| `user_config_path` | string | `config/MID360s_config.json` | 网络端口/设备 IP；**该 JSON 内没有任何时间同步字段**（本次核对 `config/MID360s_config.json` 全文只有 `lidar_type`、端口、IP、`pcl_data_type`、`pattern_mode`、`extrinsic_parameter`）。 |
| `cmdline_input_bd_code` | string | `'livox0000000001'` | 广播码。 |

结论：**驱动侧没有任何「选择时间源」「开启 PTP/GPS」的参数**；时间源由设备自身状态（包内 `time_type`）决定，驱动只做读取与分支。

**本仓库内置的纯 Python 驱动**（`src/robot_safecontrol_moveit/livox_mid360/`，官方 C++ 驱动之外的备用通道）遵循同一规则并可被逐字核对：`protocol.py:454-463` `packet_time_base_ns()`（`time_type ∈ {1,2}` 用设备时间戳，否则用传入的 `host_now_ns`），`stream.py:109` 的 `host_now_ns` 缺省取 `time.time_ns()`（Unix epoch 墙钟），`ros_node.py:22-28` 明确写出「header.stamp 是首点时间」并声明「本节点不声称硬件同步」。

### 2.2 Orbbec Gemini 335L（经 OrbbecSDK_ROS2 v2.9.3，`depth_registration:=true`）

**SDK 层事实【一手文档明确】**（包内 SDK 头文件，`orbbec_camera/SDK/include/libobsensor/hpp/Frame.hpp`）：

- `getTimeStampUs()`：**硬件时间戳**，「the time point when the frame was captured by the device, on device clock domain」（设备时钟域）。
- `getSystemTimeStampUs()`：**系统时间戳**，「the time point when the frame was received by the host, on host clock domain」（主机时钟域）。
- `getGlobalTimeStampUs()`：**全局时间戳**，「captured by the device, and has been converted to the host clock domain. The conversion process base on the device timestamp and can eliminate the timer drift of the device」；注意「**disable by default**，未启用时返回 0」，且「**Only some devices support**」，需查 `isGlobalTimestampSupported()`。

**wrapper 层事实【一手源码可见】**（`orbbec_camera/src/ob_camera_node.cpp`）：

- `6059-6070` `getFrameTimestampUs()`：`time_domain_ == "device"` → `getTimeStampUs()`；`== "global"` → `getGlobalTimeStampUs()`；否则 → `getSystemTimeStampUs()`。
- `4647-4657`：参数 `time_domain` 默认 `"global"`，并被归一化到闭集 `{"global","device","system"}`；`4688-4693`：OpenNI 设备强制 `system`；**`global` 且非回放时调用 `device_->enableGlobalTimestamp(true)`**。
- `src/utils.cpp:318-324` `fromUsToROSTime()`：µs → 秒 + 纳秒（整数拆分，无浮点丢失）。
- 深度点云（非注册）：`5657-5663` `getFrameTimestampUs(depth_frame)` → `header.stamp`。
- **注册彩色点云**（本项目用的 `/camera/depth_registered/points`）：`5770-5800` 同样 `frame_timestamp = getFrameTimestampUs(depth_frame)`，`frame_id` 取 `optical_frame_id_[COLOR]`（可被 `cloud_frame_id` 覆盖）。
  → **彩色注册点云的 header stamp 是 `depth_frame` 的时间，不是彩色帧的曝光时间**（与 issue #7 既有结论一致）。帧的时间域语义由 `time_domain` 决定。

**与时间相关的参数（逐项，`launch/gemini_330_series.launch.py` 声明）**：

| 参数 | 类型 | 官方 launch 默认 | 语义 |
|---|---|---|---|
| `time_domain` | string | `'global'` | `global`/`device`/`system`；大小写不敏感。决定上表三个 SDK 接口中取哪一个（`ob_camera_node.cpp:4654-4656, 6059-6070`）。 |
| `enable_sync_host_time` | bool | `false`（launch）；节点 `declare_parameter` 缺省为 `true`（`ob_camera_node_driver.cpp:332`） | 主机↔相机计时器同步。`true` 时无条件调用一次 `device_->timerSyncWithHost()`（`ob_camera_node_driver.cpp:1196-1200`）；**只有 `time_domain != "global"` 时才启动周期重同步定时器**（`1200-1244`），周期由 `time_sync_period` 决定。官方文档：「Set it to false when using global time.」 |
| `time_sync_period` | double | `6.0`（launch）；节点缺省 `60.0`（驱动 `333`） | 周期重同步间隔（秒）。官方文档：「takes effect only when `enable_sync_host_time = true` and `time_domain` is not `global`」。 |
| `timestamp_clock_type` | string | `''` | `realtime`/`monotonic`；空 = 节点**不**显式设置 SDK 时钟类型（SDK 缺省 realtime，`ob_camera_node_driver.cpp:1787-1800`）。 |
| `enable_frame_sync` | bool | `true`（launch；节点 declare 缺省 `false`，`ob_camera_node.cpp:4469`） | 相机**内部**多流帧同步：`pipeline_->enableFrameSync()/disableFrameSync()`（`4049-4055`），并写入点云配置（`1997`）。只影响本机多流，不构成与雷达的同步。 |
| `sync_mode` | string | `'standalone'` | 多相机同步模式（见 3.2）。 |
| `enable_ptp_config` | bool | `false` | 注释「**Only for Gemini 335Le**」（`launch:220`）；官方参数文档「Supported Modules: Gemini 335Le」。**不适用于 335L。** |
| `frame_timestamp_csv_file` | string | `''` | 逐帧时间戳 CSV 输出路径（见 3.2 与第 6 节）。 |
| `depth_registration` / `enable_colored_point_cloud` | bool | `false` / `false` | 必须显式置 `true` 才产生本项目的注册彩色点云（`launch:75,92`）。 |

**未同步情况下的实际域【推断（需实测确认）】**：本项目按官方 launch 默认启动（`enable_sync_host_time=false`、`time_domain=global`、`timestamp_clock_type=''`），设备在 `enableGlobalTimestamp(true)` 下输出**映射到主机时钟域的采集时刻**；`global` 的映射算法与误差上界厂商**未在头文件或文档中给出**（见第 8 节）。

---

## 3. 官方支持的同步手段（逐源）

### 3.1 Livox Mid-360S

**【一手文档明确】**（产品规格页 + 用户手册 + Livox wiki《时间同步说明》）：

- 规格页 `Data Synchronization` 一栏写明：`IEEE 1588-2008 (PTPv2), GPS`。同页 `Frame Rate 10 Hz (typical)`、`Point Rate 200,000 points/s`。
- 用户手册「Timestamp」节：Mid-360S 支持两种同步方式，时间戳为 64 位整数、单位 ns。
  - **IEEE 1588-2008**：Mid-360S 在 PTP 中充当 **ordinary clock（slave）**，**只支持 UDP/IPv4**，支持 `Sync`/`Follow_up`/`Delay_req`/`Delay_resp`。
  - **GPS**：用 PPS 信号 + GPS 报文事件；PPS 端口逻辑与前述 PPS 同步相同，GPRMC 经串口送到对应引脚；PPS 脉冲间隔 t0=1000 ms、高电平持续时间 t1 > 1 µs；除串口外也可以**通过网络包**把每个脉冲的时间戳发给设备。
- 用户手册连接器表（Function cable）：**pin 8 = `LVTTL_IN`（3.3 V LVTTL）秒脉冲 PPS**，**pin 10 = `LVTTL_IN` GPS 输入**；手册另注明 GPS 串口配置为 9600 波特、8 数据位、无校验（wiki 同）。
- wiki《时间同步说明》给出可执行的现场要求：
  - PTP/gPTP 与 GPS 的应用场景区别：PTP/gPTP「只需要在整个网络中有一个 master 时钟设备即可」，**连接 Livox 设备时「通过网线正常连接即可，无需额外接线」**；GPS 需要硬件接入 PPS（及 GPRMC）。
  - PTP 原理为 Delay request-response（two steps），设备为 slave；给出 `Delay`/`Offset` 公式；**PTP 优先级最高**（PTP 与 GPS 同时可用时优先 PTP）；不支持多个主时钟。
  - 主机侧主时钟用 linuxptp（推荐 v3.1.1）`sudo ptp4l -i eth0 -l 6 -m`；gPTP 模式用 `-S`/`-H` 加 `automotive-master.cfg`；并给出 `sudo phc2sys -c eth0 -s CLOCK_REALTIME -O 0` 用来「让系统时间和 PTP 硬件时钟同步」。
  - GPS 时序要求表：t0（相邻上升沿间隔）900~1100 ms，推荐 1000 ms；t1（高电平）> 1 µs，推荐 10~200 ms；t2（GPRMC 传输，9600bps）≈70 ms；t3（GPRMC 相对上升沿延迟）0~900 ms，推荐 0~430 ms；并注明 PPS 斜率建议 > 1 V/µs、硬件接线质量严重影响同步稳定性与精度。
  - 查看是否已同步：读点云包头 `timestamp_type`（1=PTP，2=GPS），或在 Livox Viewer 看 Settings → Sync Type。
  - 串口 GPS 方式「**不需要进行 SDK 软件的配置**」；以太网方式则由上位机解析 GPRMC 后按控制协议下发 UDP 包设置 GPS 时间戳。
- 协议页 key-value 表（只读项）给出可直接当诊断用的字段：`0x8009 local_time_now`（设备当前本地时间）、`0x800A last_sync_time`（**PTP：follow_up 携带的 t1；GPS：同步命令携带的 PPS 上升沿时间**）、`0x800B time_offset`（int64 ns，**设备本地时间 t0 − 源时间 t1**）、`0x800C time_sync_type`（0 无同步 / 1 PTP(IEEE 1588v2.0) / 2 GPS）。

**【一手源码可见】**（SDK2 v1.3.1 / driver 1.2.6）：

- `include/livox_lidar_api.h:350-359` `SetLivoxLidarPpsSyncMode(handle, LivoxLidarPpsSyncMode, ...)`，紧邻注释写着「**mid360s support this function, other not support**」；对比 `360-369` 的 `SetLivoxLidarEscMode` 注明「mid360 and hap lidar does not support this function」。→ **Mid-360S 是唯一被 SDK 明确支持设置 PPS 同步模式的机型**，该能力需要**调用 SDK 写设备参数**（纯软件命令，但会改变设备状态，不属于只读操作）。
- `include/livox_lidar_def.h:262-265`：`kLivoxPpsSyncNormal=0`、`kLivoxPpsSyncSpec=1`；`kKeySetPpsSyncMode = 0x0026`（同文件 `101`），`kKeyTimeSyncType = 0x800C`（同文件 `117`）。
- `include/livox_lidar_api.h:459-468` `SetLivoxLidarRmcSyncTime(handle, rmc, rmc_length, ...)`：「Set LiDAR GPS "GPRMC" string to synchronize the time.」→ **GPS 时间戳也可以由主机经网络下发**（对应 wiki 的「以太网同步」路径），但 PPS 硬件脉冲仍须接 pin 8。
- `samples/livox_lidar_rmc_time_sync/main.cpp:54-85` 是官方端到端示例：主机从 `/dev/ttyUSB0`（9600、8N1）读 GPRMC，在回调里 `SetLivoxLidarRmcSyncTime(...)`。→ **这条路径假设 GPS 模块接在主机侧串口**，而不是接雷达 pin 10。
- `CHANGELOG.md`：SDK2 `1.3.1` 新增「Support Mid-360s Lidar set pps sync mode」「set esc mode」（`7-8`）；`1.2.4` 「Support Mid-360 GPS time synchronization」（`24`）。driver `CHANGELOG.md:30`（1.2.6）「Improve support for gPTP and GPS synchronizations」。

**分类（本项目关心的取舍）**：

| 手段 | 固件已支持 | 是否需要现场接线/硬件 | 是否需要软件改动 | 证据等级 |
|---|---|---|---|---|
| PTP（IEEE1588v2.0 UDP/IPv4） | 是（规格/手册/wiki） | **不需要给雷达接线**；需要网络中有一个 PTP master（通常主机网卡 + linuxptp `ptp4l`） | 主机侧需部署 linuxptp；**并需处理 PTP 域与 CLOCK_REALTIME 的关系（wiki 给 `phc2sys`）** | 一手文档明确 |
| gPTP（L2） | 是（wiki 列为第三种方式；规格页只列 PTPv2/GPS） | 同上（二层） | 同上（`automotive-master.cfg`） | 一手文档明确 |
| GPS：串口 GPRMC + PPS | 是 | **需要接线**：PPS 进 pin 8、GPRMC 进 pin 10（TTL 3.3 V），或经电平转换 | **不需要 SDK 配置** | 一手文档明确 |
| GPS：以太网下发 GPRMC + PPS | 是 | **PPS 仍须接线到 pin 8**；GPRMC 走 UDP | 需用 SDK/协议发 `0x0202`/`SetLivoxLidarRmcSyncTime` | 一手文档明确 + 一手源码可见 |
| 设置 PPS 同步模式（`kKeySetPpsSyncMode` / `SetLivoxLidarPpsSyncMode`） | 是（且仅 Mid-360S 明确支持） | 无（配置命令） | 需调用 SDK（写设备参数，非只读） | 一手源码可见 |
| 「纯软件、不接线、不额外主时钟」的跨源同步 | **不存在** | — | — | 由上述两点与 3.2 推断 |

补充【一手文档明确】：wiki《时间同步说明》另有 **NTP** 一节，但在「应用场景」里描述为「在没有 PTP/gPTP、GPS 的情况下可以使用纯软件的 NTP 同步方式，该方式精度较低」，且该页的 NTP 小节**只有原理、没有给出 Mid-360 / Mid-360S 的配置方法**；而 Mid-360 产品页「支持的时间同步方式」只列 PTP/gPTP/GPS 三种，点云协议 `time_type` 也只有 0/1/2 三个取值。→ **Mid-360S 的 NTP 同步是否可用：未知**（不得据此假设 NTP 可用于本设备）。

### 3.2 Orbbec Gemini 335L

**【一手文档明确】**（OrbbecSDK_ROS2 官方文档「Time Synchronization」节）：

- `enable_sync_host_time`：同步主机时间与相机时间；**默认值由设备 launch 决定，Gemini 330 系列（含 336L）默认 `false`；使用 global 时间时应设为 false**。
- `time_domain`：选择时间戳类型 `device` / `global` / `system`。
- `timestamp_clock_type`：可选 `realtime` / `monotonic`；为空时节点不显式设置 SDK 时钟类型。
- `time_sync_period`：与主机系统同步相机时间的间隔（秒）；**仅当 `enable_sync_host_time = true` 且 `time_domain` 不是 `global` 时生效**。
- `enable_frame_sync`：启用帧同步（相机内部）。
- `frame_timestamp_csv_file`：逐帧时间戳统计 CSV 输出路径。

**【一手源码可见】**：

- `timerSyncWithHost()` 的语义（SDK 头文件 `Device.hpp:814-825`）：「synchronize the timer of the device with the host」；**注意两条**：(a) 若流已启动，计时器同步后**连续帧的时间戳可能跳变一次**；(b) 「Due to the timer of device is not high-accuracy, the timestamp ... will drift after a long time. User can call this function periodically to avoid the timestamp drift, **the recommended interval time is 60 minutes**」。→ 这是**主机↔单台相机**的计时器同步，不是跨设备同步；且官方建议的 60 分钟周期与 launch 默认 `time_sync_period=6.0 s` 相差 600 倍（两者用途不同，但不能把更频繁的调用当成更准的时间戳）。
- `frame_timestamp_csv_logger.cpp:527-551` 的 CSV 表头（每路 color/depth 一组）：`*_sdk_frame_index`、`*_hardware_frame_number`、`*_sensor_ts_sec/_delta_us`、`*_device_ts_sec/_delta_us`、`*_global_ts_sec/_delta_us`、`*_system_ts_sec/_delta_us`、`*_arrival_steady_delta_us`、`*_publish_steady_delta_us`、`*_arrival_to_publish_steady_us`、`*_sdk_delay_from_global_us`、`*_sdk_delay_from_system_us`。→ **厂商自带一个可同时记录三域时间戳、单调到达时刻与发布时刻的测量工具**，是本项目做现场时间核验的现成一手手段。
- 多相机同步（`sync_mode`）官方文档给出模式表，并明确相机有 **8-pin 同步接口**：`free_run`、`standalone`（默认，内置 RGBD 帧同步，8-pin 默认不输出信号）、`primary`（8-pin **输出**信号给外部设备）、`secondary`（被动，须外部连续触发信号且与设定帧率匹配，无外部触发则停流）、`secondary_synced`（无外部触发时按内部触发采集）、`hardware_triggering`、`software_triggering`。→ **该接口是相机↔相机/外部触发器的硬触发路径**；官方文档描述的场景是「多台相机同步」，**没有任何一处把 8-pin 接口描述为可与 LiDAR PPS/GPS 共用的时间基准输入**。
- 触发相关的其余参数（同一文档）：`depth_delay_us`/`color_delay_us`、`trigger2image_delay_us`、`trigger_out_delay_us`、`trigger_out_enabled`、`software_trigger_enabled`/`software_trigger_period`、`frames_per_trigger`、`sync_io_voltage_level`（仅 Gemini 301 系列）、`intra_camera_sync_reference`（**Gemini 330 系列**在 software/hardware trigger 模式下设定相机内同步参考点，可选 `Start`/`Middle`/`End`）。
- `enable_ptp_config`：launch 注释「Only for Gemini 335Le」+ 官方文档「Supported Modules: Gemini 335Le」。→ **335L 不支持该 PTP 配置项**（一手源码可见 + 一手文档明确）。

**分类**：

| 手段 | 固件/SDK 支持 | 需要现场接线 | 纯软件 | 证据等级 |
|---|---|---|---|---|
| `time_domain=system`（主机收到的帧时刻） | 是 | 否 | 是（纯 launch 参数） | 一手源码可见 |
| `time_domain=device`（设备时钟域采集时刻） | 是 | 否 | 是 | 一手源码可见 |
| `time_domain=global`（设备采集时刻映射到主机域，消除设备漂移） | 文档说「only some devices」，需 `isGlobalTimestampSupported()` 查询 | 否 | 是 | 一手文档明确 + 一手源码可见 |
| `enable_sync_host_time` + `timerSyncWithHost()`（主机↔相机计时器对齐） | 是 | 否 | 是（但流已启动时时间戳可能跳变一次） | 一手源码可见 |
| `enable_frame_sync`（相机内部多流同步） | 是 | 否 | 是 | 一手源码可见 |
| 8-pin 硬触发（`primary`/`secondary*`/`hardware_triggering`） | 是 | **需要接线（相机↔相机/触发器）** | 否 | 一手文档明确 |
| PTP（`enable_ptp_config`） | **仅 335Le**，335L 不适用 | — | — | 一手源码可见 + 一手文档明确 |
| GPS/PPS 外部时间基准输入 | **本次未找到任何官方文档或源码证据** | — | — | 未知（见第 8 节） |

---

## 4. ROS 2 Humble 可用的时间机制与它们的隐含假设

### 4.1 `PointCloud2.header.stamp` 的规范语义

**【一手文档明确】**（Humble `common_interfaces`）：

- `std_msgs/Header.msg`：只有 `builtin_interfaces/Time stamp` 与 `string frame_id`，注释仅说明「Two-integer timestamp that is expressed as seconds and nanoseconds」——**规范不限定时间域，也不要求与接收节点时钟同源**。
- `sensor_msgs/PointCloud2.msg`：`header` 上方注释为「**Time of sensor data acquisition**, and the coordinate frame ID (for 3d points)」。
  → 规范语义是「采样时刻」，不是「发布时刻」「接收时刻」。但规范**无法强制**驱动真的填采样时刻：Livox 未同步时填的是主机处理包时刻（2.1），Orbbec `system` 域填的是主机收到帧时刻（2.2）。**「字段名相同、规范语义相同」不等于「实际语义相同」**，这是本项目最容易踩的坑。

### 4.2 `use_sim_time` 与 `/clock`

**【一手文档明确】**（ROS 2 官方设计文档《Clock and Time》）：

- ROS Time 在未启用时间源时报 SystemTime；**「ROSTime is considered active when the parameter `use_sim_time` is set on the node」**。
- `/clock` 话题若有发布者，「it will override the system time」；若一直没收到，ROS 时间返回 **0**，且「A time value of zero should be considered an error meaning that time is uninitialized」。
- 时间可以**回跳**（日志回放场景），回跳时会先调用回调再做后续查询，开发者必须处理不连续。
- Steady Time 的用途被明确点名：「**hardware drivers which are interacting with peripherals with hardware timeouts**」——硬件超时不应依赖可被暂停/回跳的 ROS 时间。

**【一手源码可见】**：`rclpy/rclpy/time_source.py:82-96` 会在节点上自动声明 `use_sim_time`（缺省 `False`，即默认墙钟）；`rclpy/rclpy/node.py:222-225` 在 `Node.__init__` 内创建 TimeSource 并挂到节点时钟。

**隐含假设**：`perception_bridge` 的 `now_s = self.get_clock().now()`（`perception_bridge.py:396`）只有在**两路驱动的 header stamp 与节点 ROS 时钟同域**时，才等价于「数据年龄」。一旦启用 `use_sim_time` 而驱动仍发墙钟（或反之），年龄会立刻变成巨大值或负值。本仓库当前只在 `launch/mujoco_transition_final.launch.py:133` 给 `move_group` 显式设 `use_sim_time: False`，未对 `perception_bridge` 设置，走 rclpy 默认墙钟。

### 4.3 `tf2` 的时间戳语义

**【一手文档明确】**（`tf2_ros/include/tf2_ros/buffer.hpp:83`、`tf2/include/tf2/buffer_core.hpp:132-142`）：

- `lookupTransform(target, source, time)`：`time` 为「The time at which the value of the transform is desired. **(0 will get the latest)**」；可能抛 `LookupException`/`ConnectivityException`/`ExtrapolationException`。
- `lookupTransform(target, target_time, source, source_time, fixed_frame)`：同样「0 will get the latest」。

**【一手源码可见】**（`tf2/src/cache.cpp`）：

- `findClosest`（`102-162`）：`time == 0` 直接返回最新样本；若请求时刻晚于缓存最新 → `createExtrapolationException2`「Lookup would require extrapolation into the future」；早于缓存最早 → 「into the past」。
- `getData`（`190-211`）：命中两个相邻样本且 frame 对相同时调用 `interpolate()` **做线性/球面插值**；否则取较旧样本。→ **tf2 会对时变变换插值，但它只解决「同一时刻的几何关系」，完全不解决「两个时钟域谁对谁错」。**
- tf2 的 `/tf_static` 与静态变换不受时间查询影响，但**静态外参不能修正「时间戳域错了」这类错误**。

**隐含假设（本项目相关）**：`perception_bridge._sensor_to_world()`（`perception_bridge.py:295-326`）在 `use_tf=true` 时先按 `msg.header.stamp` 查 TF，失败后**静默回退到最新可用 TF（`Time()`）**。这意味着：**时间戳域错误的后果会被 TF 回退掩盖**——查不到就退到「最新」，看起来照常运行，但几何与时间已经不同步（尤其是 J1 直线导轨运动时）。这是「未对齐会静默降级」的一条具体路径。

### 4.4 `message_filters` 的 `ExactTime` / `ApproximateTime`

**【一手源码可见】**（Humble `message_filters`，`src/message_filters/__init__.py`）：

- `TimeSynchronizer`（`270-330`）：从每个输入的 `msg.header.stamp` 取 `Time.from_msg()`，把 `stamp.nanoseconds` 当字典键，**要求所有输入存在完全相同的纳秒键**才回调（`common = set.intersection(...)`）。→ 即 C++ 侧的 `ExactTime` 语义：**逐纳秒相等**。
- `ApproximateTimeSynchronizer`（`332-407`）：构造参数 `queue_size`（每路缓存多少条）与 `slop`（秒）；对每条新消息，在其它队列里只保留 `|Δstamp| <= slop` 的候选，再要求候选组合里 `max(stamp) - min(stamp) < slop`，命中即回调。
- 两者都**只比较 header 里的 stamp 数值**；源码中没有任何时钟校准、偏移估计或采样时刻重建逻辑。`allow_headerless=False` 时**没有 header 的消息直接丢弃**。

**隐含假设**：`ApproximateTime` 的 `slop` 是「同一时钟域内的相对差阈值」，它**隐含假设两路 stamp 已经同源**。若两源不同域（例如一方 PTP、一方主机墙钟），`slop` 既可能永远不满足（两路永远配不上，或产出错配），也可能因为域偏移恰好落在 slop 内而**把错配当成功配对**。**「配对成功」不是「已同步」的证据**（与 issue #7 既有结论一致）。

### 4.5 录制/回放侧的时间（影响「离线可验证」的边界）

**【一手源码可见】**（rosbag2 Humble）：

- `rosbag2_storage/include/rosbag2_storage/serialized_bag_message.hpp:24-30`：每条记录只有 `serialized_data`、`time_stamp`、`topic_name`。
- `rosbag2_transport/src/rosbag2_transport/recorder.cpp:387`：`writer_->write(message, topic_name, topic_type, this->get_clock()->now())`。
  → **bag 里的索引时间戳是「录制节点时钟的接收时刻」，消息体内部原有 header stamp 原样保留**。因此：录 bag 可以用来回放**原始 header 时间戳序列**，但不能把 bag 时间戳当成传感器时间；两者要分别记录（第 6 节）。

---

## 5. 对本项目的影响：`perception_bridge` 的融合与 freshness 逻辑在「时间未对齐」时会怎样

先说清当前配置下真实发生了什么（两点很关键）：

1. **当前两源「名义上同域、实际语义不同」**：相机 `time_domain=global` 得到的是**采集时刻映射到主机时钟域**（`Frame.hpp`）；雷达在设备未同步（`time_type=0`）时，header stamp 是**驱动处理包时的主机墙钟**（`pub_handler.cpp:265-275`）。两者都是主机时钟域，所以现有逻辑**不会报错**，`max_inter_sensor_dt_s=0.1` 也能通过——但一个是「采样时刻」，一个是「处理时刻」，差值里混着网络传输、驱动排队与批处理窗口，**不能当作采集时差**。
2. **一旦真的按 3.1 把雷达做成 PTP/GPS 同步，立刻变成真正的跨域问题**：PTP 域只有在主机用 `phc2sys -s CLOCK_REALTIME` 之类手段与 `CLOCK_REALTIME` 绑定时才与相机 `global` 可比（wiki 明确给出这一步）；GPS 域的时间语义（是否含跳秒、是否 Unix epoch）厂商文档未说明（见第 8 节）。此时若沿用现有逻辑，会出现「stamp 落在未来/过去」「dt 恒大于阈值导致永久单源降级」等现象，而**代码里没有一处会因此报错**。

逐条对应到代码（行号以当前工作区为准）：

| 现状 | 位置 | 时间未对齐时的后果 |
|---|---|---|
| 把 header stamp 直接当「采集瞬间」，与 ROS `now()` 相减得 age | `perception_bridge.py:350-351`（取 stamp）、`:396`（`now_s`）、`:404`（`fuse(now_s)`）；`fusion_engine.py:168-169,180-181` | age 是「ROS 时钟 − 对方时间域」的混合差值。域偏移会整体加到 age 上（正偏移 → 误判超龄丢源；负偏移 → 误判新鲜） |
| alive/age 只做 `< max_age`，**没有非负/有限性检查** | `fusion_engine.py:169,181,200,207`（`camera_age < camera_max_age_s` 等） | 未来时间戳（负 age）会被判 alive 且 `perception_valid=1`；域偏移造成的负 age 同样放行（issue #7 已复现） |
| `camera_age`/`lidar_age` 用 **buffer 最新帧**，配对后实际用的是**更旧的帧** | `fusion_engine.py:166-169`（报告）vs `192-203`（配对选中帧才 `cam_age = now - cam_stamp`） | 报告 age 小于实际贡献帧年龄 → 诊断字段低估真实障碍年龄（安全方向相反） |
| `fusion_stamp = max(stamps_used)` | `fusion_engine.py:224-231`，输出到 `:282-286`；`perception_bridge.py:423-438`（status[6]/[7]） | 融合时间戳掩盖更旧的一路；`fusion_age`/`perception_valid`（`fusion_engine.py:284-286`）都基于这个偏新的戳 → 「最新一路」可以不断刷新整体新鲜度 |
| `max_inter_sensor_dt_s` 只作为「时间差过大就丢较旧一路」的降级门 | `fusion_engine.py:212-221`；配置 `perception_runtime.yaml:67`、`perception_dual_sensor_real.yaml:50`、`portable_oscbf/config/obstacle_params.yaml:31` | 该门只约束标量差。域偏移小于阈值时**静默错配**；域偏移大于阈值时**永久退化为单源**且没有任何显式告警字段（status 只剩 `camera_used`/`lidar_used` 量） |
| 重复帧保护（stamp 组合相同 → 跳过） | `fusion_engine.py:233-240` → `_empty_result`（`309-342`，`source_count=0`、`perception_valid=0`、`fusion_stamp=0`） | 分不清「没有新帧」和「旧观测已失效」。跨域/时间跳变导致组合反复不变时，会周期性把 `perception_valid` 打成 0 |
| 三层占据/静态确认计时直接吃融合时间戳 | `fusion_engine.py:263-265`（`_occupancy_tracker.update(merged, fusion_stamp)`）；`static_occupancy.py:87-101`（`first_seen`/`last_seen`/`timed_out`/`static_mask = (stamp - first_seen) >= static_confirm_s`） | **混合域的时间戳序列会污染「持续占据时长」的语义**：向前跳变会让体素瞬间满足 `static_confirm_s`（提前进 static 层，进 ESDF）；向后跳变会让 `stamp - last_seen` 恒为负 → `timed_out` 永不触发 → 陈旧占据不清除（ESDF 保留鬼影障碍） |
| 聚类 `dt_s` 取自被提前覆盖的 `_prev_fusion_stamps` | `fusion_engine.py:233-240`（先更新）与 `:269-278`（后用） | 传入聚类的 dt 可能是 0/源间差而非帧间差（issue #7 2026-09-11 已实测），速度估计不可信；**时间未对齐时 `dt_s` 再无物理意义** |
| `fusion_stamp` 以 float32 数组发布 | `perception_bridge.py:423-438`（`Float32MultiArray.data[6]`） | 2026 年的 epoch 秒在 float32 下的间隔为 **128 s**（本次用 `np.spacing` 实测：`np.float32(1789358587.123)` 的 spacing = 128.0），status 里的 `fusion_stamp` 无法承担毫秒级绝对时间；相对的 `*_age` 字段是小量，精度够用 |
| `/perception/tracks` 不带任何采集/接收时刻 | `perception_bridge.py:490-503`（8 槽 × 10 float，无时间字段）；`oscbf_controller.py:503-518`（只缓存几何/速度/enabled/d_safe/alpha） | 控制器断流后无法自行让旧观测过期；两源时间未对齐无法在消费端补救（与 #7 结论一致） |
| TF 查询按 stamp，失败静默回退最新 | `perception_bridge.py:303-326`（`except` 后 `lookup_transform(..., Time())`） | 时间戳域错误 → TF 查不到 → 静默用最新几何，故障不可见（4.3） |

**一句话总结**：当前实现把「两路 header stamp 与 ROS 时钟同域、且都近似等于采集时刻」当成了**未声明的公理**。在都不做硬同步的现状下，这条公理「看起来成立」（都是主机域），所以系统能跑、参数也能通过；**它的失效方式是静默的**——诊断字段低估年龄、静态/动态分层计时被污染、单源降级不告警、TF 回退掩盖域错误。这正是「还没有做对齐」在本仓库里的具体代价。

---

## 6. 离线可验证 vs 必须实机验证的分界

「离线」= 不依赖设备固件同步状态、PPS/PTP 接线与真实曝光/扫描时序，只用纯 Python 引擎、合成数据、录制 bag 或注入时间戳即可判定。

### 6.1 可以完全离线判定

| 判定项 | 可离线的原因 | 入口/证据 |
|---|---|---|
| 融合逻辑对未来/零/非有限时间戳的处置 | `FusionEngine` 是零 ROS 依赖的纯 Python（`portable_oscbf/work/fusion_engine.py` 文件头），可直接注入 stamp | 现有 `portable_oscbf/tests/test_dual_sensor_fusion.py`（含 stale/dead/stop/recover、重复帧保护、占据计时等用例）+ 新增时间故障用例 |
| 「新源到达不能刷新旧源年龄」「重发/预测/空槽不能延长旧覆盖」 | 纯粹是同一个 now⇒age 函数的行为 | 同上（issue #7 交接口径） |
| 跨域时间戳下的错配/降级/计时污染（本文第 5 节各条） | 只要把两路 stamp 加上已知域偏移（含负偏移、跳变），即可离线复现全部失效模式 | `FusionEngine` + `OccupancyTracker` 单元级注入（合成双时钟映射） |
| `fusion_stamp` 经 float32 的精度损失 | 纯数值事实（本次已用 `np.spacing` 复核为 128 s） | 数值检查 + 消息编解码测试 |
| 消息 schema/字段完整性（逐源 age、覆盖、无效原因、替代覆盖） | 只需合成消息与本地编解码 | OFF-06 离线验收项 |
| bag 回放的 header 时间戳序列与 bag 索引时间戳的区分 | rosbag2 语义已核实（recorder 写 `get_clock()->now()`，消息体内 header 原样保留） | 任意已录 bag；独立只读记录器 |
| `use_sim_time` / `/clock` 路径下的节点行为（暂停、回跳、零值） | 可用只发 `/clock` 的假时钟 + 合成点云 | ROS 图本地测试，不需要真实传感器 |
| TF 语义（0=latest、外推异常、插值）在本地复现 | 可本地发 `/tf`、`/tf_static` | 不需要传感器 |

### 6.2 必须实机（设备/接线/固件）才能判定

| 判定项 | 为什么离线不行 | 可用的一手手段 |
|---|---|---|
| 雷达当前是否真的在同步、同步方式是什么（`time_type` = 0/1/2） | 由设备固件状态决定；本文只能给出判定字段，不能给出设备当前值 | 点云包头 `time_type`（wiki 协议页）；key-value `0x800C`；Livox Viewer 的 Sync Type（wiki） |
| PTP 是否锁定、master 是否工作、PTP 域与 `CLOCK_REALTIME` 的差 | 需要真实网卡、linuxptp 运行与网络 | `ptp4l` 输出 + wiki 的 `phc2sys` 步骤；key-value `0x800A last_sync_time`、`0x800B time_offset` |
| GPS 路径的 PPS/GPRMC 电气与时间质量 | 需要 GPS 模块、接线、TTL 3.3 V 电平、斜率与 t0/t1/t2/t3 窗口 | 手册连接器表、wiki 时序表；示波器 + `0x800B time_offset` |
| GPS 同步后时间戳的绝对语义（是否含跳秒、epoch 定义） | 厂商文档未说明 | **未知**，须实测（与主机 UTC 比对） |
| 相机 `global` 映射误差与稳定性（设备→主机域的换算精度） | 头文件只说「转换」并「消除漂移」，未给误差上界；且限部分机型支持 | `frame_timestamp_csv_file` 的 `*_global_ts_*`、`*_device_ts_*`、`*_system_ts_*`、`*_arrival_steady_delta_us` 列；`isGlobalTimestampSupported()` |
| 335L 上 `time_domain=device`/`global`/`system` 各自的实际读数与相互关系 | 需要设备在线 | 同上 CSV 工具 |
| 曝光/读出语义：时间戳对应曝光开始/中间/结束，曝光时长 | 需要真实曝光；`intra_camera_sync_reference` 仅在 software/hardware trigger 模式下可用，`standalone` 下的默认参考点未文档化 | **未知**，须实测（光源/事件法） |
| 雷达扫描跨度与「批首点」实际覆盖的时间区间（10 Hz × `publish_freq` 累积） | 依赖真实累积窗口与网络丢包 | 逐点 `timestamp` 字段（FLOAT64 offset 18）与 `time_interval`；实测每帧首末点差 |
| 跨源真实采集时刻差（含曝光、扫描、传输） | 需要共同可观测事件 + 仪器不确定性评估 | 需实测；不得用「到达时间差」代替 |
| 长期漂移、重同步、设备重启后的时间跳变与重连行为 | 需要长时间真实运行 | issue #7 交接的验收清单 + CSV/记录器 |
| 相机与雷达是否能共用同一硬触发/PPS 基准 | 需要接线；且厂商文档只描述相机↔相机 8-pin 触发 | 需实测/厂商确认（本文未找到 335L 的 PPS/GPS 输入证据） |

### 6.3 明确不能作为「已对齐」证据的东西

- 两路消息到达时间接近、`ApproximateTime` 配对成功、`frame_id` 相同、`max_inter_sensor_dt_s` 通过——**都不构成硬同步证据**（4.4）。
- 单次 bag 回放或一次采样窗口内的 header 无回退（issue #31 已记录约 12 s 采样）——不能推理长期上界。
- `enable_sync_host_time` / `timerSyncWithHost()` 只是主机↔单相机计时器对齐，与雷达无关（3.2）。
- 仿真（`use_sim_time` + `/clock`）下「对齐成功」只说明软件逻辑在给定注入时间下自洽。

---

## 7. 对 OFF-01 / OFF-06 / #7 的具体输入

### 7.1 OFF-01（标定记录加载与运行时身份统一 / 诊断新鲜度字段），来源票 T7（#27）

- **每个源的 `time_domain` 是标定身份之外的另一个「身份」字段**：同一份外参记录若搭配了两路不同时间域，其运行语义不同。建议诊断记录里显式携带逐源的 `time_domain`/`time_kind`（`device|global|system`、`ptp|gps|none|host_receive`），并且**与 `calibration_id` 一样按实际生效的快照上报**，而不是照抄配置默认值（参考 `calibration-ssot-runtime-contract-20260911.md` 的「同一 bytes 快照」原则）。
- **诊断自身的新鲜度必须用单调时钟，不能用 header stamp**：ROS 时间可暂停/回跳/为零（4.2）。标定诊断的 `Header.stamp` 只用于「有共同时间基准时的报告年龄检查」；消费端应另用本机单调时钟判断「距上次收到当前实例报告」的间隔。
- **不同域的绝对时间不得相加相减**：逐源诊断应分列「采集→消费」的保守年龄与「收到→消费」的纯链内时延（后者用同一单调时钟，与传感器域无关）。
- **诊断里必须能区分「没有新帧」与「旧观测已失效」**：现有 `_empty_result` 把两者都变成 `perception_valid=0`（`fusion_engine.py:309-342`），诊断字段需要显式 invalid reason（含 `no_new_frame`、`time_domain_mismatch`、`future_stamp`、`clock_jump`、`session_changed`）。

### 7.2 OFF-06（逐源时间与显式障碍观测契约），来源票 #7、T1（#21）

逐源字段建议（每条都能用本文第 2 节的机制解释、并能在第 6.1 节用合成数据验收）：

| 字段 | 取值来源（一手） | 缺失时行为 |
|---|---|---|
| `source_id` / `device_identity` | 设备 SN（Livox 广播发现 / Orbbec serial）、驱动版本 | 显式 unknown |
| `time_domain` | Livox：包内 `time_type`（0/1/2）；Orbbec：`time_domain` 参数 + `getGlobalTimeStampUs` 是否非零 | 域未知 → 不得判定「已证明新鲜」 |
| `time_kind` | 采样时刻（相机 global/device）vs 主机处理时刻（雷达未同步）/ 主机接收时刻（相机 system）——**三者语义不同，必须分列** | 不假设等同于采集时刻 |
| `raw_stamp` + `clock_error_bound` | 设备原始时间戳（雷达逐点 `timestamp` 或包 `timestamp`）+ 映射误差上界 | 上界未知 → 年龄只能报「≥」，不能报精确值 |
| `sample_start/end` | 雷达：批首点时间 + 逐点跨度（`time_interval`/`point_interval`）；相机：曝光/读出语义（**未知，须实测**） | 只有 header 时须另附扫描/曝光宽度界，不能当瞬时采样 |
| `frame_sequence` | 雷达：`udp_cnt`/`frame_cnt` 或订阅序号（须区分，不能用订阅计数冒充设备帧号）；相机：`frame_index`/`hardware_frame_number`（CSV 工具可直接给） | 缺失如实记 unknown |
| `recv_steady` / `processed_steady` / `publish_steady` | 本机单调时钟 | — |
| `invalid_reason` | 见 7.1 | — |

契约层面的硬要求（直接来自本文第 4、5 节）：

- **任何一段只接受「同一可信时间域」的年龄运算**；域未知/不一致时先判无效，不做 `max(0, ...)` 掩盖。
- `fusion_stamp` 禁止用 `max()` 掩盖更旧的一路；输出必须能表达**最旧必要贡献**的年龄。
- 绝对时间用 int64 ns（或 sec+nanosec）传输；**禁止经过 float32**（现状 128 s 分辨率）。
- 「无新帧」「关联失败」「空帧」都不得等价为空闲或有效覆盖。
- 观测/覆盖/速度未知状态必须显式，速度缺失不得当 0（`dynamic_clustering.py` 的新 track 零速度只是初始化）。

### 7.3 #7（感知时间同步与延迟模型）

- 本文补齐了 #7 尚未固化的**逐源机制级证据**：雷达 header stamp 的两条分支（`pub_handler.cpp:265-275`）、批首点语义（`pub_handler.cpp:281-286` + `lddc.cpp:310,316`）、相机的三时间域接口与注册点云取 depth 帧时间（`ob_camera_node.cpp:6059-6070, 5770-5800`）、官方同步手段与「需接线/纯软件」分界（第 3 节）。
- #7 正文与评论中三个待补项，本文只能部分推进、其余仍缺：
  1. **实际时钟映射**：仍缺「雷达当前 `time_type`」与「相机 `global` 映射误差上界」的现场值（第 6.2 节给出测量手段：包 `time_type`、key-value `0x800A/0x800B/0x800C`、Orbbec CSV）。
  2. **曝光/扫描语义**：扫描跨度可由逐点 `timestamp` 与 `time_interval` 现场量出（离线不可得）；**相机曝光参考点与时长仍未文档化**（`intra_camera_sync_reference` 只在触发模式下可用）。
  3. **运动界与最终 `age_warn`/`age_stop`**：本文不提供数值。新增的一条工程约束是：**在雷达切换时间域（未同步 → PTP/GPS）前后，0.5 s/1.0 s 这类现值不再可比**，因为 age 的时间原点从「主机处理时刻」变成了「设备采样时刻」。
- 新增一条本文独立得出的风险项（建议进 #7 的风险清单）：**跨域瞬间的静默失效**。切换时间域时必须清空缓冲/配对状态并重新验证，否则 `OccupancyTracker` 的 `first_seen`/`last_seen`（`static_occupancy.py:87-101`）会带着旧域的时间戳继续计时，可能瞬时把未确认体素判成 static，或让过期体素永不过期。

---

## 8. 未知项清单（明确写「未知」）

以下条目**在没有现场测量或厂商补充资料前，一律不得当作已知事实，也不得填估算值**：

1. 本机这台 Mid-360S **当前实际的 `time_type`**（0/1/2）与固件版本对应的同步能力：**未知**（仓库既有记录只有固件版本字节与型号，未见 `time_type` 记录）。
2. **Mid-360S 是否支持 NTP 时间同步**：**未知**（wiki 有 NTP 章节但无该机型配置方法；产品页与协议 `time_type` 只列 PTP/gPTP/GPS）。
3. 未同步（`time_type=0`）时驱动所用 `high_resolution_clock` 的 **epoch 与单位在目标机上的最终取值**：本机实测为 Unix epoch 纳秒 + `high_resolution_clock == system_clock`，但 C++ 标准不保证、驱动也未做 `duration_cast`；**换平台/标准库/编译选项后未知**。
4. 设备从「激光采样」到「发出 UDP 包」的内部时延：**未知**（厂商未公开）。
5. GPS 同步时时间戳的绝对语义（是否含 UTC 跳秒、是否 Unix epoch）：**未知**（厂商只给 2000–2037 有效范围）。
6. PTP 域与主机 `CLOCK_REALTIME` 的差，以及**不执行 `phc2sys` 时的实际偏差量**：**未知**（wiki 只给出方法，未给数值要求）。
7. Orbbec `global` 时间戳的**映射误差上界**与长期稳定性：**未知**（`Frame.hpp` 只描述机制）。
8. 本项目这台 **Gemini 335L 是否 `isGlobalTimestampSupported()`**：**未知**（需要设备在线查询；若返回 false，`global` 域会返回 0，而代码路径不会因此报错——`getFrameTimestampUs` 直接返回该值）。
9. 335L 在 `sync_mode=standalone` 下，帧时间戳指向**曝光开始/中间/结束**中的哪一个，以及典型曝光时长：**未知**（`intra_camera_sync_reference` 仅在 software/hardware trigger 模式生效）。
10. 335L 是否存在可接收 PPS/GPS 等**外部时间基准输入**的引脚（8-pin 接口是否可用于雷达共基准）：**未知**（官方文档只描述相机↔相机触发；本次未找到 335L 的 PPS/GPS 输入证据）。
11. 注册彩色点云中**配对彩色帧自身的曝光时间与时间残差**：**未知**（`ob_camera_node.cpp:5770-5800` 只用 depth 帧时间）。
12. 两路**冷启动后的首次时钟一致性**、重连/重启后的重新同步行为、长时间漂移量：**未知**（需现场长时间采集）。
13. 逐源「时钟误差上界」`epsilon_t,i`、工况障碍速度界、几何误差界、控制执行时延与停止位移：**未知**（issue #7 仍缺，本文不提供）。

---

## 9. 来源列表

**Livox（官方）**

1. Livox ROS driver2 `1.2.6`（commit `13eb05e4e6dd7a765b934d0c5fd6236676a57b49`）
   - `src/comm/pub_handler.cpp`：https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/src/comm/pub_handler.cpp （重点 `105-109`、`142-144`、`166-207`、`265-275`、`281-286`）
   - `src/comm/pub_handler.h:121-123`：https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/src/comm/pub_handler.h
   - `src/comm/comm.h:94-99,133-146`：https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/src/comm/comm.h
   - `src/lddc.cpp`（`284-296`、`298-333`）：https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/src/lddc.cpp
   - `src/lds.cpp:134,144`、`src/comm/ldq.cpp:139`：https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/src/lds.cpp 、https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/src/comm/ldq.cpp
   - `launch_ROS2/msg_MID360s_launch.py:8-14`：https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/launch_ROS2/msg_MID360s_launch.py
   - `config/MID360s_config.json`：https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/config/MID360s_config.json
   - `CHANGELOG.md:30`：https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/CHANGELOG.md
2. Livox SDK2 `v1.3.1`（commit `f5d9375f84efe2b15bc0a052d3e18482ed13adf4`）
   - `include/livox_lidar_def.h`（`129-142` 包结构、`262-265` PPS 模式枚举、`101` `kKeySetPpsSyncMode`、`117` `kKeyTimeSyncType`）：https://github.com/Livox-SDK/Livox-SDK2/blob/f5d9375f84efe2b15bc0a052d3e18482ed13adf4/include/livox_lidar_def.h
   - `include/livox_lidar_api.h`（`350-359` PpsSyncMode、`360-369` EscMode、`459-468` RmcSyncTime）：https://github.com/Livox-SDK/Livox-SDK2/blob/f5d9375f84efe2b15bc0a052d3e18482ed13adf4/include/livox_lidar_api.h
   - `samples/livox_lidar_rmc_time_sync/main.cpp`、`mid360s_config.json`：https://github.com/Livox-SDK/Livox-SDK2/blob/f5d9375f84efe2b15bc0a052d3e18482ed13adf4/samples/livox_lidar_rmc_time_sync/main.cpp
   - `CHANGELOG.md:7-8,24`：https://github.com/Livox-SDK/Livox-SDK2/blob/f5d9375f84efe2b15bc0a052d3e18482ed13adf4/CHANGELOG.md
3. Livox Mid-360S 规格页（`Data Synchronization: IEEE 1588-2008 (PTPv2), GPS`）：https://www.livoxtech.com/mid-360s/specs
4. Livox Mid-360S 用户手册（2026-06-01，Timestamp 节与连接器引脚表）：https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid-360S/UM/20260601/Livox_Mid-360s_User_Manual_en.pdf
5. Livox wiki《时间同步说明》（PTP/gPTP/GPS/NTP、linuxptp 与 phc2sys、GPS 时序表）：https://livox-wiki-cn.readthedocs.io/zh-cn/latest/tutorials/new_product/common/time_sync.html
6. Livox wiki《激光雷达通信协议–Mid360》（点云包头、时间戳类型、key-value 表含 `0x8009/0x800A/0x800B/0x800C`）：https://livox-wiki-cn.readthedocs.io/zh-cn/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html
7. Livox wiki Mid-360 页「支持的时间同步方式」（PTP/gPTP/GPS 三选一及其限制）：https://livox-wiki-cn.readthedocs.io/zh-cn/latest/tutorials/new_product/mid360/mid360.html

**Orbbec（官方）**

8. OrbbecSDK_ROS2 `v2.9.3`（commit `ce08bce25f7a0a6fe939ece87ec945447109581f`）
   - `orbbec_camera/src/ob_camera_node.cpp`（`4466-4470`、`4534`、`4541`、`4647-4693`、`5657-5663`、`5770-5800`、`6059-6070`）：https://github.com/orbbec/OrbbecSDK_ROS2/blob/ce08bce25f7a0a6fe939ece87ec945447109581f/orbbec_camera/src/ob_camera_node.cpp
   - `orbbec_camera/src/ob_camera_node_driver.cpp`（`316-336`、`1196-1244`、`1787-1800`）：https://github.com/orbbec/OrbbecSDK_ROS2/blob/ce08bce25f7a0a6fe939ece87ec945447109581f/orbbec_camera/src/ob_camera_node_driver.cpp
   - `orbbec_camera/src/frame_timestamp_csv_logger.cpp:527-551`：https://github.com/orbbec/OrbbecSDK_ROS2/blob/ce08bce25f7a0a6fe939ece87ec945447109581f/orbbec_camera/src/frame_timestamp_csv_logger.cpp
   - `orbbec_camera/src/utils.cpp:318-324`：https://github.com/orbbec/OrbbecSDK_ROS2/blob/ce08bce25f7a0a6fe939ece87ec945447109581f/orbbec_camera/src/utils.cpp
   - 包内 SDK 头文件 `orbbec_camera/SDK/include/libobsensor/hpp/Frame.hpp`（三时间戳语义）：https://github.com/orbbec/OrbbecSDK_ROS2/blob/ce08bce25f7a0a6fe939ece87ec945447109581f/orbbec_camera/SDK/include/libobsensor/hpp/Frame.hpp
   - 包内 SDK 头文件 `orbbec_camera/SDK/include/libobsensor/hpp/Device.hpp:814-825`（`timerSyncWithHost`）：https://github.com/orbbec/OrbbecSDK_ROS2/blob/ce08bce25f7a0a6fe939ece87ec945447109581f/orbbec_camera/SDK/include/libobsensor/hpp/Device.hpp
   - `orbbec_camera/launch/gemini_330_series.launch.py`（`39-44`、`75`、`92`、`210`、`220-221`、`272-277`）：https://github.com/orbbec/OrbbecSDK_ROS2/blob/ce08bce25f7a0a6fe939ece87ec945447109581f/orbbec_camera/launch/gemini_330_series.launch.py
9. Orbbec 官方文档《Launch parameters》→ Time Synchronization 节：https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/4_application_guide/launch_parameters.html
10. Orbbec 官方文档《Multi_camera synced Instructions》（8-pin 同步接口与 `sync_mode` 模式表）：https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/5_advanced_guide/multi_camera/multi_camera_synced.html

**ROS 2 / 标准库（官方）**

11. `std_msgs/Header.msg` 与 `sensor_msgs/PointCloud2.msg`（Humble）：https://github.com/ros2/common_interfaces/blob/humble/std_msgs/msg/Header.msg 、https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/PointCloud2.msg
12. ROS 2 官方设计文档《Clock and Time》：https://design.ros2.org/articles/clock_and_time.html
13. rclpy Humble：`rclpy/time_source.py`（`use_sim_time` 声明与缺省）、`rclpy/node.py`（TimeSource 挂载）：https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/time_source.py 、https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/node.py
14. message_filters Humble：`src/message_filters/__init__.py`（`TimeSynchronizer`、`ApproximateTimeSynchronizer`）：https://github.com/ros2/message_filters/blob/humble/src/message_filters/__init__.py
15. tf2 / tf2_ros Humble：`tf2_ros/include/tf2_ros/buffer.hpp`（`time=0` 语义）、`tf2/include/tf2/buffer_core.hpp`（异常）、`tf2/src/cache.cpp`（`findClosest`/`interpolate`）：https://github.com/ros2/geometry2/blob/humble/tf2_ros/include/tf2_ros/buffer.hpp 、https://github.com/ros2/geometry2/blob/humble/tf2/include/tf2/buffer_core.hpp 、https://github.com/ros2/geometry2/blob/humble/tf2/src/cache.cpp
16. rosbag2 Humble：`rosbag2_transport/src/rosbag2_transport/recorder.cpp:387`、`rosbag2_storage/include/rosbag2_storage/serialized_bag_message.hpp`：https://github.com/ros2/rosbag2/blob/humble/rosbag2_transport/src/rosbag2_transport/recorder.cpp 、https://github.com/ros2/rosbag2/blob/humble/rosbag2_storage/include/rosbag2_storage/serialized_bag_message.hpp
17. GNU libstdc++ `bits/chrono.h`（`high_resolution_clock = system_clock`，gcc-12.3.0）：https://github.com/gcc-mirror/gcc/blob/releases/gcc-12.3.0/libstdc++-v3/include/bits/chrono.h
   本机实测（Ubuntu 22.04 / g++ 11.4.0）：`high_resolution_clock == system_clock` 为真、period 1/1e9、count ≈ Unix epoch 纳秒。

**本仓库（当前工作区）**

18. `src/robot_safecontrol_moveit/perception_bridge.py`（`295-326`、`331-368`、`389-438`、`490-503`、`546-552`）
19. `portable_oscbf/work/fusion_engine.py`（`156-240`、`269-286`、`309-342`）
20. `portable_oscbf/work/static_occupancy.py`（`52-101`）
21. `src/robot_safecontrol_moveit/oscbf_controller.py:503-518`
22. `src/robot_safecontrol_moveit/livox_mid360/protocol.py:454-463`、`stream.py:109`、`ros_node.py:22-28`
23. 配置：`config/perception_runtime.yaml:65-70`、`config/perception_dual_sensor_real.yaml:48-54`、`portable_oscbf/config/obstacle_params.yaml:31-38`
24. 规格与既有研究：`docs/specs/dual_sensor_perception_fusion_spec.md`、`docs/planning/oscbf-reuse/research/perception-time-model.md`、`docs/planning/oscbf-reuse/research/perception-age-budget.md`、`docs/planning/oscbf-reuse/handoffs/7-perception-time-model.md`、`docs/planning/oscbf-reuse/research/official-sensor-drivers.md`、`docs/planning/oscbf-reuse/handoffs/31-official-sensor-drivers.md`、`docs/planning/oscbf-reuse/research/calibration-ssot-runtime-contract-20260911.md`
25. GitHub issue #7（含 2026-09-05 / 09-07 / 09-09 / 09-11 评论）、#45（OFF-01）、#51（OFF-06）：`https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/7`（同仓库 `/45`、`/51`）

---

## 附：本文未做的事

- 未启动任何设备、未读取本机当前雷达/相机的实际时间戳、未改动任何已有文件（本文件为新增）。
- 未给出任何 `age_warn`/`age_stop`、PTP 精度、曝光时长、映射误差等数值。
- 未把「二手博客/社区帖子」当作结论依据；本文所有结论都指向上述一手来源，推断项已单独标注。
