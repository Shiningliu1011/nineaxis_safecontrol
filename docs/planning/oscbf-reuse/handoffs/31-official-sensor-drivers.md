# [P0] 双传感器官方驱动接入与版本冻结

- Tracker：https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/31
- 日期：2026-09-08；认领：Shiningliu1011。
- 状态：研究与本地只读核查完成；候选版本已明确，未实施、未达到部署冻结与关闭门槛。
- 已同步[阶段进度评论](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/31#issuecomment-5586798382)，不是 resolution；保持 OPEN，不追加地图已完成索引。

## 起点与范围

用户原始指令：“继续下一个ticket”。依照[标定工具链交接](25-calibration-toolchain.md)与[推荐窗口顺序](../DECISION-WINDOW-ORDER.md)，本窗口处理本票。已先认领；原生依赖仍为开放的[规格与术语修订](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/20)和[标定 SSOT 接线](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/27)。仅推进不依赖缺失前置的研究，不撤销依赖。

起点 main，commit `659da6c6db598abddc272ad0941b72f77793211b`。保留已有 ADR、MAP、感知规格修改及未跟踪 handoffs、研究、.scratch 和大然电机资料。本轮不安装依赖、不启动传感器、不修改运行代码或配置；研究分支与产品分支分离：本地 `codex/research/official-sensor-drivers`，commit `05dbfa433dfd34ce59a38f990620bafd14f22819`，worktree `.scratch/oscbf-reuse-wayfinder/31/research-worktree`；未推送，研究文件已复制回主工作区供审阅。

## 本机核验

原始命令、返回码、输出及相关源码/配置 SHA256 见[本地审计 JSON](../../../../.scratch/oscbf-reuse-wayfinder/31/local-audit.json)。

- Ubuntu 22.04.5 LTS，x86_64，内核 `6.8.0-138-generic`；ROS Humble 已安装。系统 libusb 开发包为 `2:1.0.25-1ubuntu2`。
- 仅加载 `/opt/ros/humble/setup.bash` 后，`ros2 pkg list` 未匹配 livox/orbbec；这只描述所检查的环境，不排除其他未加载 overlay 或目录。
- 本次 `lsusb` 没有可识别的 Orbbec 设备；不能据此推断以太网雷达不存在。没有扫描网络、查询固件或触发数据采集。
- 用户已确认型号为 MID-360S / Gemini 335L，原话：“mid-360s和gemini335l，现在还没法连接，明天可以连接两个设备”。连接窗口预期为明天；未安排自动任务。序列号、固件及在线设备身份仍待读取，型号确认不等于本机兼容实测通过。
- `package.xml` 没有两个官方驱动依赖；当前 launch 没有启动它们。配置注释中出现的驱动命令不是本机可复现证据。
- 相机配置预期 `/camera/depth_registered/points`、`camera_color_optical_frame`；LiDAR 在 runtime 与 real profile 中仍禁用。不要直接打开 topic 即宣称接入完成。
- bridge 两路订阅均为 `sensor_msgs/PointCloud2` + `qos_profile_sensor_data`，解码仅保留 xyz；逐点时间、tag、line 等不会保留到其当前缓存。
- `_sensor_callback` 将 header stamp 转为秒，使用配置 input_frame 做变换，未检查实际 header.frame_id。时间域、错误帧拒绝和空帧健康语义须由既有时间、SSOT、观测/健康票落实，不能用驱动成功启动代替。

## 复用与接口研究

见[官方传感器驱动研究](../research/official-sensor-drivers.md)。候选固定版本、许可证、构建/启动说明与字段证据由该研究持有；在本机复现之前只是候选冻结，不是部署冻结。

关键接入差异：官方 MID-360S launch 默认 `xfer_format=1`（CustomMsg），与 bridge 的 PointCloud2 订阅不匹配；该脚本使用 Python 常量，不能假定追加 `xfer_format:=0` 就会覆盖。实施应在本项目提供小型 launch 适配，直接给官方节点传入 `xfer_format=0` 等已核验参数，保留上游源码不改；启动后再核对实际 topic 类型。`livox_frame` 是默认值而非不可变常量。相机配置的彩色注册点云 topic/frame 在候选源码中吻合，但实际设备流配置仍需连接后确认。相机内部帧同步和设备到主机的时间映射不构成相机与雷达硬同步。

## 编码与验收门

可准备隔离的依赖清单、构建环境和只读消息审计工具。正式双源接入仍需前置规格/SSOT 契约落地，并按实物身份选定驱动配置；本轮不自动实施产品代码。

实施时只做官方驱动依赖、launch/参数与消息薄适配；现有 FusionEngine 与几何/控制算法保留，未发现必须 fork 上游的证据。外参只应用一次：驱动输出维持其明确的传感器局部帧，传感器至 base_link 由 SSOT 负责。相机彩色/深度内部标定不等于本项目 sensor→base 标定。

每路独立验收并留存：

1. 精确硬件型号、序列号、固件、USB链路/网卡配置；源码 commit、SDK版本、许可证与依赖锁、构建命令及日志、实际 ament 解析路径。
2. `ros2 topic info --verbose` 的类型/QoS/发布者；样本的 header、fields、point_step、row_step、endianness、实际 frame 与米单位对照；避免仅凭话题名判断格式。
3. 点云时间戳的时钟域、基准时刻和扫描/曝光语义，记录设备时间与主机接收时间，不能将回调时间或近似配对当成硬同步。漂移、跳变、断流与重启重同步交给感知时间票定量核验。
4. 错帧、错消息类型、空帧、掉线、时间回退、错误序列号与重连的可观测行为；缺源不能等价为空且安全。
5. 两路独立通过后才做同机并发采集，记录丢帧、USB/网络负载、CPU/内存和延迟；之后再接统一观测链。

没有执行本机驱动构建、设备采集、bag 回放或并发性能测量，不能关闭本票或宣称双源准入。文档研究不需要运行产品测试套件。

## 回退与下一窗口

未来驱动在独立 workspace 部署，保留旧依赖/参数/标定与模型快照；停止相关节点后切回已记录版本并重新核验实际解析路径和身份，旧假定标定不会因回退获得准入。

下一研究窗口为[感知时间同步与延迟模型](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/7)；本票未完成仍是其整票原生阻塞，可先研究测量方法，不得声称已获得设备时钟证据。本窗口不自动开展下一票。


## 2026-09-09 在线验证（进行中）

以下实测更新此前“未连接/未构建”的历史状态；尚未达到双源准入。
证据目录：`.scratch/oscbf-reuse-wayfinder/31/online-2026-09-09/`。

- 实物 SDK 身份：MID-360S（type 35），IP `192.168.1.115`，固件版本字节 `[35,1,1,8]`；Gemini 335L，固件 `1.4.60`，USB 3.2。序列号见原始查询日志。
- 被动 ARP 确认雷达向主机 `192.168.1.5` 请求连接；使用独立网卡临时 NetworkManager 配置，不修改 Wi-Fi 默认路由。
- Livox SDK 1.3.1 与 ROS driver2 1.2.6 在 scratch 独立前缀构建成功。启动需 source ROS/driver overlay，并将 `livox_sdk_install/lib` 加入 `LD_LIBRARY_PATH`；首次遗漏该路径导致动态库加载失败，补充后正常。
- 直接启动官方节点，`xfer_format=0`，配置使用实际 IP。官方初始化会设置点云类型、扫描模式、零安装外参、Normal 工作模式并启用 IMU；这一步不是只读查询。启动前读到点云类型 1、模式 0、安装外参全零。
- `/livox/lidar` 实测 PointCloud2，`livox_frame`，point_step 26；字段 x/y/z/intensity float32、tag/line uint8、timestamp float64。12 秒采样 121 帧，约 10 Hz，每帧 19872–20064 点，采样窗口内 header 时间无回退；不能据此声称硬件同步或长期稳定。见 `lidar-ros-sample.json`、`lidar-ros-summary.json`、`lidar-ros-run.log`。
- 相机 SDK 深度流此前 10 秒采到 298 帧，frame index 无缺口；这不替代 ROS 注册点云与双源并发验收。
- 用户已安装 camera_info_manager、image_publisher、diagnostic_updater、camera_calibration_parsers、gflags、nlohmann-json、glog 开发依赖；相机 ROS 驱动构建继续进行。相机重连后 USB 编号改变，临时 ACL 必须对应当前设备。

### 本轮相机与并发结果

- Orbbec ROS2 v2.9.3 三个包全部构建成功（约 75 秒，`CMAKE_BUILD_PARALLEL_LEVEL=2`）。用户安装依赖后，缺包问题消失。
- 官方 `gemini_330_series.launch.py` 参数：`depth_registration:=true enable_colored_point_cloud:=true color_width:=640 color_height:=480 color_fps:=30 depth_width:=640 depth_height:=480 depth_fps:=30 time_domain:=global`。实际注册点云 topic 为 `/camera/depth_registered/points`，frame 为 `camera_color_optical_frame`，字段 xyz/rgb float32，point_step 20，小端。12 秒窗口收到 313 帧，约 26.26 Hz，无 header 时间回退。
- 双设备同时发布的 12 秒探针：相机 244 帧，约 20.63 Hz；雷达 121 帧，约 10.00 Hz；各自 header 无回退。相机点云发布未达到请求的 30 Hz，不能宣称双源性能验收通过；此数据是订阅端观测频率，尚未区分驱动处理、同步等待和消息丢失。
- 原始证据：`camera-ros-build.log`、`camera-ros-run.log`、`camera-ros-sample.json`、`camera-concurrent-sample.json`、`lidar-concurrent-sample.json` 与 `probe_ros.py`。雷达 topic info 查询发生在限时节点退出后，返回 Unknown topic；不拿这次查询充当 QoS 证据。
- 下一步：量化相机注册点云吞吐/延迟及并发负载，补充 QoS、点云单位实测、时间域和重连/断流测试；正式 observation 接线仍等待规格与标定 SSOT 前置。保持 OPEN。

### 重连持久配置

用户要求消除每次重连手动配置。已保存 NetworkManager `robot-mid360s`，绑定网卡 MAC `6c:1f:xx:xx:xx:xx`，自动连接，主机 `192.168.1.5/24`，never-default，IPv6 disabled；不再清理此持久配置。用户已安装 `/etc/udev/rules.d/99-robot-gemini335l.rules`，针对 USB `2bc5:0804` 将设备节点 owner 设置为 lsn、权限 0660，不依赖 bus/device 编号。核验当前节点 owner=lsn，普通用户 SDK 成功打开 Gemini 335L（固件 1.4.60 / USB3.2）。规则重载并触发已通过，安装后的物理拔插与整机重启尚未实测。上述配置解决主机网络与 USB 权限持久性，不负责自动启动 ROS 节点或驱动运行中热插拔恢复。

### 用户提供的 Python 驱动评估（补充）

用户提供 `~/robot/tmbs-main.zip`，要求评估其 MID-360 驱动对本项目的可复用性并做适配。已落地为项目内纯 Python 模块 `src/robot_safecontrol_moveit/livox_mid360/`（协议层/设备控制/发现/连续流接收/ROS 节点/CLI，57 个新测试全过），补的是官方驱动不提供的设备控制面与免 SDK 备用通道；**官方 C++ 驱动仍按本票结论保持主路径**。字段布局对齐官方 `lddc.cpp`（point_step 26），`perception_bridge` 无需改动。**2026-09-10 已对该真机完成验证，见下节**；同日用户确认来源可用于本项目，模块与测试已入库。详见[复用与适配记录](../research/tmbs-mid360-python-driver-reuse.md)。

## 2026-09-10 纯 Python 模块真机验证

对 MID-360S（SN `ARMCP7D****`、`192.168.1.115`）完成发现、连续流与 `perception_bridge` 端到端验证。证据目录 `.scratch/livox-hw-test/`（脚本、日志、逐帧统计 JSONL）；详细表格见[复用与适配记录](../research/tmbs-mid360-python-driver-reuse.md)。

- 广播发现修复：设备对发现广播的应答是广播目的地址，只有 `INADDR_ANY` 绑定能收到（绑定 `host_ip` 收不到）。改为 any-bind 后 `discover_subnet` 5/5、CLI `discover` 2/2 返回 SN。
- **真机暴露并修复一个丢帧缺陷**：Fast-DDS（`rmw_fastrtps_cpp`）共享内存单样本上限 512 KiB（524,288 B）；本模块按 0.1 s 切帧加 96 点包粒度会产出 20,064–20,544 点（521,664–534,144 B）的帧，越过上限后订阅端只剩 4.6–7.25 Hz。尺寸扫描确认悬崖：20,100 点（522,600 B）→ 10 Hz 稳定，20,160 点（524,160 B）→ 3.57 Hz。修复为 `MAX_FRAME_POINTS=20,000` 的硬切帧规则，回归测试锁定“载荷 + 4 KiB < 512 KiB”。
- 修复后：C++ 订阅端 9.969 Hz（std dev 0.5 ms）、Python 订阅端 10.00 Hz；14 s 采集 0 丢包、200,318 点/s；与官方驱动逐帧内容一致（零返回 34.96% vs 35.48%，最大距离 18.10 vs 18.46 m，point_step 26）。
- bridge 端到端（本轮 LiDAR-only，相机未运行）：`lidar_alive=1` 持续、`/perception/status` 19.99 Hz、`/perception/tracks` 10.00 Hz、`/perception/cloud_world` 3.64 Hz、`/perception/esdf` 2.14 Hz、`/collision_object` 29.90 Hz；`/perception/status` 采样 `lidar_used=1, source_count=1, perception_valid=1`。`lidar_used` 每两拍为 0 是 FusionEngine 重复帧保护的既定行为，不是掉流。
- 回归：主包 `tests/` 307 通过；`portable_oscbf/tests` 146 通过 / 34 跳过 / 1 既有 JAX 容差失败（与本改动无关）。
- 与官方驱动的能力对比已写入[复用与适配记录](../research/tmbs-mid360-python-driver-reuse.md#与官方驱动的能力对比)：官方独有 `/livox/imu`、CustomMsg（`xfer_format=1`）、多雷达；本模块独有设备控制面、免构建部署、只读启动；官方切帧只有 `publish_freq` 没有点数上限，`publish_freq=5` 时单帧 ~40,000 点会越过 512 KiB 上限（源码推断，未实测）。
- 仍未做：PTP/GPS 同步实测（设备未同步，两驱动都退回主机时间）、长时间稳定性与断线重连、写命令（configure/mode/reboot）真机演练。设备只向最近一次配置的主机:端口推流，两个驱动同时运行会抢同一 UDP 端口并重复发布同一话题，接入时二选一或改用不同话题名。本票仍保持 OPEN，双源准入未因此节完成。
