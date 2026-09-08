# 双传感器官方驱动：源码候选与连接前边界

日期：2026-09-08。对应 [P0 双传感器官方驱动接入与版本冻结](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/31)。本研究仅查阅第一方页面与固定版本源码，未安装依赖、构建驱动、连接硬件或刷新固件。用户已确认型号为 **Mid-360S 与 Gemini 335L**；序列号、固件、实际消息和运行质量仍待连接验证。以下是可复现的**候选锁定**，不能标作部署冻结。

## 候选版本

|组件|官方 tag → 完整 commit|适配依据与边界|
|---|---|---|
|Livox ROS driver2|`1.2.6` → `13eb05e4e6dd7a765b934d0c5fd6236676a57b49`|[CHANGELOG](https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/CHANGELOG.md) 明示1.2.5新增Mid-360s、1.0.0支持Ubuntu22.04/Humble；不是把MID-360当成S型号。|
|Livox SDK2|`v1.3.1` → `f5d9375f84efe2b15bc0a052d3e18482ed13adf4`|[CHANGELOG](https://github.com/Livox-SDK/Livox-SDK2/blob/f5d9375f84efe2b15bc0a052d3e18482ed13adf4/CHANGELOG.md) 的1.3.0新增Mid-360s，1.3.1新增该型号ESC/PPS配置；不能沿用没有S支持证据的1.2.x。未找到driver1.2.6与SDK1.3.1严格配对矩阵，需构建与运行验证。|
|OrbbecSDK_ROS2|`v2.9.3` → `ce08bce25f7a0a6fe939ece87ec945447109581f`|[固定README](https://github.com/orbbec/OrbbecSDK_ROS2/blob/ce08bce25f7a0a6fe939ece87ec945447109581f/README.MD) 列Ubuntu22.04、Humble、Gemini335L与`gemini_330_series.launch.py`；v2系列推荐用于新设计。|
|Orbbec随驱动SDK|同一driver commit内的SDK **2.9.3** 二进制|[SDK/lib](https://github.com/orbbec/OrbbecSDK_ROS2/tree/ce08bce25f7a0a6fe939ece87ec945447109581f/orbbec_camera/SDK/lib) 含x64/arm64 `libOrbbecSDK.so.2.9.3`；[CMake](https://github.com/orbbec/OrbbecSDK_ROS2/blob/ce08bce25f7a0a6fe939ece87ec945447109581f/orbbec_camera/CMakeLists.txt)直接链接包内SDK，不应另外安装不明版本覆盖它。|

tag/commit通过官方GitHub tags API并fetch该tag验证；固定commit优先于浮动分支或未固定apt版本。连接后还应记录所加载动态库路径、文件SHA256和SDK运行时报告。

## Livox接入契约与最小薄适配

[官方S型号launch](https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/launch_ROS2/msg_MID360s_launch.py) 确实叫 `msg_MID360s_launch.py`（小写s）。它默认 `xfer_format=1`，因此输出CustomMsg，**与现有“该launch输出PointCloud2”的假设冲突**。该文件用Python常量，未声明launch参数，不能把 `xfer_format:=0` 当成有效覆盖办法。[rviz_MID360s_launch.py](https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/launch_ROS2/rviz_MID360s_launch.py) 默认0但会启动RViz。

建议项目薄launch直接创建官方 `livox_ros_driver2_node`，显式设置 `xfer_format: 0, multi_topic: 0, data_src: 0, publish_freq: 10.0, output_data_type: 0, frame_id: livox_frame, user_config_path: <项目绝对配置路径>`；其他参数沿固定官方样例。这样仅适配参数，不改驱动。`frame_id`是可配置值，并非厂商固定不可变名称。项目配置应从[S专属配置](https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/config/MID360s_config.json)生成：`lidar_type=8`、`Mid360s`节、主机网络项为数组。示例主机192.168.1.5、设备192.168.1.12只是样例，必须按现场地址修改；设备UDP端口56100–56500、主机56101–56501逐项配置。外参保持恒等，安装外参由项目标定SSOT提供（TF仅可由同一记录派生），避免重复变换。

[消息构建源码lddc.cpp](https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/src/lddc.cpp)显示：无namespace且multi_topic=0时`/livox/lidar`与`/livox/imu`。PointCloud2字段为x/y/z/intensity FLOAT32、tag/line UINT8、timestamp FLOAT64，offset依次0/4/8/12/16/17/18。不要只依据旧launch注释把它当成不含时间的PointXYZRTL。header.stamp来自`pkg.base_time`纳秒；timestamp直接装入内部`offset_time`。其[生产代码pub_handler.cpp](https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/src/comm/pub_handler.cpp)实际为包时间加点间隔（纳秒），**不可见到offset_time名称就当成相对header秒数**。XYZ由毫米转换到米。CustomMsg另有timebase和相对offset，不应混用两种解码规则。

[Mid-360S规格](https://www.livoxtech.com/mid-360s/specs)明确列PTPv2与GPS能力；能力存在不证明现场已同步。未从上述固定源码材料确定必须固件版本，应先读设备固件及官方对应发布记录再决定，禁止填写推测版本或自动刷写。

## Orbbec接入契约

[固定launch](https://github.com/orbbec/OrbbecSDK_ROS2/blob/ce08bce25f7a0a6fe939ece87ec945447109581f/orbbec_camera/launch/gemini_330_series.launch.py)真正声明了参数，以下是待执行候选：

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py camera_name:=camera depth_registration:=true enable_colored_point_cloud:=true enable_point_cloud:=true time_domain:=global
```

[固定消息源码](https://github.com/orbbec/OrbbecSDK_ROS2/blob/ce08bce25f7a0a6fe939ece87ec945447109581f/orbbec_camera/src/ob_camera_node.cpp)建立`depth_registered/points`彩色PointCloud2：camera namespace下对应`/camera/depth_registered/points`，XYZ米并带RGB，默认color optical frame（`camera_color_optical_frame`）；可由cloud_frame_id覆盖，所以应检查实际header。普通深度点云为`depth/points`，不能仅凭名称把它与彩色注册云互换。颜色配准是相机内部D2C，不是相机到雷达外参。

launch默认depth_registration=false、enable_colored_point_cloud=false；要生成项目选定彩色云须显式打开。time_domain默认global，源码选`getGlobalTimeStampUs()`再转ROS时间；device/system分别选另两个SDK时间接口。enable_frame_sync默认true控制相机内部帧同步，sync_mode默认standalone，enable_sync_host_time默认false；这些不能证明与Livox同一时钟域。`enable_ptp_config`注明仅用于Gemini335Le，**不能套用于335L**。明日必须测stamp与主机时间差、双流时间差、抖动/回退并记录实际时域。

固定README表推荐Gemini335L固件 **1.6.00**，这是推荐值而非已证明的最低兼容版本或设备现值。上游release页可能随时间提供不同推荐，不能用浮动页面替代固定版本配置。设备连接后保留现值并核对官方发布说明，再决定是否另开升级工作。

## 构建复现路线（未执行）

新建独立ROS工作空间，按上表tag clone后核验`git rev-parse HEAD`与完整SHA一致。Livox依赖SDK2：[官方SDK README](https://github.com/Livox-SDK/Livox-SDK2/blob/f5d9375f84efe2b15bc0a052d3e18482ed13adf4/README.md)给出CMake≥3.0/gcc≥4.8.1、Linux Ubuntu18.04以上，`cmake ..`、`make -j`、`sudo make install`安装至/usr/local。随后把driver放在`<workspace>/src/livox_ros_driver2`，source `/opt/ros/humble/setup.sh`，在driver目录执行`./build.sh humble`，source工作空间install/setup.sh。详见[固定driver README](https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/README.md)。本研究不执行sudo或安装。

Orbbec依赖以固定README的Installation依赖列表为准（gflags、nlohmann-json、image_transport及plugins、camera_info_manager、diagnostic、xacro、backward_ros、libdw/ssl/glog等）。固定源码放独立workspace/src，source Humble后执行：

```bash
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release -DINSTALL_UDEV_RULES=OFF
source install/setup.bash
```

`INSTALL_UDEV_RULES=OFF`由固定CMake支持，可避免普通构建尝试写/etc；之后单独按README安装官方udev规则，再重新插拔/验证设备访问。规则安装是现场操作步骤，不代表此处已配置。避免source其他driver workspace使同名包覆盖；记录`ros2 pkg prefix`及运行时动态库路径。当前没有编译成功证据，所有命令还需要现场验证。

## License与验收边界

Livox [driver LICENSE](https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/LICENSE.txt)与[SDK LICENSE](https://github.com/Livox-SDK/Livox-SDK2/blob/f5d9375f84efe2b15bc0a052d3e18482ed13adf4/LICENSE.txt)列自有部分MIT及第三方通知（driver还列ROS BSD、rclcpp Apache等）；复制/分发时保留整份通知。Orbbec wrapper [LICENSE](https://github.com/orbbec/OrbbecSDK_ROS2/blob/ce08bce25f7a0a6fe939ece87ec945447109581f/LICENSE)为Apache-2.0，随包SDK与扩展的[licenses目录](https://github.com/orbbec/OrbbecSDK_ROS2/tree/ce08bce25f7a0a6fe939ece87ec945447109581f/orbbec_camera/SDK/licenses)有独立第三方条款，不能将所有二进制笼统称Apache。

成为部署冻结前仍缺：两设备序列号/固件/连接拓扑，目标机驱动构建结果，消息类型与字段/单位/header样本，连续录包的频率/丢帧/时钟误差证据，以及实际SDK库哈希。官方支持型号、固定源码和launch能核对，只证明候选有依据；这些未测项应留在原票的现场验收条件中。
