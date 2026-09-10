# 感知时间同步与延迟模型：官方语义核验

研究日期：2026-09-09。服务于“感知时间同步与延迟模型”决策票。只核验一手资料与固定源码，未运行设备。本地链审计和决策由独立交接记录；以下建议不是已完成的校准。

## 固定研究版本

| 组件 | 版本及提交 |
|---|---|
| Livox driver2 | `1.2.6`，`13eb05e4e6dd7a765b934d0c5fd6236676a57b49` |
| Livox SDK2 | `v1.3.1`，`f5d9375f84efe2b15bc0a052d3e18482ed13adf4` |
| Orbbec ROS2 wrapper | `v2.9.3`，`ce08bce25f7a0a6fe939ece87ec945447109581f`；SDK语义取自该提交自带头文件，不由wrapper版本推定现场库版本 |
| ROS2 message_filters | humble研究快照 `14ffd199d23cdfcd7ed102de8328081a5ec8f5e6`，不是部署锁定 |

Livox两个发布分别加入Mid-360S支持：[driver发布](https://github.com/Livox-SDK/livox_ros_driver2/releases/tag/1.2.6)、[SDK发布](https://github.com/Livox-SDK/Livox-SDK2/releases/tag/v1.3.1)。支持机型不证明时钟已同步。

## Livox：header不总是设备采样时间

同步类型PTP/gPTP或GPS时，驱动采用包内时间；未同步时，`GetEthPacketTimestamp` 丢弃设备timestamp，改用 `high_resolution_clock::now().time_since_epoch().count()`。它接近驱动处理包的主机时刻，不能作为已校准的激光采样时刻。**C++标准不保证high_resolution_clock就是system_clock，也不保证count单位为ns或epoch为Unix**；源码未显式duration_cast。现场必须核验构建平台，不能把本机典型实现提升为通用保证。[pub_handler.cpp 265–275](https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/src/comm/pub_handler.cpp#L265)

SDK包包含time_type、8字节timestamp、time_interval（0.1微秒）和点数。驱动按 `time_interval * 100 / dot_num` 构造ns间隔，再以包时间加点索引间隔重建点时间。这是重建规则，不是每点有独立硬件打点的证据。[SDK包定义](https://github.com/Livox-SDK/Livox-SDK2/blob/f5d9375f84efe2b15bc0a052d3e18482ed13adf4/include/livox_lidar_def.h#L129)、[间隔换算](https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/src/comm/pub_handler.cpp#L140)、[逐点赋值](https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/src/comm/pub_handler.cpp#L388)

一次发布聚合多点；`GetLidarBaseTime` 取集合首点时间。不能把整云当瞬时采样，也不能把发布周期直接认作精确扫描跨度。应记录实际点时间范围与丢包。[批处理](https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/src/comm/pub_handler.cpp#L164)、[首点基准](https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/src/comm/pub_handler.cpp#L280)

**PointCloud2 timestamp为FLOAT64绝对ns值**，由内部offset_time转double；名称offset_time不能被误解为相对header的秒。header把pkg.base_time按ns交给rclcpp。保留整数来源时用int64 ns，FLOAT64解析应承认量化误差，绝不经过float32 epoch。[字段与填充292–328](https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/src/lddc.cpp#L292)

## Gemini 335L：global是主机域映射

自带API定义：system是主机收到帧时刻；global是设备采集时刻转换到主机时钟域并补偿设备漂移。未启用global返回0，且只有部分设备支持。此API没有给出可直接采用的最大映射误差，也没有界定曝光开始/中心/结束。[Frame.hpp 195–219](https://github.com/orbbec/OrbbecSDK_ROS2/blob/ce08bce25f7a0a6fe939ece87ec945447109581f/orbbec_camera/SDK/include/libobsensor/hpp/Frame.hpp#L195)

wrapper在global且非回放时开启global timestamp，并调用getGlobalTimeStampUs。主机时间还可选择realtime/monotonic，所以global名称不足以证明能与ROS epoch直接相减；必须记录SDK时钟设置。[开启](https://github.com/orbbec/OrbbecSDK_ROS2/blob/ce08bce25f7a0a6fe939ece87ec945447109581f/orbbec_camera/src/ob_camera_node.cpp#L4692)、[取值](https://github.com/orbbec/OrbbecSDK_ROS2/blob/ce08bce25f7a0a6fe939ece87ec945447109581f/orbbec_camera/src/ob_camera_node.cpp#L6059)、[时钟选择API](https://github.com/orbbec/OrbbecSDK_ROS2/blob/ce08bce25f7a0a6fe939ece87ec945447109581f/orbbec_camera/SDK/include/libobsensor/hpp/Context.hpp#L88)

**深度点云及注册彩色点云header均取depth_frame时间。** 注册后frame_id可以是COLOR光学坐标系，这不意味着color与depth同时曝光。若使用颜色必须另保存配对的color时间及残差；仅用几何也要明确depth为年龄来源。[深度点云](https://github.com/orbbec/OrbbecSDK_ROS2/blob/ce08bce25f7a0a6fe939ece87ec945447109581f/orbbec_camera/src/ob_camera_node.cpp#L5657)、[彩色点云](https://github.com/orbbec/OrbbecSDK_ROS2/blob/ce08bce25f7a0a6fe939ece87ec945447109581f/orbbec_camera/src/ob_camera_node.cpp#L5789)

滚动官方参数文档建议global下关闭enable_sync_host_time；该参数是另一机制。enable_frame_sync同样不能证明与Livox共享触发。[官方参数](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/4_application_guide/launch_parameters.html)。具体行为优先以上述固定源码为准。

## ROS Humble的边界

ROS Time未启用仿真源时跟随System Time；use_sim_time可切至/clock并允许暂停、前后跳变。硬件超时宜隔离使用steady clock；不同域不能直接相减。[ROS2时间设计](https://design.ros2.org/articles/clock_and_time.html)

Humble ApproximateTimeSynchronizer只按header排队并比较stamp差与slop，不校准设备时钟，不还原采样时刻。近似/精确配对、相同frame_id、相近stamp均不足以证明物理同步。[官方Humble实现](https://github.com/ros2/message_filters/blob/14ffd199d23cdfcd7ed102de8328081a5ec8f5e6/src/message_filters/__init__.py#L337)。ROS文档站本次访问被拒，改读官方仓库源码。

## 测量与契约建议（工程推论）

1. 每源保存source_id、sequence、raw_stamp、clock_domain、time_kind、映射后sample_start/end、clock_error_bound、recv_steady、处理结束steady、实际选中sequence。time_kind区分采样、估计采样、主机收到。映射未知不能判已证明fresh。
2. 控制消费时对每个**实际选中的源**分别计算最老点年龄 `A_i=t_consume_common-mapped_sample_start_i`，保守界加时钟映射/采样定义误差。LiDAR保存[start,end]；只有header须另有扫描跨度界。融合输出stamp不能用max掩盖旧源，缺失源不能借别源刷新TTL。
3. 区分收到至消费的steady处理时延、header至收到的表观差、采样至消费的校准年龄。表观差统计无法单独分离传输延迟和时钟偏差，p99或样本max不是安全硬上界。
4. 记录Livox time_type/PTP状态及offset日志；相机device/global/system三时间、SDK版本/时钟选项；主机realtime与steady对照、use_sim_time及CPU/USB/网络负载。覆盖暖机、校时、重连、丢锁；映射变化须清旧配对并重新验证。
5. 用共同可观测运动事件或可用硬件参考核验跨源采样偏差，计入曝光/扫描跨度、事件定位、触发路径及仪器不确定性。包到达差不能辨认单向传输时延与时钟偏差。不预设两设备可直接共触发，现场固件与接口仍需核验。
6. 分源统计重复/倒退序号、零/未来/倒退stamp、扫描跨度、最大间隔、配对残差、队列等待和消费年龄；验证断线、突发负载及时间跳变。未来超出允许映射误差应不可用，不能将负年龄裁0而判鲜。
7. 本票可定字段、域分类、年龄公式及校准门槛；数值TTL、最大clock error与障碍运动裕度需后续实测和控制预算。证据不足保留未知/不可用，不以当前样本最大值直接批准运行阈值。

## 未获得的证据

未核实现场Livox time_type/PTP锁定；未建立相机global误差上界或曝光参考点；未验证双设备共同参考；未进行长期尾延迟/故障实验。本研究不宣称硬件同步或时延验收通过。
