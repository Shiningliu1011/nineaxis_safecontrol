# 传感器静态外参的规范真源：TF 还是参数？

日期：2026-09-14。范围：为 [OFF-01](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/45) 的「是否保留 TF 查找分支、是否保留旧 ROS 参数探测」提供一手依据。本研究不实施、不改代码、不动配置；只读检索 + 隔离的本地离线复现（无 ROS 图、无硬件）。

**证据分级**（全文每条结论都带标记）：

| 标记 | 含义 |
| --- | --- |
| 【一手文档明确】 | 官方文档 / 规范 / 官方 README 的明确表述 |
| 【一手源码可见】 | 官方源码或官方头文件注释可见；已给 file:line |
| 【本机实测】 | 在**本机安装的 Humble 版本**上离线复现（属"一手源码可见"的实测加强项，最有力） |
| 【推断（需实测确认）】 | 由源码推导、但本研究未运行验证 |
| 【未知】 | 没有一手依据。不猜造数值、默认值或固件能力 |

本机被核对的版本：ROS 2 Humble，`tf2_ros 0.25.22`、`rclcpp 16.0.19`、`rclpy 3.3.21`（`/opt/ros/humble/share/*/package.xml`）。上游在线源码读取于 2026-09-14；仓库内 `.scratch/oscbf-reuse-wayfinder/31/online-2026-09-09/upstream/` 另有一份固定提交的副本（OrbbecSDK_ROS2 `ce08bce25f7a0a6fe939ece87ec945447109581f`、livox_ros_driver2 `13eb05e4e6dd7a765b934d0c5fd6236676a57b49`），本研究的 Orbbec/Livox 结论已用两份来源交叉核对。

---

## 1. 问题与范围

一句话：**决定 OFF-01 是否保留 `perception_bridge` 的 TF 查找分支（`use_tf`）与旧 ROS 参数（`camera_to_world_static` / `lidar_to_world_static`）探测**，依据是 ROS 2 规范/官方源码对「sensor→base_link 静态外参的真源」的表述。

三个子问题：(a) 规范层推荐把静态外参放在哪；(b) tf2 静态变换的确切语义（能否替代标定记录、能否与记录并存）；(c) 未声明参数 override 的确切语义（旧参数路径今天是"静默失效"还是"仍然生效"）。

本文件是 `docs/adr/0009-calibration-record-schema-identity-admission.md` 中「仍待定」第 2 项（*"旧 ROS 参数（`use_tf` / `camera_to_world_static` / `lidar_to_world_static`）与 `perception_bridge.py:303-322` 的 TF 查找分支的最终处置：纯删除 / 删除＋旧参数探测器 / 保留 TF 可选路径"*）的调研输入；结论面向 OFF-01 的取舍，不代替该票的实施与验收。

---

## 2. 一手来源结论

### 2.1 规范层：静态外参的官方载体

**A1. REP-105 采用"树 + 单一权威"模型。【一手文档明确】**
[REP-105](https://www.ros.org/reps/rep-0105.html)（*Coordinate Frames for Mobile Platforms*, Status: Active）在 "Relationship between Frames" 一节：*"We have chosen a tree representation to attach all coordinate frames in a robot system to each other. Therefore each coordinate frame has one parent coordinate frame, and any number of child coordinate frames."* 并在 "Frame Authorities" 一节为每段变换指定唯一产生者（例如 *"The transform from odom to base_link is computed and broadcast by one of the odometry sources."*、*"The transform from earth to map is statically published and configured by the choice of map frame."*）。
→ 含义：`sensor→base_link` 这一段应当**只有一个发布权威**；两个发布者同时写同一条边违背该模型的意图（技术上 tf2 不会拒绝，见 B6）。

**A2. REP-105 并未规定传感器外参放在 URDF / 参数 / 配置文件的哪个。（文档边界）**
该 REP 的 frame 词汇表只有 `base_link` / `odom` / `map` / `earth`，通篇不涉及传感器安装外参的载体。【一手文档明确（"不涉及"这一事实本身）】
→ 含义：**不能用 REP-105 论证"外参必须写进 URDF"**。规范层能给的只有 A1 的单一权威原则。

**A3. REP-103 规定了 `_optical` 后缀的 frame 语义。【一手文档明确】**
[REP-103](https://www.ros.org/reps/rep-0103.html) 的 "Suffix Frames"：*"In the case of cameras, there is often a second frame defined with a `_optical` suffix. This uses a slightly different convention: z forward, x right, y down"*。
→ 含义：本项目 `input_frame: camera_color_optical_frame` 属于该约定；相机 frame 与其 optical frame 之间的固定旋转（Orbbec 用 `setRPY(-π/2, 0, -π/2)`）是**驱动内部约定**，不是安装外参。

**A4. URDF 规范：fixed joint 的 `<origin>` 就是"父 link → 子 link 的变换"。【一手文档明确】**
URDF XML 规范 [urdf/XML/joint](http://wiki.ros.org/urdf/XML/joint)：`<origin>` *"(optional: defaults to identity if not specified) This is the transform from the parent link to the child link."*；joint `type` 中 *"fixed — this is not really a joint because it cannot move. All degrees of freedom are locked."*
→ 含义：把 `base_link → camera_link` 写成一个 `fixed` joint 的 `<origin xyz rpy>`，是 URDF 规范层面表达安装外参的方式。注意：该规范页现为 ROS Wiki（已归档，页面顶部提示迁到 ROS 2 文档），但它是 `urdfdom` 解析器所实现的同一 XML 规范。

**A5. robot_state_publisher 官方 README：fixed joint → `/tf_static`，启动时发一次。【一手文档明确】**
Humble 版官方 README（[docs.ros.org/en/humble/p/robot_state_publisher](https://docs.ros.org/en/humble/p/robot_state_publisher/)）原文：*"Fixed joints (with the type "fixed") are published to the transient_local /tf_static topic once on startup (transient_local topics keep a history of what they published, so a later subscription can always get the latest state of the world). Movable joints are published to the regular /tf topic any time the appropriate joint is updated in the joint_states message."* 其 Published Topics 列出 `tf_static (tf2_msgs/msg/TFMessage) - The transforms corresponding to the static joints of the robot.`；参数只有 `robot_description` / `publish_frequency`(默认 20.0) / `ignore_timestamp`(默认 false) / `frame_prefix`(默认空)，**没有任何"外参参数"**。
→ 含义：URDF 里声明的固定安装关系，由 RSP 自动变成 `/tf_static`；不需要手写广播节点，也不需要把数值再放进节点参数。

**A6. RSP 源码印证「启动一次 + URDF 变化时重发」。【一手源码可见】**
`robot_state_publisher` humble 分支：`src/robot_state_publisher.cpp:129-130` 同时创建 `TransformBroadcaster` 与 `StaticTransformBroadcaster`；`:150` 构造末尾调用 `publishFixedTransforms()`；`:277-296` 中 fixed 段走 `static_tf_broadcaster_->sendTransform(...)`，movable 段（`:273`）走 `tf_broadcaster_`；`:402-423` 仅在 `robot_description` 参数变化时再次 `publishFixedTransforms()`。
（附带发现：`include/robot_state_publisher/robot_state_publisher.hpp:112` 的注释写作 `/tf2_static`，实际 topic 是 `/tf_static`，见 B1；属上游注释笔误。）

**A7. tf2 官方教程：静态变换用来表达"机器人底座与传感器"的关系，且只发一次。【一手文档明确】**
Humble 教程 `Writing-A-Tf2-Static-Broadcaster-Py`（[rst 源码](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.rst)）：
- 第 21 行：*"Publishing static transforms is useful to define the relationship between a robot base and its sensors or non-moving parts. For example, it is easiest to reason about laser scan measurements in a frame at the center of the laser scanner."*
- 第 135 行（示例代码注释）：*"The transforms are only published once at startup, and are constant for all time."*
- 第 411-416 行 "The proper way to publish static transforms"：*"In your real development process you shouldn't have to write this code yourself and should use the dedicated `tf2_ros` tool to do so."* 并给出 `static_transform_publisher` 的命令行与 launch 片段用法（第 416-449 行）。
- 第 387-388 行：发布后 `ros2 topic echo /tf_static` 应看到 *"a single static transform"*。

**A8. 官方 `static_transform_publisher` 只发布一次。【一手源码可见】**
`tf2_ros/src/static_transform_broadcaster_node.cpp:95-98`：构造函数里 `broadcaster_ = std::make_unique<...StaticTransformBroadcaster>(this); broadcaster_->sendTransform(tf_msg);`，之后没有任何 timer；`static_transform_broadcaster_program.cpp:405` 仅 `rclcpp::spin(node)`。即官方提供的静态外参发布节点**靠一次发布 + transient_local 存活**，而不是周期重发。

**A9. MoveIt 2 官方感知教程：安装外参用 launch 里的 `static_transform_publisher`；相机内部 frame 树用 URDF + RSP；配置 YAML 里没有外参。【一手文档明确 + 一手源码可见】**
- 教程正文（[perception_pipeline_tutorial.rst](https://raw.githubusercontent.com/moveit/moveit2_tutorials/main/doc/examples/perception_pipeline/perception_pipeline_tutorial.rst) 第 53 行）：*"...we must save both camera topics and tf topics due to the fact that MoveIt perception pipeline needs to listen TF's in order to convert the coordinates of pointcloud points according to `world` frame. Moreover, the reason of publishing static tf from world to camera frames in `depth_camera_environment.launch.py` is that it's necessary to determine transformation between robot and poincloud and that MoveIt's sensor plugins uses this transformation later."*
- `doc/examples/perception_pipeline/launch/depth_camera_environment.launch.py:66-73`：*"It is necessary to make transformation between world frame and camera frames enable later."* → `Node(package="tf2_ros", executable="static_transform_publisher", arguments=[*camera_1_pose, "world", "camera_1_base_link"])`；同文件 `:23-39` 另起 `robot_state_publisher` 加载相机自身的 `camera.urdf.xacro`。
- `doc/examples/perception_pipeline/config/sensors_3d.yaml` 全文只有 `sensor_plugin` / `point_cloud_topic` / `max_range` / `point_subsample` / `padding_*` / `max_update_rate` / `filtered_cloud_topic`，**没有任何外参或 frame 字段**。
- 消费侧 `moveit_ros/perception/pointcloud_octomap_updater/src/pointcloud_octomap_updater.cpp:262` 用 `tf_buffer_->lookupTransform(monitor_->getMapFrame(), cloud_msg->header.frame_id, cloud_msg->header.stamp, ...)` 把点云换到地图 frame；`moveit_ros/occupancy_map_monitor/src/occupancy_map_monitor.cpp:61-92` 把 `tf2_ros::Buffer` 作为构造入参。
→ 含义：主流 ROS 2 感知栈（MoveIt 2）**只用 TF 读传感器位姿**，不认识"外参参数"；但教程把外参的**真源**放在 launch 文件的 `static_transform_publisher` 参数里（即"配置→发布 TF"，而不是"从外部 TF 反读配置"）。

### 2.2 tf2_ros 静态变换的确切语义

**B1. `/tf_static` 的 QoS = depth 1 + TRANSIENT_LOCAL。【一手源码可见】**
`tf2_ros/include/tf2_ros/qos.hpp:63-71`：`StaticBroadcasterQoS : public rclcpp::QoS`，`explicit StaticBroadcasterQoS(size_t depth = 1) : rclcpp::QoS(depth) { transient_local(); }`；订阅端 `StaticListenerQoS`（`:53-61`）是 depth 100 + `transient_local()`。topic 名在 `tf2_ros/include/tf2_ros/static_transform_broadcaster.hpp:81-82`：`rclcpp::create_publisher<tf2_msgs::msg::TFMessage>(node, "/tf_static", qos, options)`。Python 侧（本机 `/opt/ros/humble/lib/python3.10/site-packages/tf2_ros/static_transform_broadcaster.py`）：`QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)`，同样是 `/tf_static`。

**B2. transient_local 的官方语义。【一手文档明确】**
[Humble About-Quality-of-Service-Settings](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)：第 49 行 *"*Transient local*: the publisher becomes responsible for persisting samples for "late-joining" subscriptions."*；第 80 行 *"The "durability" policy "transient local", combined with any depth, provides functionality similar to that of "latching" publishers."*；第 189-190 行 *"To achieve a "latched" topic that is visible to late subscribers, both the publisher and subscriber must agree to use 'Transient Local'."*
→ 含义：`/tf_static` 的"一次发布即可长期可见"来自 durability，不来自重发频率。

**B3. 静态帧的缓存完全忽略请求时间。【一手源码可见 + 本机实测】**
`tf2/src/static_cache.cpp:37-45`：`StaticCache::getData(TimePoint time, ...)` 直接 `data_out = storage_; data_out.stamp_ = time; return true;`（`error_str` 被 `(void)` 丢弃）。`:47-51` `insertData` 无条件覆盖并返回 true。`:53` `clearList() {}`。
本机离线复现（`tf2_py.BufferCore`，脚本见 §4/§5 附录）：静态链 `base_link→camera_link→camera_depth_frame→camera_color_optical_frame` 在 `Time(0)`、`stamp=1700000000` 与 `stamp=2000000000`（2033 年）三个时间点查到**完全相同**的平移；返回消息的 `header.stamp` 就是请求时间。
→ 含义：对纯静态链，"用 msg stamp 查"和"用 `Time()`（取最新）查"**永远同值**。本项目 `perception_bridge.py:295-326` 的两段式查找（先 stamp、失败再 latest）在静态链上不产生任何差异；它只对含动态帧的链有意义。

**B4. `cache_time` 只约束动态帧。【一手源码可见 + 本机实测】**
`tf2/src/buffer_core.cpp:308-317`：`allocateFrame()` 对 `is_static` 建 `StaticCache`，否则建 `TimeCache(cache_time_)`。`tf2_ros/include/tf2_ros/buffer.hpp:70-78` 的构造文档：`\param cache_time How long to keep a history of transforms`，默认 `tf2::BUFFER_CORE_DEFAULT_CACHE_TIME`。
→ 含义：`Buffer(cache_time=...)` 不会让静态外参"过期"；查询任意时间（含远超 cache_time 的将来/过去）都成功（实测 B3 的 2033 stamp 即为此证）。

**B5. Buffer 的清空（时钟跳变）不会清掉静态变换。【一手源码可见 + 本机实测】**
`tf2_ros/src/buffer.cpp:99-110`：`onTimeJump()` 在时钟源切换或**向后跳变**时 `RCLCPP_WARN("Detected jump back in time. Clearing TF buffer.")` 并 `clear()`。`tf2/src/buffer_core.cpp:165-177` 的 `clear()` 对每个 frame 调 `clearList()`；而 `tf2/src/static_cache.cpp:53` 的 `StaticCache::clearList()` 是空实现。
本机实测：`clear()` 之后 `base_link→camera_link` 与整条静态链仍可查到（值不变）。
→ 含义：静态外参在时钟跳变/复位后依然有效——**这既是优点（不会因时钟问题丢标定），也意味着"TF 还在"不能证明任何新鲜度**（与既有研究 `calibration-ssot-runtime-contract-20260911.md` 关于"时间戳本身不能证明新鲜度"的结论一致）。

**B6. 同一 child frame 被两个发布者写：静默覆盖，无异常、无警告。【一手源码可见 + 本机实测】**
`tf2/src/buffer_core.cpp:269-300` 的静态插入路径没有任何"重复/冲突"检测，`:291` 只是把 `frame_authority_[frame_number] = authority` 覆盖掉。对比：动态路径 `tf2/src/cache.cpp:254-270` 有"同时间戳完全重复则跳过"的去重逻辑。
本机实测：先用权威 `ssot_publisher` 发 `base_link→camera_link z=1.7`，再用 `legacy_param_publisher` 发同一条边 `z=9.9` → 查询得到 `z=9.9`，**无异常、无日志**；`tf2_py` 也不暴露任何"谁发布了该 frame"的查询接口（`BufferCore` 仅暴露 `all_frames_as_string` / `all_frames_as_yaml`）。
→ 含义：**"TF 成为第二条来源"这件事在运行时是不可观测的**。只有值本身能暴露分歧，且必须以另一个已知真值去比对。

**B7. 把安装外参发到 `/tf`（动态）会改变语义，并可能在 msg stamp 处抛异常。【一手源码可见 + 本机实测】**
`tf2/src/buffer_core.cpp:275-284`：同一 child frame 先有静态 cache、后来一次**非静态**插入时，会把该 frame 的 cache 类型从 `StaticCache` 换成 `TimeCache(cache_time_)`。
本机实测：先静态 `base_link→camera_link z=1.7`，再动态插入 `z=5.5` → `Time(0)` 查询得到 `5.5`；但对 `stamp=1700000000` 的查询变成 `ExtrapolationException: Lookup would require extrapolation at time 1700000000.000000, but only time 0.000000 is in the buffer`。
→ 含义：如果为了"能被某消费者看到"而把静态外参改为周期性发到 `/tf`（Orbbec 驱动的 `tf_publish_rate>0` 就是这种模式，见 D3），该边就不再"任意时间有效"，消息级时间戳查询可能失败或落在缓存窗口外。

**B8. "静态变换不该被反复重发"：官方有推荐做法，但没有强制或告警。【一手文档明确 + 一手源码可见】**
支持"发一次"的一手依据：A5（RSP README）、A7（教程注释 "only published once at startup"）、A8（官方 `static_transform_publisher` 只发一次）。
上游**没有**任何"重复发布静态变换"的报错或警告：`tf2/src/buffer_core.cpp` 全文的告警只有参数校验类（`:95`）与动态的 `TF_OLD_DATA`（`:294`）两种，无静态重复项。
C++ `StaticTransformBroadcaster` 的行为细节（`tf2_ros/src/static_transform_broadcaster.cpp:52-72`）：它把每次传入的变换**累积**进成员 `net_message_`（同 `child_frame_id` 则原地替换），每次 `sendTransform` 都把**累积的全部变换**整体发一遍；Python 版（本机 `tf2_ros/static_transform_broadcaster.py`）则只发本次传入的列表，不做累积。
→ 含义：重复重发不会被拒绝，只会把"启动一次的语义"退化为周期性消息（带来带宽/日志/顺序问题）；C++ 与 Python 的累积差异是 rclpy 项目需要留意的实现细节，但本项目当前不发布 TF。

**B9. tf2 的异常分级（与 fail-closed 直接相关）。【一手源码可见 + 本机实测】**
`tf2/src/buffer_core.cpp:795-812`：`TF2_CONNECTIVITY_ERROR → ConnectivityException`、`TF2_EXTRAPOLATION_ERROR → ExtrapolationException`、`TF2_LOOKUP_ERROR → LookupException`；`:84-97、:112-124` 的 frame 校验对空串/前导 `/` 走 `fillOrWarnMessageForInvalidFrame`。异常层级（本机实测）：`LookupException`、`ConnectivityException`、`ExtrapolationException`、`InvalidArgumentException`、`TimeoutException` **都继承自 `TransformException`**，彼此不是父子关系。
本机实测（`tf2_py.BufferCore`）：

| 场景 | 结果 |
| --- | --- |
| 目标/源 frame 从未出现在 buffer 中 | `LookupException: "base_link" passed to lookupTransform argument target_frame does not exist.` |
| 两个 frame 都存在但属于两棵不连通的树 | `ConnectivityException: Could not find a connection between 'base_link' and 'camera_color_optical_frame' because they are not part of the same tree.Tf has two or more unconnected trees.` |
| `source_frame` 为空串 | `InvalidArgumentException: ... in tf2 frame_ids cannot be empty` |
| `source_frame` 以 `/` 开头 | `InvalidArgumentException: ... in tf2 frame_ids cannot start with a '/'` |
| 同一 frame（target == source） | 直接返回 identity，`TF2_NO_ERROR` |

→ 含义：本项目 `perception_bridge.py:311-320` 只捕获 `LookupException / ConnectivityException / ExtrapolationException`；**`InvalidArgumentException` 不在其中**（例如 `input_frame` 配成空串或以 `/` 开头时）。该异常的实际传播后果（rclpy executor 是否吞掉回调异常）本研究未实测，列为未知项（第 6 节）。

### 2.3 未声明参数 override 的确切语义（源码级，见第 4 节展开）

**C1. rclcpp 官方头文件明文：未被显式声明的 override "根本不会出现在节点上"。【一手源码可见】**
本机 `/opt/ros/humble/include/rclcpp/rclcpp/node_options.hpp:345-358`（与 humble 分支 `rclcpp/include/rclcpp/node_options.hpp` 同文）：
```cpp
  /// Set the automatically_declare_parameters_from_overrides, return this.
  /**
   * If true, automatically iterate through the node's parameter overrides and
   * implicitly declare any that have not already been declared.
   * Otherwise, parameters passed to the node's parameter_overrides, and/or the
   * global arguments (e.g. parameter overrides from a YAML file), which are
   * not explicitly declared will not appear on the node at all, even if
   * `allow_undeclared_parameters` is true.
   * Parameter declaration from overrides is done in the node's base constructor,
   * so the user must take care to check if the parameter is already (e.g.
   * automatically) declared before declaring it themselves.
   * Already declared parameters will not be re-declared, and parameters
   * declared in this way will use the default constructed ParameterDescriptor.
   */
```

**C2. rclpy 的实现路径与 C1 一致。【一手源码可见】**
`rclpy/rclpy/node.py`（本机 `local/lib/python3.10/dist-packages/rclpy/node.py`，行号与 humble 分支一致）：
- `:203` `self._parameter_overrides = self.__node.get_parameters(Parameter)` —— override 来自本节点的 CLI/params-file（C 扩展 `rclpy/src/rclpy/node.cpp:340-356` 经 `_parse_param_overrides` 从 `use_global_arguments` + 本节点参数解析；`rcl/include/rcl/arguments.h:312-339` 的 `rcl_arguments_get_param_overrides` 是其底层）。
- `:472-474` **override 只在这里被使用**：`if not ignore_override and name in self._parameter_overrides: value = self._parameter_overrides[name].value`——即**必须先由代码 declare 这个名字**，override 才有机会生效。
- `:485-489` 已声明的名字再 declare 会抛 `ParameterAlreadyDeclaredException`。
- `:576-588` `get_parameter()` 对未声明名字抛 `ParameterNotDeclaredException`；`:603-626` `get_parameter_or(name, alt)` 则安全返回 `alt`。

**C3. 官方 How-To 的说法。【一手文档明确】**
[Humble Node-arguments](https://docs.ros.org/en/humble/How-To-Guides/Node-arguments.html)（"Setting parameters from YAML files" 小节）：*"Then either declare the parameters within your node with declare_parameter or declare_parameters, or set the node to automatically declare parameters if they were passed in via a command line override."*
→ 即：**override 生效有两个前提之一——显式声明，或打开 auto-declare**。该页没有出现 "ignored" 之类的措辞；"静默丢弃"的精确表述在一手来源中来自 C1 的头文件注释与 C2 的实现。

### 2.4 上游同类项目怎么做（源码级）

**D1. `orbbec_camera` 发布的 frame 与 TF。【一手源码可见】**
在线 `orbbec/OrbbecSDK_ROS2` 分支 `main`（`orbbec_camera` v1.5.22，与本项目文档中的启动命令 `depth_registration:=true` 同代）`orbbec_camera/src/ob_camera_node.cpp`：
- `:1391` `camera_link_frame_id_ = camera_name_ + "_link";`（`camera_name` 默认 `camera`，`:1390`）
- `:1407-1413` 每路流的 `frame_id_[i]` 默认 `<camera>_<stream>_frame`，`optical_frame_id_[i]` 默认 `<camera>_<stream>_optical_frame`
- `:2998-3046` `calcAndPublishStaticTransform()`：用设备内部外参 `stream_profile->getExtrinsicTo(base_stream_profile)` 生成"父 frame → 各 `*_frame`"以及 `*_frame → *_optical_frame`（后者固定 `setRPY(-π/2,0,-π/2)`，`:3001`）
- `:3055-3058` `publishStaticTF(..., camera_link_frame_id_, frame_id_[base_stream_])` → 发布 `camera_link → camera_depth_frame`
- `:3153-3165` `publishStaticTransforms()`：`StaticTransformBroadcaster` + `sendTransform(static_tf_msgs_)`；若 `tf_publish_rate_ > 0` 则另起线程用 `TransformBroadcaster` 周期重发（`:3167-3181`）
- 点云 header frame：`:2017-2019` `std::string frame_id = depth_registration_ ? optical_frame_id_[COLOR] : optical_frame_id_[DEPTH];`，若 `cloud_frame_id_` 参数非空（`:1504`，默认 `""`）则覆盖之。发布话题含 `depth_registered/points`（`:1762`）。
- 参数：`publish_tf`（默认 true，`:1454`）、`tf_publish_rate`（默认 0.0，`:1455`）、`depth_registration`（默认 false，`:1456`）。
- **该文件全文没有 `base_link`**（`grep -n "base_link" ob_camera_node.cpp` 无输出）。
- 另有一条"外参消息"路径：`:1846-1879` 以 `transient_local` 发布 `/<camera>/depth_to_color`、`/depth_to_ir`、`/depth_to_left_ir`、`/depth_to_right_ir`、`/depth_to_accel`、`/depth_to_gyro`（`orbbec_camera_msgs::msg::Extrinsics`）。
交叉核对：仓库内固定提交 `ce08bce`（orbbec_camera v2.9.3 / v2-main）的同一文件行号不同（`camera_link_frame_id_` 在 `:4374`、`cloud_frame_id_` 在 `:4542`、`publishStaticTransforms` 在 `:7286`），但上述结论逐条成立，且同样**不含 `base_link`**。

**D2. Orbbec 官方把安装外参放在 URDF fixed joint 里。【一手源码可见】**
`orbbec_description/urdf/test_gemini335L.urdf.xacro`（v1.5.22 main 分支）：
```xml
  <link name="base_link"/>
  <!-- Connect the virtual root to the actual base link -->
  <joint name="base_link_to_camera_link" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.011715 0.0475 0.014311" rpy="0 0 0"/>
  </joint>
```
同族文件 `gemini335L_336L.urdf.xacro` 描述相机内部 link/joint 树；`orbbec_description` 包内含 `launch/view_model.launch.py` 与 meshes。仓库内 `ce08bce` 副本的 `orbbec_description/urdf/test_gemini_335_L.urdf.xacro:8-12` 结构相同。
→ 官方惯例：**驱动负责相机内部 frame 树；"装到机器人上"这件事由用户在 URDF 里以 `base_link_to_camera_link` fixed joint 表达，再由 RSP 发成 `/tf_static`**。
附带低价值线索（不作为结论）：`orbbec_camera/scripts/static_transforms_publisher.py` 是一个把硬编码矩阵写死在校验脚本里的车载多相机示例，说明"驱动侧也存在用脚本发静态 TF"的做法，但它是示例脚本、无配置接口、无可审计来源（证据等级低）。

**D3. Orbbec 也支持把静态树低频发到 `/tf`。【一手源码可见】**
见 D1 的 `tf_publish_rate_ > 0` 分支（`:3160-3181`，日志 *"Publishing dynamic camera transforms (/tf) at %g Hz"*）。官方默认值是 `0.0`（launch `:152-153`）。
→ 这是"上游确实存在动态重发用法"的一手证据，但它把静态树发到**动态** topic，带来 B7 的后果；属"存在的事实"，不是"官方推荐"。

**D4. `livox_ros_driver2` 完全不发布 TF。【一手源码可见】**
`Livox-SDK/livox_ros_driver2` master：
- 全仓 `src/` 无 `tf2` / `StaticTransformBroadcaster` / `TransformBroadcaster` 引用（`grep -rln` 无输出；仓库内固定提交 `13eb05e` 副本同样为空）。
- `src/livox_ros_driver2.cpp:59` `std::string frame_id = "livox_frame";`；`:68` 从参数 `frame_id` 读取；`:136-146` `declare_parameter("frame_id", "frame_default")` 后从参数取实际值并传入 `Lddc`。
- `src/lddc.cpp:263 / 355 / 421` 把 `frame_id_` 写入 PointCloud2 / CustomMsg 的 `header.frame_id`；但 `:481` IMU 消息的 `frame_id` **硬编码 `"livox_frame"`**，不受 `frame_id` 参数影响。
- `launch_ROS2/*.launch.py` 全部以 `frame_id = 'livox_frame'` 作为默认（如 `msg_MID360s_launch.py:13,28`）。
→ 官方惯例：LiDAR 驱动的职责止于"把 frame_id 写进消息 header"；`livox_frame → base_link` 必须由使用者自己提供（URDF、`static_transform_publisher` 或应用侧参数/矩阵）。这也解释了本项目 `sensor_extrinsics.yaml` 里 `child_frame_lidar: livox_frame` 与"占位单位阵"的来源。

---

## 3. ROS 2 生态惯例：静态外参通常放在哪里

| 载体 | 一手依据 | 权威范围 | 主要缺点 / 注意事项 |
| --- | --- | --- | --- |
| **URDF fixed joint `<origin>` + robot_state_publisher** | A4、A5、A6、D2 | `base_link → <sensor frame>` 全段 | 数值会进入 URDF 这份**第二份拷贝**；URDF 通常由建模/导出工具生成，改数值要走重新生成/编辑 + 重启 RSP。RSP 只在 `robot_description` 变化时重发（A6） |
| **launch 里的 `tf2_ros static_transform_publisher`** | A7、A8、A9（MoveIt 2 教程用法） | 仅这一条边；相机内部树仍由 RSP/URDF 出 | 数值写在 launch 文件里（与 URDF 并列的第二处），且需与实际标定记录同步；一份 launch 一个节点，数量增长后难审计 |
| **驱动自带 TF（厂商内部外参）** | D1、D3 | 只覆盖设备内部（`camera_link → *_optical_frame`），**不含安装位姿** | 固件/驱动版本变化会改动这段树；把厂商内参和标定记录混用会产生"组合值和记录不一致"（见 §5） |
| **节点参数（应用私有）** | C1、C2、C3（override 语义）+ A9 反面（MoveIt 2 不读参数） | 只有本节点自己 | 其它消费者（MoveIt 2 感知插件、RViz、TF 工具）读不到；若同一数值在参数与 TF 中并存，即为双真源 |
| **独立消息 topic（如 `orbbec_camera_msgs/Extrinsics`）** | D1 | 需要自定义消费者 | 非通用约定，生态工具不认 |
| **专用标定记录文件 + 由它派生发布** | ADR 0004（本项目已定案）；上游无逐字等价表述 | 记录本身不是 ROS 接口 | 必须自己承担"派生发布"的正确性与唯一权威，否则仍会与 URDF/launch 副本分叉 |

**小结（本项目语境）**：ROS 2 生态里，"静态外参"的**事实标准接口是 TF**（MoveIt 2 感知栈、RViz 都只读 TF），而"数值放哪里"在官方层面有两条被推荐的落地方式：**URDF fixed joint（+RSP）** 或 **launch 里的 `static_transform_publisher`**。官方教程**没有**"从外部 TF 反读配置作为标定真源"这种模式；MoveIt 2 教程的模式是"配置 → 发布 TF → 消费方读 TF"。

---

## 4. 未声明参数 override 的确切语义（源码级 + 本机实测）

**语义链条（全部一手）**

1. 节点启动时，client library 按**本节点 FQN** 收集 CLI/params-file 里的 override：rclpy `node.py:203` → C 扩展 `rclpy/src/rclpy/node.cpp:340-356` → `rcl_arguments_get_param_overrides`（`rcl/include/rcl/arguments.h:312-339`）。
2. **只有代码 declare 的名字才会去查 override**：rclpy `node.py:472-474`；rclcpp 同语义在 `rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp:347-352`（`// Use the value from the overrides if available, otherwise use the default.`）。
3. 因此**未声明名字的 override 不会成为节点参数**：rclcpp 官方头文件明文（C1）：*"will not appear on the node at all, even if `allow_undeclared_parameters` is true"*。
4. `automatically_declare_parameters_from_overrides=True` 的官方语义（rclpy `node.py:212-219`）：把**该节点在参数文件/CLI 里出现的所有键**逐个隐式声明——`self.declare_parameters('', [(name, param.value, ParameterDescriptor()) for name, param in self._parameter_overrides.items()], ignore_override=True)`；rclcpp 对应实现（`node_parameters.cpp:103-115`）用 `descriptor.dynamic_typing = true`，注释为 *"If asked, initialize any parameters that ended up in the initial parameter values, but did not get declared explcitily by this point."*
5. 类型由 YAML/CLI 决定：rclcpp 走 `dynamic_typing`（无类型约束）；rclpy 用**默认构造的 `ParameterDescriptor`**，随后在 `node.py:461-470` 按值推断类型（`descriptor.type = Parameter.Type.from_parameter_value(value).value`）。

**本机实测（Humble / rclpy 3.3.21，离线，无 ROS 图）**

实测 1（默认关闭 auto-declare）：

```python
# /tmp/t_params.yaml:  t_node: {ros__parameters: {declared_one: 0.25, undeclared_legacy: [1.0, 2.0]}}
n = Node("t_node", cli_args=["--ros-args","--params-file","/tmp/t_params.yaml"],
         automatically_declare_parameters_from_overrides=False)
# 输出：
# declared 'declared_one'? False        <- override 存在，但未声明前它还不存在
# declared 'undeclared_legacy'? False
# declared_one value: 0.25              <- declare 之后 override 生效（默认值 0.5 被覆盖）
# all params: ['declared_one', 'use_sim_time']   <- undeclared_legacy 从未出现，无异常、无警告
```
结论：**未声明名字的 params-file override 被静默丢弃**（节点侧完全不可见；`get_parameter_or` 只能拿到调用方给的替代值）。这正是 C1 头文件注释的行为在 rclpy 上的实测确认。

实测 2（打开 auto-declare）：

```python
n = Node("t_node", cli_args=[...], automatically_declare_parameters_from_overrides=True)
# AUTO 'declared_one'  type=Type.DOUBLE        value=0.25
# AUTO 'matrix_double' type=Type.DOUBLE_ARRAY  value=[1.0, 0.0, 0.0, 4.5]
# AUTO 'matrix_int'    type=Type.INTEGER_ARRAY value=[1, 0, 0, 0]
# AUTO 'use_sim_time'  type=Type.BOOL          value=False
# redeclare raised: ParameterAlreadyDeclaredException ('Parameter(s) already declared', ['declared_one'])
# declare fresh_name ok -> 1.0
```
结论（回答任务里的两个具体问题）：
- **会**把配置里所有键都变成已声明参数；
- **类型由 YAML 决定**：浮点数组 → `DOUBLE_ARRAY`、整数数组 → `INTEGER_ARRAY`（同一 4x4 外参矩阵写成 `1` 还是 `1.0` 会得到不同参数类型）；
- **风险**：任何随后显式 `declare_parameter` 同名参数都会抛 `ParameterAlreadyDeclaredException`（实测），所以它只适用于"从不显式声明、全靠 override"的节点；对像 `perception_bridge` 这种显式声明全部字段的节点，打开它会直接启动失败。

---

## 5. 本项目适用性

### 5.1 现状事实（本仓库，只读核对）

| 位置 | 事实 |
| --- | --- |
| `src/robot_safecontrol_moveit/perception_bridge.py:264-272` | `_declare_parameters()` 里**显式声明**了 `use_tf`（默认 `False`）、`camera_to_world_static`、`lidar_to_world_static`（默认 4x4 单位阵）；节点构造为 `super().__init__("perception_bridge")`（`:123`），**未开启** auto-declare |
| `perception_bridge.py:295-326` | `_sensor_to_world()` 顺序：`use_tf and buffer and world != sensor` → `lookup_transform(world, sensor, stamp)` → 失败再 `lookup_transform(world, sensor, Time())` → 再失败 → `get_parameter(static_param_name)` → 仍为空则 `_identity_extrinsics()` |
| `perception_bridge.py:303-326` | `except (LookupException, ConnectivityException, ExtrapolationException)`（`:311`、`:319`）；`static = self.get_parameter(static_param_name).value`（`:323`）→ `_identity_extrinsics()`（`:326`）；`_use_tf` 控制是否创建 `Buffer`/`TransformListener`（`:183-186`） |
| `perception_bridge.py:351` | 传给 TF 的是**配置的** `input_frame`，不是 `msg.header.frame_id` |
| `config/perception_runtime.yaml:39,52,56` | 仍带 `camera_to_world_static`（假定安装位姿）、`lidar_to_world_static`（单位阵占位）、`use_tf: false`；文件头注明 T7 会把矩阵移走 |
| `config/perception_dual_sensor_real.yaml:29,39` | 同样复制了 `camera_to_world_static` 与 `use_tf: false` |
| `launch/mujoco_transition_final.launch.py:205-214` | 生产入口把 `perception_runtime.yaml` 作为 `params-file` 传给 `perception_bridge`；同文件 `:104-118` 启动 `robot_state_publisher`（URDF 来自 `moveit_params["robot_description"]`），**没有**任何 `static_transform_publisher` |
| `models/ninezzhou/urdf/ninezzhou.urdf` | link 只有 `base_link, Link1..Link9, tool0`；joint 只有 `J1..J9, tool0_fixed`——**没有任何相机/雷达 link 或固定 joint** |
| `config/sensor_extrinsics.yaml` | 记录 `parent_frame: base_link`、`child_frame: camera_color_optical_frame`、`child_frame_lidar: livox_frame`，矩阵为**单位阵占位**；ADR 0004 已定其为唯一 calibration authority |

### 5.2 TF 分支在今天是否可达（本机实测 + 源码）

把上面的 facts 拼起来做个定向实测（`tf2_py.BufferCore` 离线复现，2026-09-14 本机）：

| 复现场景 | 结果 |
| --- | --- |
| 只有 Orbbec 内部静态链（`camera_link→camera_depth_frame→camera_color_optical_frame`），无 `base_link` 相关边 | `LookupException: "base_link" passed to lookupTransform argument target_frame does not exist.` |
| 机器人树（`base_link→Link1`，RSP 角色）+ Orbbec 树（`camera_link→...`）并存，二者不连通 | `ConnectivityException: Could not find a connection between 'base_link' and 'camera_color_optical_frame' because they are not part of the same tree.Tf has two or more unconnected trees.` |

→ **推断（需实测确认）：在当前 URDF（无 sensor link）+ 当前 launch（无 `static_transform_publisher`）+ Orbbec 驱动（树根是 `camera_link`）三者组合下，`use_tf: true` 只会得到异常与警告日志，得不到任何变换。** 也就是说：保留 TF 分支今天既不提供数据，也不会算错——它只是"死代码 + 每次点云一条 warn"。（Offline 复现与真机唯一差别是"谁把哪些边放进 buffer"，不改变 tf2 的树/异常语义；真机上如需闭环确认，只需 `ros2 run tf2_tools view_frames` 看 `base_link` 与 `camera_color_optical_frame` 是否同树。）

### 5.3 如果 TF 分支被"修好"，它会成为第二条来源

要让它可达，就必须有人补上 `base_link → camera_link`（URDF fixed joint 或 launch 的 `static_transform_publisher`）。此时：

1. `lookup_transform(base_link, camera_color_optical_frame, stamp)` 的返回值 = `T(base_link←camera_link)`（安装位姿） ∘ `T(camera_link←…←camera_color_optical_frame)`（**Orbbec 固件/驱动给出的设备内参**，D1）。
2. 而 `sensor_extrinsics.yaml` 记录的是**整段** `base_link ← camera_color_optical_frame` 矩阵（文件头注释：*"把 child_frame 下的点变换到 parent_frame"*）。两者的相等关系依赖"记录里的设备内参部分 == Orbbec 驱动上报的内参"，**没有任何机制保证**，也没有机制检测（B6：静默覆盖、不可观测）。
3. ADR 0004 的硬约束是 *"T6 校对的是节点实际加载的 ID/hash"*、*"record == runtime 加载值"*。TF 组合出来的值**不进入**这条等价性检查——CBF 实际用的矩阵会随 `use_tf` 开关在"记录值"与"TF 组合值"之间切换，而两种情况下上报的 `calibration_id/hash` 都是同一个。这是结构性违反 SSOT，而不是精度问题。
4. 另外，TF 值**不受 freshness/身份约束**：静态边一次发布后永久有效（B3、B5），时钟跳变清 buffer 也不影响它（B5）。因此"TF 查到了"不能作为"标定有效"的任何证据。

### 5.4 旧参数路径今天是否仍然生效

**是。** `camera_to_world_static` / `lidar_to_world_static` 在 `:264-272` **被显式声明**，所以在 `perception_runtime.yaml`（经 launch 传入）里的值会被接受并生效（§4 实测 1 的 `declared_one` 即此机制）。`use_tf: false` 时它就是唯一的矩阵来源。
→ 换言之，现状不是"探测旧参数"，而是**两条都活的写入路径**：`sensor_extrinsics.yaml`（ADR 0004 规定的真源，当前尚未被消费，见 ADR 0004 与 handoff 27）与 `perception_runtime.yaml` 的 ROS 参数（真正进 CBF 的值）。这正是 ADR 0004 开头描述的 split-brain。

### 5.5 对 OFF-01 取舍的直接含义

1. **TF 分支**：保留它需要回答"它服务哪个场景"。按 A1/A9 的惯例，TF 的正确角色是**派生产物**（"由 SSOT 记录发布 `base_link→<sensor frame>` 到 `/tf_static`，供 MoveIt 2/RViz 消费"），而不是**输入**（"从外部 TF 读取本节点的标定矩阵"）。若 OFF-01 只做"统一身份、拒绝第二条来源"，则**删除以 TF 为输入的读取分支**与一手惯例一致；若将来需要 TF 给 MoveIt 2 用，应另开"由记录派生发布"的票，并要求发布者唯一（A1）、一次性发布 + `TRANSIENT_LOCAL`（B1/B2/A7/A8）、且明确记录"TF 是派生、不是真源"。
2. **旧参数**：不要只做"删掉声明"——那会让 params-file 里的外参 override **静默失效**（§4 实测 1、C1），把"配置写错却运行"从显式错误变成无声行为。更安全的最小改动是**保留读取但改为显式拒绝**：加载记录后用 `get_parameter_or(...)`（不抛异常，`:603-626`）比对参数值，若与记录不一致就报错/拒绝模式；或保留声明 + 比对 + 拒绝。两种做法的共同点是"不留静默路径"。
3. **identity 兜底**：`:326` 的 `_identity_extrinsics()` 是 fail-open 的（"相机与 base_link 重合"是一套具体的、几乎必然错误的几何）。OFF-01 若要求 fail-closed，应把"TF 与参数都拿不到"变成明确的拒绝/未知状态，而不是 identity。
4. **双向一致性检查的可行性**：如果决定长期允许 TF 与记录并存，必须自己实现比对（上游不提供冲突检测，B6），并规定"谁负责发布那条权威边、谁负责声明 `calibrated` 状态"。

---

## 6. 未知项清单

1. 【未知】Orbbec 官方是否有**文字**规范/文档明确写"安装外参应写入 URDF fixed joint"。本研究只找到源码与官方 xacro 示例（D2），未找到逐字表述；README 内容未逐条核读。
2. 【未知】上游是否存在"禁止/不推荐重复发布静态变换"的正式声明（issue、设计文档或文档措辞）。已确认的是：官方工具只发一次（A7/A8）、上游没有对应的运行时告警（B8）。
3. 【未知】`rcl_yaml_param_parser` 对**混合类型序列**（如 `[1.0, 0, 0]`）的解析行为，以及它对本项目外参矩阵（写 `1.0` 还是 `1`）的实际影响。本研究只实测了"全浮点 → DOUBLE_ARRAY、全整数 → INTEGER_ARRAY"，未测混合情形，也未测 `ros2 launch` 传参路径下 rcl **非** rclpy 路径的差异。
4. 【未知】`use_tf: true` 时 `InvalidArgumentException`（`input_frame` 为空串/带前导 `/`）在 rclpy 回调中的实际后果（是否被 executor 吞掉、是否导致点云被静默丢弃）。B9 只证实了异常类型与消息。
5. 【未知】真机上 `base_link` 与 `camera_color_optical_frame` 是否同树（`ros2 run tf2_tools view_frames` 未运行）。§5.2 的不可达结论是离线复现 + 源码推导。
6. 【未知】`camera_color_optical_frame` 在实机 Orbbec 驱动中是否确实由驱动发布（本项目配置假定 `depth_registration:=true` 时点云 frame 为 `camera_color_optical_frame`；D1 的源码路径支持该结论，但未在设备上核验）。
7. 【未知】Orbbec 驱动的设备内参与 `sensor_extrinsics.yaml` 记录的等价性（若将来走 §5.3 的 TF 组合，这决定分歧大小）。无一手依据。
8. 【未知】`sensor_extrinsics.yaml` 的**数值本身**（当前为单位阵占位）与 `perception_runtime.yaml` 里假定安装位姿的一致性验收；属 T5/T6 现场工作，不在本研究范围。
9. 未涉及（避免与并行任务重复）：雷达与相机的时间戳对齐、时间域/freshness 语义。

---

## 7. 来源列表

**规范与官方文档**

1. REP-105 *Coordinate Frames for Mobile Platforms*（Status: Active）— https://www.ros.org/reps/rep-0105.html （章节：Specification / Relationship between Frames / Frame Authorities）
2. REP-103 *Standard Units of Measure and Coordinate Conventions* — https://www.ros.org/reps/rep-0103.html （章节：Coordinate Frame Conventions / Suffix Frames / Rotation Representation）
3. URDF XML 规范 `<joint>`（`<origin>` 定义）— http://wiki.ros.org/urdf/XML/joint
4. robot_state_publisher 官方 README（Humble, 3.0.3）— https://docs.ros.org/en/humble/p/robot_state_publisher/
5. tf2 教程 *Writing a static broadcaster (Python)* 的 rst 源 — https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.rst
6. *About Quality of Service Settings*（Humble）— https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst
7. *Passing ROS arguments to nodes via the command-line / Node-arguments*（Humble）— https://docs.ros.org/en/humble/How-To-Guides/Node-arguments.html
8. ROS 2 参数设计文章 *Parameter API design in ROS* — https://design.ros2.org/articles/ros_parameters.html （注：其 "Parameter initialization" 与 "Predeclared interface" 两节明确标为 "Topics not covered at the moment"，因此 override/声明语义的权威来源是 client library 源码与 How-To）
9. MoveIt 2 感知教程 — https://raw.githubusercontent.com/moveit/moveit2_tutorials/main/doc/examples/perception_pipeline/perception_pipeline_tutorial.rst ；配套 `config/sensors_3d.yaml` 与 `launch/depth_camera_environment.launch.py`

**官方源码（在线 humble / main / master，读取于 2026-09-14）**

10. `ros2/geometry2` humble：`tf2_ros/include/tf2_ros/qos.hpp:53-71`、`tf2_ros/include/tf2_ros/static_transform_broadcaster.hpp:81-82`、`tf2_ros/src/static_transform_broadcaster.cpp:52-72`、`tf2_ros/src/static_transform_broadcaster_node.cpp:95-98`、`tf2_ros/src/static_transform_broadcaster_program.cpp:405`、`tf2_ros/src/buffer.cpp:45-110`、`tf2_ros/include/tf2_ros/buffer.hpp:70-78`、`tf2/include/tf2/time_cache.hpp:170-196`、`tf2/src/static_cache.cpp:37-77`、`tf2/src/cache.cpp:228-283`、`tf2/src/buffer_core.cpp:84-124, 165-177, 269-317, 409-448, 475-487, 795-812, 1049-1104`
11. `ros/robot_state_publisher` humble：`src/robot_state_publisher.cpp:129-150, 255-296, 363-423`、`include/robot_state_publisher/robot_state_publisher.hpp:101-113, 151-152`
12. `ros2/rclcpp` humble：`rclcpp/include/rclcpp/node_options.hpp:340-362`、`rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp:98-116, 330-391, 470-529`、`rclcpp/src/rclcpp/detail/resolve_parameter_overrides.cpp:26-72`
13. `ros2/rclpy` humble：`rclpy/rclpy/node.py:116-219, 339-500, 559-626, 960-998`；`rclpy/src/rclpy/node.cpp:340-356`
14. `ros2/rcl` humble：`rcl/include/rcl/arguments.h:306-339`
15. `moveit/moveit2` main：`moveit_ros/perception/pointcloud_octomap_updater/src/pointcloud_octomap_updater.cpp:230-290`、`moveit_ros/occupancy_map_monitor/src/occupancy_map_monitor.cpp:61-92`
16. `orbbec/OrbbecSDK_ROS2` main（orbbec_camera 1.5.22）：`orbbec_camera/src/ob_camera_node.cpp:1390-1460, 1846-1879, 2017-2019, 2998-3059, 3153-3181`、`orbbec_camera/launch/gemini_330_series.launch.py:72-80, 152-153`、`orbbec_description/urdf/test_gemini335L.urdf.xacro`、`orbbec_description/urdf/gemini335L_336L.urdf.xacro`、`orbbec_camera/scripts/static_transforms_publisher.py`；`v2-main`（2.9.3）同文件交叉核对
17. `Livox-SDK/livox_ros_driver2` master：`src/livox_ros_driver2.cpp:53-87, 129-159`、`src/lddc.cpp:45-71, 263, 355, 421, 481`、`launch_ROS2/msg_MID360s_launch.py:13,28`

**仓库内固定提交副本（可离线核对）**

18. `/home/lsn/robot/robot_safecontrol/.scratch/oscbf-reuse-wayfinder/31/online-2026-09-09/upstream/OrbbecSDK_ROS2`（commit `ce08bce25f7a0a6fe939ece87ec945447109581f`，2026-07-17，v2-main / orbbec_camera 2.9.3）：`orbbec_camera/src/ob_camera_node.cpp:4374, 4542, 5369, 5660, 7166, 7286-7290`、`orbbec_description/urdf/test_gemini_335_L.urdf.xacro:8-12`
19. 同目录 `.../upstream/livox_ros_driver2`（commit `13eb05e4e6dd7a765b934d0c5fd6236676a57b49`，2026-04-14）：`src/` 内无 tf2 引用
20. 同目录 `.../upstream/Livox-SDK2`（commit `f5d9375f84efe2b15bc0a052d3e18482ed13adf4`）

**本仓库内部依据**

21. `docs/adr/0004-calibration-ssot-sensor-extrinsics-authority.md`
22. `docs/planning/oscbf-reuse/handoffs/27-calibration-ssot.md`
23. `docs/planning/oscbf-reuse/research/calibration-ssot-runtime-contract-20260911.md`

**本机实测环境与复现**

24. ROS 2 Humble（`/opt/ros/humble`）：`tf2_ros 0.25.22`、`rclcpp 16.0.19`、`rclpy 3.3.21`；Python 3.10.12。复现方式（离线，无 ROS 图/无硬件）：

```bash
# (a) tf2 静态语义：tf2_py.BufferCore 上 set_transform_static / clear / 双权威覆盖 / 动态替换
source /opt/ros/humble/setup.bash && python3 /tmp/tf_test2.py   # 见 §2.2 B3-B9 的表格
# (b) 参数 override：以 --params-file 传入 YAML，比较 has_parameter / declare 结果 / 自动声明类型
source /opt/ros/humble/setup.bash && python3 /tmp/param_test.py   # §4 实测 1
source /opt/ros/humble/setup.bash && python3 /tmp/param_test2.py  # §4 实测 2
```
（脚本为本研究在 `/tmp` 下的一次性草稿，未写入仓库；逻辑已在 §2.2 与 §4 内联。）
