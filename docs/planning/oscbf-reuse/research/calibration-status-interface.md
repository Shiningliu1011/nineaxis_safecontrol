# 标定文件身份的状态接口研究

- 日期：2026-09-09。
- 状态：研究完成；用户随后于 2026-09-09 回复“可以接受”，采纳保留健康接口并新增标准诊断通道的建议。详见 ADR 0004 补充决议；本文其余实现细节仍为工程建议。
- 方法：核对仓库源码、本机 ROS 2 Humble 消息定义及 ROS 官方 Humble 源码。docs.ros.org 部分页面拒绝访问，改读 ROS 官方 GitHub 的 Humble 分支原文。没有修改运行代码，没有实机验证。

## 推荐回答

**保留 `/perception/status` 的十个健康数值，新增 `/perception/calibration_status`，使用 `diagnostic_msgs/msg/DiagnosticArray` 上报标定身份与校验状态。** 这是结合现有契约作出的工程建议，不是 ROS 强制的话题命名。

可以把旧话题理解为“仪表盘”：告诉我们相机、雷达是否在线、数据有多旧。新话题是“标定身份证”：告诉我们正在运行的程序究竟读了哪份标定文件。这样补足身份核验，不必同时更换现有仪表盘接口。

修正上一轮交接稿的一点：**不优先使用 `std_msgs/String` 包装 JSON**。ROS 已有诊断消息，足以表达这次的路径、文件指纹、标定编号及检查结果。采用 JSON 还要自行约定字段与解析；标准诊断消息也需要明确键名，但无需再加一层 JSON。

## 仓库现状与实际矛盾

1. [bridge 的发布器与 `_publish_status`](../../../../src/robot_safecontrol_moveit/perception_bridge.py) 将 `/perception/status` 定义为 `Float32MultiArray`，依次发布 `camera_alive`、`lidar_alive`、`camera_age`、`lidar_age`、`camera_used`、`lidar_used`、`fusion_stamp`、`fusion_age`、`source_count`、`perception_valid`。现有发布使用 `qos_profile_sensor_data`。
2. [双传感器规格](../../../specs/dual_sensor_perception_fusion_spec.md) 明确了这十个字段的接口。[ADR 0004](../../../adr/0004-calibration-ssot-sensor-extrinsics-authority.md) 则要求把实际加载的绝对路径、内容 SHA-256 和 calibration_id 写入同名话题。这是既有两份契约之间的表示冲突。
3. [上一轮交接稿](../handoffs/27-calibration-ssot.md) 的新增通道方向合理；本报告将其中“标准消息承载 JSON”的建议细化为现成诊断类型。
4. 本机 `/opt/ros/humble/share/diagnostic_msgs/msg/` 存在所需消息定义；[package.xml](../../../../package.xml) 尚未声明 `diagnostic_msgs`。未来实施时需显式声明依赖，不能仅依赖本机碰巧安装。

## 官方事实与其设计含义

| 官方事实 | 对本项目的含义（推论） |
|---|---|
| `Float32MultiArray.data` 类型为 `float32[]`；官方 Humble 定义标明该示例消息自 Foxy 起弃用，建议语义明确的消息。[定义](https://raw.githubusercontent.com/ros2/common_interfaces/humble/std_msgs/msg/Float32MultiArray.msg) | 不能直接将文件路径、完整文本 hash 放入数值数组。暂时保留旧接口是兼容性取舍，不是推荐它用于新设计。 |
| `MultiArrayLayout` 描述多维数组的维度、跨度和偏移，其 label 用于维度名。[定义](https://raw.githubusercontent.com/ros2/common_interfaces/humble/std_msgs/msg/MultiArrayLayout.msg) | 技术上可以往字符串 label 塞文本，但那会借用数组布局字段装业务身份，不推荐。 |
| `String` 只有一个字符串 data 字段，官方同样建议语义明确的消息。[定义](https://raw.githubusercontent.com/ros2/common_interfaces/humble/std_msgs/msg/String.msg) | String 能传 JSON，却没有原生诊断级别、时间戳或字段结构；并非这次的首选。 |
| `DiagnosticArray` 专门传机器人状态诊断，含时间戳 Header 与组件状态列表。[定义](https://raw.githubusercontent.com/ros2/common_interfaces/humble/diagnostic_msgs/msg/DiagnosticArray.msg) | 适合一次报告文件及多传感器标定状态。 |
| `DiagnosticStatus` 含 OK/WARN/ERROR/STALE、name、message、hardware_id、values；`KeyValue` 的 key/value 都是字符串。[状态定义](https://raw.githubusercontent.com/ros2/common_interfaces/humble/diagnostic_msgs/msg/DiagnosticStatus.msg)、[键值定义](https://raw.githubusercontent.com/ros2/common_interfaces/humble/diagnostic_msgs/msg/KeyValue.msg) | 可直接装路径/hash/编号和检查结果；hardware_id 应留给硬件身份，文件 hash 应在 values 中。STALE 枚举本身不会自动执行超时判定。 |

## 最小接口建议

以下均为待采纳的项目约定：

- 旧健康话题保持现有名称、类型、字段顺序。
- 新话题类型为 `DiagnosticArray`，固定组件名如 `perception/calibration/camera`、`perception/calibration/lidar`，每源一条状态，携带 `schema_version`、`node_fqn`、`session_id`、`resolved_path`、`file_sha256`、`calibration_id`、`calibrated`、`validation_result`。节点重启产生新 session；字段来自节点实际加载的同一份内容，不能报告时另读一个可能已经变化的文件冒充已加载内容。
- `header.stamp` 表示本次诊断报告时间；必要时另有 loaded_at 标记加载时间。每次重发只更新时间戳，身份仍描述实际使用中的内容。
- 文件有效加载、未标定假定位姿、文件加载失败分别给出明确检查结果。`OK` 的含义须限定为这里的身份/加载检查通过，不能被消费方解释成整机可以安全运动。
- 检查方按固定键名和版本解析，必填字段缺失、多个发布者身份含混或内容不匹配均不放行。`DiagnosticArray` 是承载格式，不会自动完成一致性检查。
- 将来若这些字段进入复杂的严格类型控制协议，再评估专用消息；当前没有必要为这张身份报告迁移整个健康接口。

文件路径说明“从哪里读”，SHA-256 用于比对“内容是否相同”，calibration_id 说明“是哪次标定”。**内容一致仅解决是否用对文件；标定本身是否准确仍要靠既定实测验收。** 此区别对应 [ADR 0004 的运行时一致性要求](../../../adr/0004-calibration-ssot-sensor-extrinsics-authority.md) 与 [ADR 0003 的已验证标定要求](../../../adr/0003-perception-input-failure-strategy.md)。

## 传输与陈旧报告

官方事实：Reliable 会重试传送；Transient Local 由发布者保存消息供迟加入订阅者获取；Keep Last 的 depth 控制保留数量。要收到历史样本，发布和订阅双方都应使用 Transient Local；仅发布侧设置而订阅使用 Volatile，只能收到新消息。Lifespan 限制样本从发布到接收的有效时间；默认时长通常意味着无限。[ROS Humble QoS 文档源码](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)

工程建议：新通道双方显式使用 Reliable、Transient Local、Keep Last(1)，启动时及状态改变时发布，并周期重发供检查方判断活性。具体周期/超时在实施契约中一起确定。它让后启动的自检能拿到身份，但**收到保留报告不等于当前实例仍正常**；检查方还须验证当前 node/session、报告新鲜度和当前健康输入，超时失效。有限 Lifespan 可作补充，不能撤销订阅者已经缓存的数据，也不能代替本地超时。不要宣称 Transient Local 会跨发布进程重启永久保存状态：官方定义的持久责任在发布者。

## 为什么不直接更换旧话题类型

整体迁移能统一状态，但会扩大本票需要同步变更的规范、发布代码、工具与消费者范围。保留十字段并新增诊断通道只为缺失的身份信息补接口，故当前优先推荐；不声称已经穷尽仓库外消费者。

如果采纳，后续需同步修订 ADR 0004 的上报话题名称、交接稿和启动自检契约，再实施并验证迟加入、进程重启、标定不匹配、报告过期等行为。本次研究没有执行这些变更，也没有关闭决策票。
