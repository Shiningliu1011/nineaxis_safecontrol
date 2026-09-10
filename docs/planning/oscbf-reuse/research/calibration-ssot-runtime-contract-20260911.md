# 标定 SSOT 接线：运行时身份、诊断与规范化补充研究

日期：2026-09-11。范围：补齐 [标定 SSOT 接线](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/27) 的外部接口事实；不实施、不关闭实施票。依据本仓库 CONTEXT.md、ADR 0004 和 [既有交接](../handoffs/27-calibration-ssot.md)。下文“建议”尚非用户定案，任何周期、超时或机械容差均未实测。在线来源读取于本日；Humble 分支与 colcon released 文档不等同本机锁定版本。

## 结论

继续使用 ament share 加载单份标定记录与 DiagnosticArray，不需要换技术路线。需要补上三个精确边界：ament 查找路径与物理 resolved 路径不同；诊断消息类型不自带实例认证或新鲜度；原始字节 hash 与规范化 record ID 是两种身份，必须明确定义规范化输入域。建议按下列契约准备实施，数值参数和启动准入接口留待相关票据定案。

## 1. ament share 与 symlink install

**事实**：Humble `get_package_share_directory()` 从包前缀拼接 `share/<package>`，不调用 `resolve()`；share 目录不存在时仅警告并返回路径。`get_package_prefix()` 找不到索引才抛 PackageNotFoundError。[Humble packages.py](https://raw.githubusercontent.com/ament/ament_index/humble/ament_index_python/ament_index_python/packages.py)

**事实**：资源查找按环境搜索路径查找包标记，因此不同 overlay 环境可以解析不同安装前缀；同一个包名并不构成跨进程的唯一文件身份。[Humble resources.py](https://raw.githubusercontent.com/ament/ament_index/humble/ament_index_python/ament_index_python/resources.py)

**事实**：colcon 对 `--symlink-install` 的保证是“在可行处”用链接替代复制；不是所有文件都保证链接。Python `Path.resolve(strict=True)` 展开符号链接、得到绝对路径并检查存在。[colcon build](https://colcon.readthedocs.io/en/released/reference/verb/build.html)、[Python pathlib](https://docs.python.org/3.10/library/pathlib.html#pathlib.Path.resolve)

**建议**：先经 ament 得到 `lookup_path`，再得到 `resolved_path`，两个字段都记录。允许 ament 下的合法链接最终指向源码位置；这仍是经安装索引查找，不能误判为私自源码回退。ADR 中“两物理副本”应理解为复制安装场景；symlink 场景不保证该事实。也不能只以是否执行了重新 build 判断生效，应重启后核对节点实际快照。

**建议**：一次 `read_bytes()`，同一 bytes 做 SHA-256 与 YAML 解析，冻结校验后的运行记录。启动失败不得退回源码文件或 identity。读取完成之后文件或链接改动不应改变当前实例使用的矩阵；后续诊断报告内存中已加载身份，不重新读取磁盘伪装成运行值。部署时避免并发原位覆写，使用完整文件替换；单次读取本身不是针对任意并发覆写的事务保证。

## 2. DiagnosticArray 的表达能力与限制

**事实**：DiagnosticArray 只有 Header 和 DiagnosticStatus 数组。DiagnosticStatus 含 level、name、message、hardware_id 和 KeyValue 数组；level 的 OK/WARN/ERROR/STALE 常量不会自动进行超时判断，hardware_id 的定义是硬件唯一字符串。它没有节点启动实例字段，也没有内建准入布尔值。[Humble DiagnosticArray](https://raw.githubusercontent.com/ros2/common_interfaces/humble/diagnostic_msgs/msg/DiagnosticArray.msg)、[Humble DiagnosticStatus](https://raw.githubusercontent.com/ros2/common_interfaces/humble/diagnostic_msgs/msg/DiagnosticStatus.msg)

以下为**建议键名**，应在实施前固定 schema 版本；所有 KeyValue 值按明确格式转字符串，不依赖 Python repr。

| 位置 | 建议内容 | 意义 |
| --- | --- | --- |
| Header.stamp | 本次报告生成的 ROS 时间 | 不写成标定日期或最初加载时间 |
| 汇总 status.name | `<节点全名>/calibration` | 人类可识别的组件名 |
| values | `schema_version`, `node_fqn`, `instance_id`, `report_seq` | 每次节点构造生成新实例 ID，序列在实例内递增 |
| values | `lookup_path`, `resolved_path`, `file_sha256` | 同一启动读取快照的来源和文件身份 |
| 每源 status.name | `<节点全名>/calibration/<source>` | camera/lidar 独立状态 |
| 每源 values | `source`, `enabled`, `calibration_id`, `frame_from`, `frame_to`, `math_valid`, `provenance_valid`, `calibrated` | 不把数学合法等同于实测标定 |
| 每源 hardware_id | 已知真实传感器序列号 | 未知则空，另有显式未知状态；不用进程 UUID 冒充硬件身份 |

**建议**：未标定假定矩阵可为调试几何保持兼容，但诊断必须明确不可避障准入；不要让汇总 OK 掩盖 `calibrated=false`。外部实测验收、健康锁存、人工恢复等状态只在实际拥有证据的组件报告，加载器不得自行宣称已验收。

### 新鲜度不能仅由时间戳或实例 ID 推断

**事实**：ROS 时间可能暂停、跳变，启用仿真时间而没有时钟数据时可能为零；官方设计将 steady time 留给超时等内部用途。[ROS 2 Clock and Time](https://design.ros2.org/articles/clock_and_time.html)

**建议**：消费者使用本地单调时钟检查“最后收到有效当前实例报告至今”的间隔；Header.stamp 用于有共同时间基准时的报告年龄检查，零值、向未来跳变或时钟重置不能自动通过。标定 timestamp 是 provenance 日期，与报告新鲜度分离。跨机器直接比较两个 monotonic 时间值无意义。

**建议**：实例 ID 只区分重启，不能仅凭“收到的第一个 ID”证明它是当前实例。T6 必须从当前启动编排获得预期实例标识，或在独立握手中取得当前实例并要求后续序列推进；重启、实例变化、多发布者冲突都撤销此前通过结果。握手/期望实例的具体提供方式仍需与启动自检票对齐。应用序列与握手用于避免正常缓存/旧进程混淆，不是抗恶意发布者的认证机制。

## 3. QoS 建议与未决参数

**事实**：Humble 文档中 transient-local 为晚加入订阅者保留样本，只有发布、订阅双方都使用它时才能取得历史数据；volatile 订阅者可连接 transient-local 发布者，但只收新样本。Reliable 是传输策略；deadline 是发布间隔契约，lifespan 是发布到接收的最大年龄；automatic liveliness 可能由节点的其他发布者维持。[Humble QoS 官方文档源码](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)

**建议**：诊断发布者与常规诊断订阅者显式采用 RELIABLE、TRANSIENT_LOCAL、KEEP_LAST depth=1，并周期重发完整快照，便于晚加入的查看器。depth=1 是建议的快照队列设计，不是安全时间阈值。T6 可选择 VOLATILE 接收后续周期报告；即使这样仍须绑定当前实例，不能把刚收到的样本自动认定为目标实例。若 T6 也用 TRANSIENT_LOCAL，则缓存初报仅作身份候选，等待当前实例的序列推进与时效校验。不要只启动时发一次。

**建议**：报告周期 P、接收超时 T、可选 lifespan L 与 deadline D 需结合启动耗时、executor 调度抖动和通信条件验证。deadline/liveliness 事件是辅助故障线索，不取代消费端超时与准入撤销。不得在仅一端设置更严格 QoS 而不检查兼容性；记录实际 RMW 与双方 QoS。

## 4. PyYAML 与 hash 规范化

**事实**：safe_load 限制可构造类型，但不提供本项目 schema 校验。PyYAML 文档列出了标量隐式类型转换；6.0.2 官方构造器中普通映射最终执行 `mapping[key] = value`，后出现的重复 key 会覆盖；SafeConstructor 支持 merge，并把日期变成 date/datetime，yes/no/on/off 等可变成布尔值。[PyYAML 文档](https://pyyaml.org/wiki/PyYAMLDocumentation)、[PyYAML 6.0.2 constructor.py](https://raw.githubusercontent.com/yaml/pyyaml/6.0.2/lib/yaml/constructor.py)

**事实**：Python JSON 默认允许 NaN/Infinity；`allow_nan=False` 才拒绝它们。sort_keys 只排序键，separators 控制空白，ensure_ascii 控制字符转义；非字符串字典键可被转换成字符串。由此不能把一次普通 json.dumps 叫作已完成的跨语言规范化协议。[Python 3.10 JSON](https://docs.python.org/3.10/library/json.html)

**建议**：

1. 文件 SHA-256 始终覆盖实际加载原始 bytes；注释、换行或编码改变时它应变化。record ID 覆盖版本化单源 payload（矩阵和规定 provenance，排除 ID 自身），其输入应由共享代码显式构造，不能对任意加载字典直接 dump。
2. 限制单文档、字符串 key 与显式允许字段；重复 key 拒绝。建议拒绝 YAML merge/alias，保持配置审计直接可读。日期、序列号、frame 与 operator 要求字符串；日期写成有引号的规定格式，未知值用 schema 定义的 null，不猜造日期。不要 `default=str` 静默接受 date、set 或其他对象。
3. 数值接受域显式定义，先排除 bool 再接受整数/浮点，再检查 finite。矩阵 1 与 1.0 是否同 ID、-0.0 是否归为 0.0，须固定；建议矩阵统一 binary64 并将负零归零，不以容差量化掩盖实际矩阵修改。非矩阵 provenance 数值也要固定单位和类型。
4. Python 内部共享实现可使用 `json.dumps(payload, sort_keys=True, separators=(',', ':'), ensure_ascii=False, allow_nan=False).encode('utf-8')`，但前提是已完成上述 schema 规范化。须标注项目算法版本，不宣称它自然等同某跨语言 canonical JSON 标准。文本保持原码点，不进行隐藏 trim/Unicode 归一化；若未来需要归一化，修改版本并提供迁移。
5. 固定测试向量应同时包含：键顺序/注释变化只改变文件 hash；矩阵/provenance 修改改变 record ID；ID 字段本身不参与 ID；1/1.0 与负零；中文字符串；重复 key、隐式日期、非有限值、bool-as-number 拒绝。跨工具、Python/PyYAML 升级必须复跑同一字节与摘要向量。

## 尚未决定或实测的项目

- P/T/L/D、允许时钟偏差、启动等待上限，以及真实节点实例如何交给 T6。
- 数学正交/行列式/底行数值容差、机械平移范围；沿用已有 20 mm/5° 要求不等于已完成其实测验收。
- 调试模式与避障准入的明确接口、是否由另一个健康组件汇总实测证据。
- schema 精确字段、错误码、版本字符串、数字规范化与固定测试向量。
- 本机 colcon 安装到底复制还是链接、部署 overlay、实际 RMW/QoS 与真实 ROS 启动行为。本研究只核验上游接口事实，没有执行 ROS 或硬件实验。
