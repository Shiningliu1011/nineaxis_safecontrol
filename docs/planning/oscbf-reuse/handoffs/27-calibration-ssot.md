# [T7] 标定 SSOT 接线: sensor_extrinsics.yaml 唯一真源 (impl)

- Tracker：https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/27
- 日期：2026-09-09；认领：Shiningliu1011。
- 状态：本地事实核验完成，状态接口方案已获用户接受；尚未实施，不满足关闭条件。
- 起点：`659da6c6db598abddc272ad0941b72f77793211b`。保留既有 MAP.md 修改、handoffs、.scratch 和大然电机资料。

## 队列与范围

用户指令：“[$wayfinder] 继续下一个ticket”。上一窗口 [术语修订交接](20-spec-terminology.md) 指向本票。原生依赖仍被该术语修订票阻塞（其补丁尚未实施），正文“Blocked by: 无”为过时信息；以原生关系为准。本窗口只做不依赖它的核验与规划，不删除依赖，不宣称前置完成。曾按无阻塞前沿短暂认领几何研究票，读取上一窗口交接后已撤销认领，未开展其研究。

沿用 ADR 0003、0004；不重新选择世界坐标系或标定权威。规划、实施、实机准入分别记录。

## 已核验事实

1. `perception_bridge.py:263` 声明静态矩阵 ROS 参数，`:319` 在每次变换时读取。当前无 sensor_extrinsics 文件加载。
2. `_sensor_to_world`（`:293` 起）仍为采集时刻 TF → latest TF → 参数矩阵 → identity。仅更换最后的参数读取不足以建立唯一权威。
3. `_sensor_callback`（`:348`）以配置 input_frame 取变换，未核对收到的 `msg.header.frame_id`；实施必须拒绝错帧，不能把另一传感器的数据套上正确 hash 的矩阵。
4. runtime 配置和 dual_sensor_real 配置分别在第39、29行复制同一假定位姿；sensor_extrinsics 当前是单位阵占位。应迁移现有 runtime 值并删除两个 profile 的复制项，不能直接切换到占位矩阵。
5. 离线 YAML/NumPy 核验：两份 runtime 相机矩阵相同，R 的 det=1、正交残差=0，平移为 [-0.9, 0.4, 1.7] m；这只证明数学合法性，不证明标定可信。两 profile 均未启用 LiDAR。结果与文件 SHA256 留在 ignored `output/audit-27/config-audit.json`。
6. `setup.py:23` 已安装 config/*.yaml；launch `mujoco_transition_final.launch.py:23,198` 从 ament share 读取运行配置。package.xml 已声明 ament_index_python 与 python3-yaml。
7. `/perception/status` 在 bridge `:238,421` 使用 Float32MultiArray，只有10个浮点健康字段，无法直接加入路径/hash/id 字符串。原票要求在该话题上报文件身份，需要消息契约决策。
8. `tests/test_perception_bridge_demo.py` 明示只测纯解码/预处理、不启动节点；不能作为本票实际启动验证证据。

## 实施交接（状态接口已接受，其余为既定要求与工程建议）

- 启动时由 ament share 解析路径并 resolve，一次读取字节，解析、校验和文件 SHA256 都基于同一读取快照；在 FusionEngine、订阅与定时器创建前完成。运行中使用不可变记录，不热重载、不回退源码路径。
- 每源独立 camera/lidar record，保留票据要求的全部 provenance 字段。假定相机标记 calibrated=false、method=assumed、operator=assignment；未知序列号/实测残差/日期不得编造。
- 区分两个 hash：文件 SHA256 对实际加载原始字节；calibration_id 对版本化、规范化的单源矩阵及 provenance 内容计算，排除 calibration_id 本身，避免循环。标定工具与运行加载器共用规范化函数。格式/算法须有固定测试向量。
- 固定臂外传感器的 sensor→base_link 直接来自加载记录；旧 use_tf=true 不可继续绕过该记录。建议明确拒绝该模式；若需要 TF 展示，由同一记录派生发布，外部 TF 不再决定标定矩阵。
- 帧名必须匹配记录和消息 header；非法/缺失记录不转 identity。禁用 LiDAR 不要求它的占位记录具备实测 provenance，启用则必须通过完整门槛。
- “单 Camera 不变”指保留默认假定矩阵的几何/离线行为，不能把 calibrated=false 当作避障准入通过；完整健康锁存交给感知健康/启动自检票。若现有启动模式无法区分调试与避障准入，须先明确该模式接口，不能默认放行。
- 已接受的状态接口：保留旧 `/perception/status` 十字段数组，新增 `/perception/calibration_status`，采用 `diagnostic_msgs/msg/DiagnosticArray` 上报路径、文件 hash、逐源 calibration_id 与检查结果；不优先采用 String/JSON。自检仍须绑定当前节点实例并验证新鲜度。详细键名、发布周期、超时与模式接口在实施时细化并验证。


## 复用结论

保留 FusionEngine 和现有几何/控制算法；复用已声明的 ament_index_python、PyYAML 及 Python 标准库 pathlib/hashlib/json，仅增加本项目的 record schema、加载校验和 ROS 适配。没有新上游选型或 fork，不以本次本地核验宣称第三方版本/许可证审计完成。

## 验收、回退与编码门

验收应覆盖：默认单相机点变换等价；两个 profile 无矩阵复制；source/install 分叉时只读 ament resolved 文件；修改字节/矩阵/provenance 后身份变化符合约定；无文件、非有限值、反射、错误末行、错帧、无标定双源均拒绝；合法 identity 可过数学检查但不跳过 provenance；旧参数和 TF 无法绕过 SSOT；实际 ROS 启动日志与状态上报一致；自检不能用源码文件或历史节点消息误报通过。记录机械平移范围/数值阈值与真实20mm验收的来源，未知项不臆造。

当前可以准备独立 loader/schema 的实现，但整票尚不能无条件按完整契约实施或关闭：仍需前置术语票收尾，部署和实际启动验证亦未完成。本窗口没有改产品代码、运行配置或硬件，没有运行运行时测试。

回退以完整代码与配置版本成对恢复并重启节点，禁止运行时改回另一条外参来源。未经标定的旧配置不能因回退获得实机准入。

## 下一步

状态接口已定，可按此准备 SSOT 加载、配置迁移与诊断发布实施；前置与验收缺口继续保留。记录阶段决议，不关闭实施 issue、不追加地图已完成票索引。之后进入标定工具链窗口时仍须注明本票的实现依赖；本窗口不自动启动下一票。

## 补充研究与用户决定（2026-09-09）

用户要求使用 research 回答状态接口问题。见[标定文件身份的状态接口研究](../research/calibration-status-interface.md)。研究推荐继续保留十字段健康话题，新增 `/perception/calibration_status`，以 `diagnostic_msgs/msg/DiagnosticArray` 直接携带身份与诊断状态；这项建议取代上文优先用 String/JSON 的提案。ROS 官方消息已有字符串键值和诊断等级，本机已安装，但项目实施时仍需显式声明依赖。用户随后原文回复：“可以接受”。据此接受保留健康接口、新增标准诊断通道的方案，已同步 ADR 0004 与感知规格；未将其扩大为自动实施或实机运行授权。


## 续研：当前接线契约与验收缺口（2026-09-11）

用户指令“开始研究ticket#27”；本轮为研究，不扩展到产品实施。工作区 HEAD 为 `5b23222d49c44d95d5220c811b275d1f1ba9dbeb`，存在其他窗口未提交材料，原样保留。GitHub 原生前置「[T0] spec 与参数文件坐标系/术语修订 (impl)」现已 CLOSED；本票仍 OPEN，已确认认领 Shiningliu1011。上文“前置未完成”仅为历史状态，不再是当前阻塞。

### 当前事实（源码与只读配置核验）

- `perception_bridge.py:123–180` 先构造 FusionEngine，尚无 SSOT loader；`:293–322` 仍为 TF→latest TF→参数→identity；`:328–364` 未检查消息 frame 与配置 frame 是否一致。既有缺口尚未修复。
- 两 profile 的相机矩阵仍相同，平移 `[-0.9, 0.4, 1.7] m`，旋转 det=1、正交最大残差=0；SSOT 的 camera/lidar 仍为 identity。两个 profile 的 `source_topic_lidar` 均为空。数学合法性不等于实测可信。
- 在本机 source ROS Humble 与 install/setup.bash 后，ament 查询得到 `install/robot_safecontrol_moveit/share/robot_safecontrol_moveit/config/sensor_extrinsics.yaml`，`resolve(strict=True)` 实际指向源码 `config/sensor_extrinsics.yaml`。本机是软链接安装，当前不是两个独立物理副本。实际 SHA256 为 `b8327f546f4febdfa37ed06b644599c9513fa65eed427a500846ac0522dfedd4`。仍必须经 ament 查询，不允许硬编码源码 fallback；复制安装分叉是必须覆盖的另一部署情形。
- runtime YAML SHA256 为 `4dfe85f18c38d223a395d1b389fc539c5094d02f5aa0bd7e890e569117e0feba`；dual profile 为 `7346cf48806305bc336251446a748d448e27c68ae4327eba6df5ff2978e8acaf`。
- `package.xml` 已声明 ament_index_python、python3-yaml，尚未声明 diagnostic_msgs。
- launch 的 `start_perception` 同时启动 bridge 并启用控制器感知障碍；bridge 当前没有区分“假定外参调试”与“可信外参避障”的模式。控制器订阅 tracks，单纯新增诊断不能充当其准入门。
- [共架布局](../../../sensor_layout_and_calibration.md)是安装初始方案，不是实测外参；[纸箱验证](../research/targetless-box-validation03-20260911.md)评估的是传感器相对配准候选，不能证明 sensor→base_link 标定或全工作区20mm验收已完成。两者均不能自动替换默认矩阵或标记 calibrated=true。

### 建议实施分解（未实施）

1. 增加纯 Python 的 schema/loader/record ID 入口，供 bridge 与标定 exporter 共用。记录按 camera/lidar 分开，固定 `T_base_sensor`、米、明确 frame；读一次 bytes，同时计算文件 SHA256、解析和校验，返回不可变快照。记录 ID 需带 schema/算法版本；拒绝重复 YAML key、非有限数值、bool 冒充数字、未知或缺失必要字段。未知实测字段写 null，不能编造。
2. bridge 在 FusionEngine/订阅/定时器前完成加载。变换只来自快照；废除旧矩阵参数影响并明确报错旧覆盖配置，拒绝 use_tf 绕过。先校验 msg.header.frame_id 再消费点云；运行期错帧丢弃，不回退其他矩阵。固定帧同名也不能绕过 provenance。
3. 将旧相机矩阵迁移至唯一记录，method=assumed、operator=assignment、calibrated=false；移除两 profile 的重复矩阵。保留 LiDAR 禁用状态。预览 JSON 与离线候选不是生产写入入口。
4. 沿用已接受的 DiagnosticArray 通道：文件路径/hash、逐源 ID、校验结果、节点实例、报告序号；路径同时记录 ament lookup 与最终 resolved 值以便解释 symlink。失效退出时日志必须明确；启动失败不能只依赖尚未发送的诊断。状态消息仅报告加载快照，不能重新读文件后报告成当前内存身份。
5. 标定 exporter 先离线导出候选，部署/重启后对照节点快照。软链接写入也不代表运行中快照更新；复制安装需重新部署。保留旧代码和配置的配对恢复记录。

### 仍需展示给用户的模式取舍

推荐将未标定调试与避障准入显式区分：调试可复现旧单相机点变换，但未通过标定门的数据不能成为控制器的可信避障输入；避障模式对启用源完整 fail-fast。这将影响 launch、控制器/健康票的接口边界，不能仅给诊断打 WARN 就称为安全隔离。若选择所有启动一律要求完整标定，则当前假定单相机默认启动会被拒绝，无法满足原票兼容要求。推荐前者，但模式名称、默认值、隔离实现及跨票分工尚待审阅，不记录为已接受决定。

SE(3) 数值容差属于软件校验参数；支架平移包络、实测误差准入和报告超时属于不同证据层级，不用任意常量冒充已验证安全阈值。几何 ROI 也不能当成传感器安装平移范围。

### 验收矩阵

| 层次 | 必须证明 |
|---|---|
| 纯加载 | 缺文件、畸形结构、重复键、NaN/Inf、反射、末行错误拒绝；合法 identity 不误拒绝；启用源与禁用源规则清楚 |
| 身份 | 固定测试向量；仅注释变化改文件 hash、不改 record ID；矩阵/provenance 变化改相应 ID；伪造或陈旧 ID 拒绝 |
| 几何/兼容 | 默认单相机代表点与旧变换等价；错帧拒绝；TF/旧参数无法改写快照 |
| 部署 | ament overlay、symlink、复制安装分叉；节点启动后文件变化不改变内存快照或冒报新身份 |
| ROS 集成 | 仅启动隔离 bridge 与合成输入，观察实际诊断；晚订阅、重启、旧实例、暂停/回跳时钟及启动失败；不以纯函数测试替代 |
| 准入 | 未标定调试不能被控制器当作可信避障；完整标定门与健康票衔接，实测证据另行验收 |

本轮只做源码/配置/ament 路径只读核验，未启动 ROS 节点、未运行机器人、未改产品代码或配置。测试方案不是测试通过报告。本票不满足 Done when，保持开放；研究阶段进度不进入地图已完成索引。

外部接口的一手来源及详细建议见[运行时身份、诊断与规范化补充研究](../research/calibration-ssot-runtime-contract-20260911.md)。其中“本机安装方式未核验”指研究子任务边界，本窗口主核验已确认上述 ament 软链接事实。
