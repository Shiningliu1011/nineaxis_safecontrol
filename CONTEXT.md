# robot_safecontrol

9-DOF 冗余机械臂安全控制项目的领域词汇表：MuJoCo 物理仿真中，机械臂从任意位姿经无碰撞过渡抵达蝴蝶形参考轨迹起点，由 OSCBF 安全控制器跟踪至终点。

## 系统结构

**过渡管线**:
从随机工作位姿规划并执行到参考轨迹起点的无碰撞过渡，回放结束后把控制权交给安全控制器的整条流程。
_Avoid_: transition pipeline、过渡流程

**过渡执行器**:
执行过渡管线的纯逻辑模块——状态机、相位转移与失败诊断都在其实现内，不依赖 ROS，节点与命令行入口只是它的适配器。
_Avoid_: 过渡状态机、pipeline orchestrator

**交接**:
过渡回放结束、安全控制器接管命令流的时刻。交接前后圆柱拟合与轨迹变换口径必须一致，否则姿态会差 180°。
_Avoid_: handoff moment、接管点

**状态流**:
控制器及显示/评估消费者读取的关节状态话题流。`run_demo` 仿真闭环中由被控对象持续发布；默认最终 launch 未启用被控对象时，过渡回放可向该状态话题发布。viewer 只订阅状态，不拥有状态 publisher。具体实体和 QoS 见 [ONBOARDING](docs/ONBOARDING.md)，共享约定见 `ros_conventions.py`。
_Avoid_: joint-state topic、传感器流

**命令流**:
控制器发布、被控对象订阅的安全命令话题流，与状态流分离。
_Avoid_: command topic、控制流

**控制内核**:
纯 JAX 的 OSCBF 安全控制计算核心（OSC + CBF + QP + 积分），无 ROS 依赖，节点只能经其 facade 访问。
_Avoid_: portable_oscbf 库、内核包

**被控对象**:
带加速度与 jerk 限幅的执行器仿真，积分安全命令并持续发布关节状态，模拟真实编码器行为。
_Avoid_: plant 节点、执行器桥

**AEB-RRT***:
自研九轴关节空间全局规划器，使用碰撞安全模块验证状态与区间，负责为 JAX trajectory
optimization 提供已经证明无碰撞的连通路径。
_Avoid_: MoveIt OMPL 插件、单独的规划碰撞检查器

**JAX trajectory optimization**:
以 AEB-RRT* 路径为初始值，联合优化过渡段与完整任务名义轨迹的长度、平滑程度、时间与
安全余量，并将结果重新交给碰撞区间证明。
_Avoid_: 单独使用局部优化器、轨迹平滑后直接执行

**任务名义轨迹**:
覆盖完整任务参考路径的带时间九轴关节轨迹；它保持末端位置与工具轴要求，为 OSCBF 提供
任务和零空间参考，不具有命令授权能力。
_Avoid_: 只到任务起点的过渡轨迹、最终安全命令

**IK 目标集合**:
JAX multi-start IK 为任务起点生成的多组不同冗余关节构型；每组都通过关节、末端任务与
当前场景碰撞验证，可作为多目标 AEB-RRT* 的终点。
_Avoid_: 单个 IK 解、未经碰撞验证的 IK 候选

## 机器人

**ninezzhou**:
本项目使用的 9-DOF 串联冗余机械臂，运动学拓扑 1P8R：第 1 轴为固定工作台上的直线
导轨/丝杠移动模组（移动关节，位移 d1，滑台沿导轨平移；base_link 固定于工作环境，
直线轴运动显式表示为关节变量 q1，而非基座坐标系运动）；滑台之上的 8 个旋转关节
（J2–J9）组成主臂与腕部。结构分层：直线移动基座 + 主臂串联关节 + 腕部关节组 +
末端执行器接口。
**base_link（Y-up）**: 用户规定的机架基坐标系，沿用仓库 legacy Y-up 约定——+Y 竖直
向上（J2 肩部 origin (0, 0.343, 0)、轨迹偏移/工作空间/相机假定姿态均以此为准），
+Z 为 J1 直线导轨方向（水平），+X 横向；J1 沿该系 Z 轴滑动、base_link 本身固定。
感知/控制器/MoveIt/POE FK 全链路共用此系；MuJoCo viewer 经 display_frame 旋
转（Y-up→Z-up）仅用于显示。
_Avoid_: 机械臂、robot arm

**世界坐标系（base_link 固定环境系）**: 05B 定案——本项目的「世界坐标系」就是
base_link 本身：基座固定且 J1 直线导轨在链内显式建模，base_link 不随任何关节运动，
等价于任何固定环境系；感知融合、控制器、MoveIt、POE FK 全链路以此为唯一规范系，
不再另设独立固定世界系。相机/激光雷达均为臂外固定安装，其静态外参
（sensor_frame → base_link）是常量。
_Avoid_: world_frame（独立 env 帧）、map、odom

**标定 provenance**: 每个传感器静态外参的来源记录——方法（如 FAST-Calib2 /
AX=YB）、日期、操作者、残差与误差估计。外参进入 `config/sensor_extrinsics.yaml`
时必须携带 provenance（含 `calibrated`、`sensor_serial`、`calibration_id`、
`timestamp`、`operator`、`residual`、`error_estimate`、`frame_from`、
`frame_to`）；该文件是**唯一 calibration authority**（ADR 0004，标定工具写它、
bridge/launch 读它、T6 校验「被校验 record == runtime 加载 record（ID/hash）」），
`perception_runtime.yaml` 不持有标定矩阵（T7 落地后）。启动期自检分两组——
数学合法性（元素有限/R^T R ≈ I/det≈1/bottom row/translation 机械范围，
**非单位阵不是判据**——identity 可以是合法外参）与标定状态（provenance 完整 +
与记录比对 ≤20mm/5° + 已知物体验收）——任一不通过则拒绝进入避障模式。
_Avoid_: 外参来源、calibration record

## 轨迹与几何

**工具轴角误差**:
当前工具 X 轴与参考工具 X 轴之间 0–π 的真实夹角，忽略绕工具轴的 roll；验收不再用 sin(θ) 代替夹角。
_Avoid_: 把控制内核的二维叉积误差直接标为真实角度

**声明验收区间**:
实验采样前固定的路径起止弧长；允许子区间单独验收，其通过不代表整条路径已验证。
_Avoid_: 事后截取的稳定片段（作为预先声明区间的同义词）

**路径覆盖**:
本次样本从初始投影弧长推进到最终投影弧长所覆盖的声明区间；绝对弧长位置到达 100% 不独立证明从起点走完或任务验收通过。
_Avoid_: source_time 比例、参考端点事件（作为路径覆盖的同义词）

**冗余避障**:
利用九轴协调运动调整臂身以避开外部障碍，同时保持末端沿原路径的位置与工具轴精度；必要时降低进给，无法兼顾时暂停任务并进入安全处理路径。
_Avoid_: 关节回中（作为冗余避障的同义词）、末端绕路（指冗余避障时）

**蝴蝶轨迹**:
末端执行器要跟踪的蝴蝶形参考轨迹，由 NURBS 数据（ik_input.mat）给出。
_Avoid_: reference path、NURBS 曲线

**轨迹变换**:
把仓库参考轨迹标定到 MuJoCo 场景的变换；mat 加载、单位换算与圆柱投影都收敛在唯一入口内。
_Avoid_: calibration transform、坐标对齐

**圆柱拟合**:
对参考轨迹拟合圆柱，得到轴方向与轴心，用于生成表面法向姿态参考；拟合口径必须在轨迹、过渡、控制三端一致。
_Avoid_: cylinder fitting、轴线拟合

## 真机部署

**真机执行端**:
目标职责是订阅命令流，经安全网关和硬件传输发送真实执行命令，并将真实反馈转换成状态流。当前 `hardware_bridge` 仅提供 fail-closed containment：sim 不创建硬件控制 I/O；shadow 只记录请求/拒绝，不收发 CAN、不发布真实硬件状态；live 禁用。真实 SocketCAN、反馈 freshness/watchdog 和真机准入见 [GitHub #13](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13)。
_Avoid_: hardware bridge、CAN bridge

**安全网关**:
命令流的唯一卡口——超时/故障/限幅违例时输出零速保持命令并锁存停车原因，直到外部健康确认后人工恢复；网关只检验不重塑。
_Avoid_: safety gate、command validator

**解析障碍物**:
点云经聚类后拟合的球/圆柱几何障碍物，以包络球进入控制内核的 obs_* 接口；与稠密点云/ESDF 不同，它是可验证的几何单元。
_Avoid_: obstacle shape、fitted obstacle

**零位标定表**:
记录每个关节的方向符号与电机零位偏移的配置文件（hardware_joint_zero.yaml），由机械零位标记法生成。
_Avoid_: zero calibration、home offset

**DrEmpower 帧**:
电机 CAN 通信的基本单元，CAN ID = (node_id << 5) | cmd_byte，位置命令 0x19、系统命令 0x08。
_Avoid_: CAN frame、motor command


## 障碍观测

**即时障碍观测**:
本项目中，在给定采集时刻、坐标与有效范围内对环境障碍几何的观测；其存在不依赖对象身份关联或速度估计成功。
_Avoid_: 已跟踪障碍（作为全部在线障碍的统称）

**局部占据场景**:
LiDAR 射线在机器人附近更新的 `occupied`、`free`、`unknown` 空间状态；占据 voxel
生成环境碰撞 support points，自由与未知状态用于覆盖准入。
_Avoid_: 单帧点集、把无返回区域称为自由空间

**占据证据规则**:
LiDAR hit 立即建立 occupied，连续且时间有效的 free 射线才能清除 occupied；陈旧占据
转为 unknown，时间经过本身不能产生 free。
_Avoid_: 最新射线直接覆盖、超时后删除障碍

**运动跟踪层**:
局部占据场景之上的可选运动估计；可信 track 为关联 support points 提供速度与 barrier
时间项，不能创建或删除占据证据。
_Avoid_: 用 track 列表代替占据场景、把未跟踪障碍称为空闲

**Support point 米制范围（rho_m）**:
以 support point 为中心、覆盖 occupied voxel 体积与全部空间误差的球形范围；它按机器人
ellipsoid 的最短半轴转换为 pair-specific `scale_margin`。公开配置与诊断使用 `mm`，
`rho_m` 是 collision facade 转换后的 JAX 内部量。
_Avoid_: 零体积点、全局固定 scale_margin

**保守 support 合并**:
用一个新的 `support_point + rho_m` 完整覆盖一组 occupied support；用于满足固定 JAX
容量，无法证明覆盖时场景进入 `OVER_CAPACITY`。
_Avoid_: 截断点云、删除较远 occupied voxel

**固定环境模型**:
经校验的工作台、夹具、地面等已知环境几何，以版本化 `support_point + rho_mm` 进入
`CollisionScene` 并与 LiDAR occupied 取并集；环境变化后需重新校验。
_Avoid_: 静态跟踪结果（指固定环境模型时）

**必要空间覆盖**:
对当前动作及停车所需空间具有仍有效的环境信息，其范围与可信条件由观测契约确定；无返回点不等于具有覆盖。
_Avoid_: 点云非空（作为覆盖充分的同义词）

**环境距离场**:
环境障碍几何的一种固定形状表示：按基准系的固定栅格给出各位形到最近被观测占据位置的距离，机器人侧的固定查询点据此取得与环境的距离；查询点不在栅格覆盖内时按不安全处理。
_Avoid_: ESDF、静态地图（作为该距离场的同义词）

**自体过滤**:
从 LiDAR 射线中识别机器人自身表面返回的处理；使用 point 采集时间对应的关节状态与
机器人 mesh 预计首次交点，机器人侧的组合 ellipsoid 不参与删除环境点。
_Avoid_: 机器人自滤、机械臂点剔除

**歧义观测（ambiguous observation）**:
受测量、标定或时间误差影响，无法明确判定为 robot self 或 environment occupied 的
LiDAR 返回；对应空间不能取得覆盖准入。
_Avoid_: 直接删除、直接标记为 free

## 标定记录与准入

**标定记录**:
单个传感器静态外参及其 provenance 组成的一条记录，按源存放在
`config/sensor_extrinsics.yaml` 的对应分节中（ADR 0009）。
_Avoid_: 外参来源、calibration entry

**记录身份（calibration_id）**:
一条标定记录**内容**的稳定标识：标定内容改变时它改变，文件其它部分（注释、别的源、
无关配置块）改变时它不变；用于比对「被校验的记录」与「运行时实际加载的记录」。
_Avoid_: file hash、文件版本号

**文件身份（file_sha256）**:
节点实际加载的整个文件字节的摘要，证明读到的是被检查的那份文件；它不刻画单源标定
内容是否变过。
_Avoid_: 用 calibration_id 指代文件身份

**未标定调试身份**:
某源以 `calibrated: false` 的假定几何被启用时的身份。仅当部署 profile 明确逐源允许时
才允许启动，且不构成标定准入。
_Avoid_: 未标定模式、debug mode

**标定准入**:
某个源的标定记录可否作为控制器**可信障碍输入**的判据，四个条件同时成立：矩阵数学合法、
`calibrated: true`、provenance 完整、记录身份自洽（声明的 `calibration_id` 与内容算出的
一致）。部署 profile 的逐源豁免只决定「能否启动」，不改变本判据的结论。
_Avoid_: 准入检查（那是求解后对松弛额度的检查）、admission、calibration gate

## 安全约束与准入

**碰撞安全模块**:
规划、轨迹执行与 OSCBF 在各自进程内共同加载的无 ROS JAX 碰撞判定来源；它持有统一
机器人碰撞几何与 `CollisionScene`，并返回尺度、屏障值、关节梯度、危险对象身份和查询状态。
_Avoid_: 把规划碰撞检查、轨迹复查和 OSCBF 碰撞约束称为三套独立检测器

**PreparedScene**:
`CollisionSafety.prepare_scene` 完成 identity、容量、时间、覆盖与几何检查后返回的不可变
场景；只有 PreparedScene 可以进入状态查询与区间证明。
_Avoid_: 原始点云、未经验证的 CollisionScene

**PreparedScene 发布**:
感知工作线程完成整个不可变场景后，通过原子引用一次替换控制线程可见版本；控制线程不能
读取构造中的场景，也不能等待场景构造。
跨进程发布使用固定布局的双缓冲共享内存；ROS 只传递 revision、时间、布局身份与健康状态。
读取进程每个 revision 检查 identity 与 checksum，并只向本地 JAX device buffer 复制一次。
_Avoid_: 原地修改 scene、控制线程处理 LiDAR、用 ROS 传递完整 PreparedScene

**碰撞场景服务（collision_scene_server）**:
CollisionScene、`scene_epoch`、`scene_revision` 与共享内存 slot 的唯一写入进程；负责 mesh
self-filter、射线占据、固定环境合并、tracking、support 生成和场景验证。控制器、规划器与
轨迹验证器只读取它发布的不可变场景。
_Avoid_: 多个进程各自构造环境场景、控制器拥有占据状态

**碰撞计算精度**:
LiDAR 原始坐标可以使用 `float32` 传入；`prepare_scene` 将参与碰撞计算的数据统一转换为
`float64`。DCOL、point-scale、barrier、梯度、区间证明、距离查询与 QP 输入均保持
`float64`，该策略属于 `kernel_version`。
_Avoid_: Module 内部混合精度、运行期间降低精度

**组合 ellipsoid 包络**:
每个机器人 link 由固定容量槽位中的一个或多个 ellipsoid 共同覆盖；`active_mask`
标识有效槽位，完整包络具有稳定的 `geometry_hash`。
_Avoid_: 单 ellipsoid link、球体集合、OBB 包络

**Collision geometry artifact**:
由 closed collision mesh 自动生成并通过四面体覆盖证明与运行时间验证的组合 ellipsoid
数据，包含 mesh hash、固定槽位、覆盖证明和 `geometry_hash`。
_Avoid_: 手工参数列表、未经证明的拟合结果

**精确 ellipsoid DCOL**:
直接使用两个 ellipsoid scaling functions 求最小线性 `alpha_star` 的原生 JAX 查询；结果
包含梯度、残差、迭代次数和求解状态。
_Avoid_: 42 面 polytope 近似、米制 OBB distance

**允许接触列表**:
碰撞安全模块中唯一允许跳过检查的 link pair 集合；每项都记录允许原因、适用几何身份
和验证依据，其余完整 link pair 默认进入自碰撞查询。
_Avoid_: 固定 14 pair、隐式忽略列表

**自碰撞 primitive 行**:
一个有效 ellipsoid pair 对应的一条独立 DCOL barrier 约束；link pair 最小值只用于
诊断展示，不替代各 primitive 的梯度与安全判定。
_Avoid_: link pair 聚合行、全身最小碰撞行

**环境活动约束集合**:
每个 robot ellipsoid 当前最危险且必须进入 OSCBF 的固定容量 support pair 集合；每个
pair 保留独立 barrier 行，容量不足时场景进入 `CONSTRAINT_OVERFLOW`。
_Avoid_: 每个 robot ellipsoid 一个聚合行、无证明省略近场 point

**碰撞不可松弛行**:
命令 QP 中不能使用 slack 放宽的 self collision 或 environment collision CBF 行；任何
冲突、求解失败或无效梯度都不能产生被准入的运动命令。
_Avoid_: relaxed collision row、用诊断 slack 授权命令

**碰撞区间证明**:
对关节运动区间计算 `proximity_scale` 下界的验证结果；无法直接证明时递归二分，计算
预算耗尽仍未获得证明的区间不能用于规划或轨迹执行。
_Avoid_: 端点安全、固定步长采样

**规划场景准入**:
规划结果在最新 `CollisionScene` 上通过整条路径复查并取得 `admitted_revision` 的状态；
只有已准入路径才能进入轨迹执行。
_Avoid_: 规划完成、旧 revision 路径

**重新规划触发**:
剩余名义轨迹出现未通过区间、覆盖不足、动态预测冲突或 `remaining_margin_mm` 低于阈值时
产生的重新规划请求；无关 revision 更新不触发重新规划。
_Avoid_: 每次 LiDAR 更新都重新规划、只在碰撞后重新规划

**碰撞临时等待**:
revision 更新、前视验证或重新规划尚未完成时的保持状态；取得当前场景准入后可以自动继续。
_Avoid_: collision fault、人工恢复状态

**碰撞故障锁存**:
标定、时间、覆盖、容量、碰撞 solver 或命令 QP 已失去安全依据后的保持状态；条件恢复、
当前状态与轨迹重新验证并收到人工确认后才能继续。
_Avoid_: revision pending、条件恢复后自动继续

**受限恢复模式**:
安全间距已经越界、几何仍然分离且全部碰撞依据有效时，经过人工确认执行的低速恢复轨迹；
每个证明区间都让全部已违反 barrier 严格单调增加，并保持其余碰撞约束。
_Avoid_: 物理接触恢复、unknown 状态运动、自动恢复原任务

**碰撞场景（CollisionScene）**:
由 LiDAR 射线更新形成的带版本局部占据快照，包含 `scene_revision`、采集时间、
`occupied/free/unknown`、覆盖状态、误差范围、有效槽位和几何身份。
_Avoid_: 点云数组、障碍物列表

**碰撞内核身份（kernel_version）**:
碰撞查询算法、数值设置和接口版本的稳定身份；与 `geometry_hash`、`scene_revision`
共同证明不同运行进程使用相同算法、机器人几何和环境输入。
_Avoid_: ROS 节点版本、程序启动时间

**邻近缩放量（proximity_scale）**:
当前几何达到接触所需的无量纲线性缩放量；大于一表示分离，等于一表示接触，
小于一表示相交。自碰撞由 ellipsoid–ellipsoid DCOL 给出，环境碰撞由
ellipsoid–point scale 给出。
_Avoid_: distance、clearance、CBF alpha

**缩放安全阈值（scale_margin）**:
邻近缩放量必须达到的下界，数值不直接表示米制距离；传感器误差、体素范围和时间误差
需要通过经过验证的几何范围进入查询。
_Avoid_: d_safe、cbf_gain、metric_inflation_m

**碰撞间距（clearance_mm）**:
机器人碰撞包络与外部 occupied 几何包络之间的保守欧氏间距，公开配置、查询与诊断统一
使用毫米；JAX 内部在 facade 边界转换为米。
_Avoid_: proximity_scale、原始 LiDAR range、无单位 distance

**外部距离查询**:
返回机器人组合 ellipsoid 与外部 occupied 保守包络之间的最小有符号 `distance_mm`、要求
间距、剩余余量、最近 pair 和场景身份；用于规划代价与诊断，不授权 OSCBF 安全状态。
_Avoid_: 用 point-scale 直接标为毫米、用距离查询替代 environment barrier

**动态 support 安全模型**:
已跟踪 occupied support 的名义速度进入 point-scale barrier 的 `partial_h_partial_t`，运动估计
误差、加速度范围与延迟进入 `beta` 及 `beta_dot`。规划与区间证明使用 time-indexed reachable
support tube；tracking 失效后改用未跟踪运动上界。
_Avoid_: 用 tracking 删除 occupancy、只按常速度外推未来位置

**碰撞 CBF 响应率（cbf_rate）**:
一阶 CBF 条件中乘以 barrier 的固定时间响应参数，单位为 `s^-1`。自碰撞与环境碰撞分别使用
`self_collision_cbf_rate_s_inv` 和 `environment_cbf_rate_s_inv`；动态障碍物运动通过
`partial_h_partial_t` 表达。
_Avoid_: CBF alpha、运行期间按 pair 改变响应率、DCOL proximity_scale

**碰撞查询状态**:
`query()` 与 `certify()` typed result 的公共枚举状态；只有 `OK` 结果及其有效 mask 可以进入
命令 QP、规划边接受或轨迹准入。其余状态的数值只用于诊断。
_Avoid_: 用 NaN、inf、负距离或全零梯度表示失败

**碰撞切换证据门**:
新 CollisionSafety Module 取得命令权限前必须通过离线回放、MuJoCo、shadow、独立几何参考、
覆盖证明、区间证明、故障注入与 deadline 标准。通过后新 Module 成为唯一判定来源，旧运行
路径与 fallback 被移除。
_Avoid_: 把现有 OBB 或球体输出当作正确性参考、生产双实现切换

**碰撞验收证据 manifest**:
记录几何 kernel、mesh 覆盖、真实 LiDAR 回放、规划与 OSCBF、故障注入、目标设备运行时间和
独立验收数据结果的版本化文件；绑定代码、配置、geometry、kernel、数据与设备 identity。
_Avoid_: 只记录测试是否通过、验收期间修改阈值

**碰撞参数 artifact（CollisionParameterArtifact）**:
由离线测量与验证生成的不可变参数集合，包含误差范围、延迟、间距、CBF 响应率、固定容量、
solver 限制与 deadline；通过 `parameter_hash` 参与 `collision_policy_hash`。
_Avoid_: 运行期间参数覆盖、在线调整安全范围、无数据来源的常量

**安全裕度**:
每条安全约束当前还剩多少余量，是屏障函数的自变量；关节限位行以 rad 计，统一碰撞
模块的自碰撞与环境行使用各自经过论证的无量纲 barrier。
_Avoid_: 用 d_safe 泛指裕度；把裕度为负当作故障

**安全阈值**:
把安全裕度与「够不够」相比的判据，按约束类各自定义——关节限位、自碰撞、障碍三者的物理含义不同，不互为同义词。
_Avoid_: margin、d_safe（作为三类阈值的统称）

**可行 / 不可行**:
可行指存在至少一组关节速度同时满足全部安全约束与执行器速度上限，不可行指不存在；安全裕度为负不等于不可行。
_Avoid_: 把「裕度为负」「求解器未收敛」当作不可行

**弹性松弛**:
弹性 QP 为每条安全约束引入的非负变量，加在该约束的上界上使问题恒有解；其量纲是约束变化率，除以该类增益才换回裕度单位。花掉松弛即放弃安全要求，不是数值误差。
碰撞安全模块的命令 QP 禁止对碰撞行使用弹性松弛。
_Avoid_: 把松弛当作长度、用碰撞 slack 授权命令

**准入检查**:
求解之后把该 tick 放弃的安全要求量与该类约束的允许额度相比，超限则该命令不被采信，进入冻结/锁存路径。
_Avoid_: 健康门（现状只检查解有限与求解器收敛）
