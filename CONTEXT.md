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
自适应双向 RRT* 运动规划器，作为 MoveIt 2 的 OMPL 插件运行，负责过渡管线的无碰撞路径搜索。
_Avoid_: AEB 规划器、RRT 插件

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

**固定环境模型**:
经校验的工作台、夹具、地面等已知环境几何，是在线障碍观测之外的约束来源；环境变化后需重新校验。
_Avoid_: 静态跟踪结果（指固定环境模型时）

**必要空间覆盖**:
对当前动作及停车所需空间具有仍有效的环境信息，其范围与可信条件由观测契约确定；无返回点不等于具有覆盖。
_Avoid_: 点云非空（作为覆盖充分的同义词）

**环境距离场**:
环境障碍几何的一种固定形状表示：按基准系的固定栅格给出各位形到最近被观测占据位置的距离，机器人侧的固定查询点据此取得与环境的距离；查询点不在栅格覆盖内时按不安全处理。
_Avoid_: ESDF、静态地图（作为该距离场的同义词）

**自体过滤**:
从单帧点云中删除机器人自身表面点的处理；几何取机器人本体的表面几何，按该帧采集时刻的位形计算，机器人侧的保守避障几何不参与剔点。
_Avoid_: 机器人自滤、机械臂点剔除

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

**安全裕度**:
每条安全约束当前还剩多少余量，是屏障函数的自变量；关节限位行以 rad 计，自碰撞与障碍行以 m 计。
_Avoid_: 用 d_safe 泛指裕度；把裕度为负当作故障

**安全阈值**:
把安全裕度与「够不够」相比的判据，按约束类各自定义——关节限位、自碰撞、障碍三者的物理含义不同，不互为同义词。
_Avoid_: margin、d_safe（作为三类阈值的统称）

**可行 / 不可行**:
可行指存在至少一组关节速度同时满足全部安全约束与执行器速度上限，不可行指不存在；安全裕度为负不等于不可行。
_Avoid_: 把「裕度为负」「求解器未收敛」当作不可行

**弹性松弛**:
弹性 QP 为每条安全约束引入的非负变量，加在该约束的上界上使问题恒有解；其量纲是约束变化率，除以该类增益才换回裕度单位。花掉松弛即放弃安全要求，不是数值误差。
_Avoid_: 把松弛当作长度

**准入检查**:
求解之后把该 tick 放弃的安全要求量与该类约束的允许额度相比，超限则该命令不被采信，进入冻结/锁存路径。
_Avoid_: 健康门（现状只检查解有限与求解器收敛）
