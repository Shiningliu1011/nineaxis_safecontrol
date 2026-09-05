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
被控对象与查看器发布、控制器订阅的关节状态话题流（BEST_EFFORT、深度 20），约定统一在共享模块中。
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
订阅命令流（/oscbf_command）、经安全网关校验后换算为 DrEmpower CAN 帧发送给电机，并把反馈帧解码换算后发布状态流（/mujoco_joint_states）的 ROS 2 节点。
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
