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
本项目使用的 9-DOF 机械臂：J1 为棱柱关节（升降），J2-J9 为旋转关节。
_Avoid_: 机械臂、robot arm

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
