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
控制器及显示/评估消费者读取的关节状态话题流。`run_demo` 仿真闭环中由被控对象持续发布；默认最终 launch 未启用被控对象时，过渡回放可向该状态话题发布。viewer 只订阅状态，不拥有状态 publisher。具体实体和 QoS 见 [ONBOARDING](../ONBOARDING.md)，共享约定见 `ros_conventions.py`。
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
