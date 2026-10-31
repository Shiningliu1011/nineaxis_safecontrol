# 模块实现与入口证据

检查日期：2026-10-31。工作区 HEAD：`a95831bd6a452364e224eb4bcbbc02b1c8d3ec43`。

本次使用当前源码、配置、测试定义、已确认规格和历史会话原文，追踪静态调用关系。没有启动仿真、连接设备或运行行为测试。涉及控制通过条件的结论描述源码逻辑；实际违反量、发生频率和运行耗时未在本次测量。

工作开始时已有用户修改：`docs/README.md`、`docs/planning/nineaxis-full-project/README.md`、`docs/specs/dcol-collision-core.md`、`docs/planning/nineaxis-full-project/collision-core-prototype-report.md`。本任务保留这些内容，记录范围仅为当前任务目录。

## 当前决定与资料来源

- 当前用户明确要求机器人包络采用已有 OBB，并指出椭球表示仍保留在代码中。
- `trellis mem context 01a180e0-8176-7270-8630-c66c5882d9e4 --grep '包围盒' --turns 3 --around 1 --max-chars 15500` 恢复第 101 轮用户原文：“你这个包络不对，我之前应该是用OBB包围盒包络过，舍弃掉这一版”。第 103 轮答复记录 10 个 OBB、排除相邻后 36 对。
- [最新碰撞规格](../../../../docs/specs/dcol-collision-core.md)规定 DCOL、几何外扩、自碰撞 10 mm、环境 30 mm、碰撞行不可松弛、PCL/OctoMap、双传感器及公共接口；该文件明确实施暂缓。
- [总体规格](../../../../docs/specs/predictive-coordination-control.md)及 `decision-136` 至 `decision-151` 的相关主题已经确认五维任务、共同模型与限制、三种正式比较方法、执行结算和 Runtime。
- [ADR 0011](../../../../docs/adr/0011-fixed-capacity-compound-ellipsoid-geometry.md)仍标记 `status: accepted`，内容要求组合 ellipsoid。[ADR 0046](../../../../docs/adr/0046-collision-refactor-design-baseline.md)继续将 ADR 0010–0045 视为目标依据。文档有效性需要随最终决定更新。

## 模块检查覆盖

| 模块 | 当前使用的实现与入口 | 需要跟踪的事项 |
|---|---|---|
| 机器人模型与运动学 | NumPy/JAX POE 共享生成数据；MoveIt/KDL 与 PSO 分别服务不同入口；Python benchmark 有独立近似模型 | 共同模型身份接入、五维 IK 目标集合、近似模型实验用途 |
| 传感器接入 | 普通 bridge 双源处理；scene forwarding 仅允许 LiDAR | 双源统一接入及实际来源记录 |
| 环境场景 | FusionEngine/ESDF/tracks 与 LidarOccupancyScene/support 各自运行 | 已确认 PCL/OctoMap 与完整占据凸盒尚未接入 |
| 碰撞安全 | 默认控制 OBB、独立 CollisionSafety ellipsoid、感知演示采样球/ESDF、静态实验 FCL | 统一几何、对象对、量纲、查询和连续检查 |
| 作业参考 | 默认控制已使用连续 NURBS；感知演示使用样本参考；过渡目标和 viewer 从 MAT 取得样本 | 消费来源与身份；默认控制轴向与位置进度不同 |
| 规划 | 默认 C++ AEB-RRT* MoveIt 插件；Python benchmark 独立 | 共享碰撞接入、JAX 轨迹优化及 benchmark 保留范围 |
| OSCBF | 默认 JaxControlLoop；历史 StaticControl 的联合/分离/SNS；感知演示六维 | 碰撞行松弛、五维参考、正式三方法接入 |
| 执行流程 | TransitionExecutor/TrackingRun；历史静态恢复逻辑 | 已确认十二阶段 Runtime 尚未形成共同入口 |
| 安全检查 | TrackingRun 的求解检查；HardwareBridge 内 CommandSafetyGate；静态完整运动检查 | 检查路径与命令入口对应关系 |
| 执行器与反馈 | 位置目标 Ruckig plant；ScheduledMotion 的静态 plant；单关节设备实验 | 共同限值、最终运动身份、反馈和停止证据 |
| 显示 | viewer 订阅状态并执行 `mj_forward` | 上游状态可能来自 plant 或轨迹回放 |
| 记录评价 | 模型控制步报告与静态执行反馈报告 | 正式三方法共同评价尚未接入；现有报告分别声明证据来源 |

## A01：碰撞几何、对象对与数学量

`run_demo.sh:55` → `mujoco_transition_final.launch.py:174` → `oscbf_controller` → `JaxControlLoop`。

- `oscbf_velocity_config.py:264` → `dpax_collision.py:247`：默认自碰撞使用 10 个 OBB 中的 14 对，米制距离减去 30 mm。当前要求为全部 36 对非相邻本体组合、自碰撞 10 mm。
- `oscbf_controller.py:163` → `jax_barrier_terms.py:39`：启用感知后使用机器人 OBB 对球形障碍，固定 8 个 slots；final launch 的 `start_perception` 默认关闭。
- `perception_demo.py:166`、`oscbf_velocity_config.py:328`：ESDF 路径使用 32 个机器人球体；球体来自 `obb_collision_model.py:96` 的 OBB 分格。
- `collision_scene_sources.py:158` → `collision_safety.py:208` → `_collision_geometry.py:43`、`_ellipsoid_dcol.py:19`：独立公共模块使用 ellipsoid 和原生 JAX DCOL。正式控制器尚未消费它。
- `collision_safety.py:332`、`_collision_geometry.py:95`：自碰撞量为 `scale-scale_margin`；环境采用 ellipsoid point-scale 和 support 半径。最新规格要求完整几何外扩后的 `h=alpha-1`。
- `static_experiment_nodes.py:27` → `static_collision.py:7/93/172`：静态实验调用 CPU/C++ `robot_static_fcl`，环境采用 OBB，自碰撞采用 convex mesh。该路径参与实际仿真命令和区间检查。
- `collision_safety.py:457`：公共 `certify` 返回 `CERTIFICATE_FAILED` 和无效 mask；静态实验通过 `static_motion.certify_motion` 使用另一套区间检查。

源码依据：[默认 CBF 配置](../../../../portable_oscbf/work/oscbf_velocity_config.py)、[OBB 查询](../../../../portable_oscbf/work/dpax_collision.py)、[公共碰撞模块](../../../../portable_oscbf/work/collision_safety.py)、[静态碰撞](../../../../portable_oscbf/work/static_collision.py)。

当前入口记录应分别标注默认控制、独立公共库、感知演示和静态实验。公共库内的椭球表示是实际实现；从所查默认控制链无法推导它已经成为 `run_demo.sh` 的碰撞路径。

## A02：感知与场景存在两条完整路径

- `mujoco_transition_final.launch.py:69/211`：默认关闭感知，显式开启后启动普通 `perception_bridge`。
- `perception_bridge.py:355/203`：`scene_server_mode=false` 时创建 `FusionEngine`；球体自身点过滤、静态 ESDF、聚类 tracks 分别由 `fusion_engine.py:251` 等位置处理。
- `perception_bridge.py:754`：普通 bridge 向 MoveIt 发布球体。默认 OSCBF 只订阅 tracks，未消费 SharedSceneReader 或 ESDF。
- `collision_scene_server.launch.py:16` → `perception_bridge.py:367`：独立服务模式要求相机输入为空，只转发 LiDAR。
- `collision_scene_pipeline.py:55` → `lidar_occupancy.py:268`：新路径使用原始 mesh 射线过滤、NumPy 占据网格、体素半对角线形成的 support 包围球。
- `collision_scene_sources.py:116/128` 提供共享 writer/reader 工厂；所查正式规划和控制入口没有使用 reader。

已确认的双源 PCL/OctoMap 和完整占据凸盒属于待实现要求。源码依据：[bridge](../../../../src/robot_safecontrol_moveit/perception_bridge.py)、[场景管线](../../../../src/robot_safecontrol_moveit/collision_scene_pipeline.py)、[占据场景](../../../../portable_oscbf/work/lidar_occupancy.py)。

## A03：默认控制的碰撞行允许松弛

`oscbf_controller.py:431` 创建 `JaxControlLoop` 时未传 `rate_limit_du_max`；`jax_control_facade.py:116/138` 默认关闭 rate limit。`oscbf_velocity_config.py:158` 设置 `relax_cbf=True`、`penalty=1e5`，因此进入 `jax_kernel_factory.py:318` 的 elastic solver。

`jax_kernel_factory.py:348` 的 `qp_ok` 检查有限值与求解收敛；`delta_slack` 和原始约束违反量记录在结果中。`tracking_run.py:221` 按该通过状态交付命令，`oscbf_controller.py:555` 发布。

[decision-143](../../../../docs/planning/nineaxis-full-project/decision-143-joint-qp.md)及最新碰撞规格已要求碰撞行不可松弛。历史 `static_velocity_solver.py:354` 另有原始约束违反即拒绝的实现。这个决定已有来源；后续需要修改实现及验证，无需重新选择是否放宽碰撞行。

源码依据：[内核求解与通过条件](../../../../portable_oscbf/work/jax_kernel_factory.py)、[控制运行](../../../../src/robot_safecontrol_moveit/tracking_run.py)。

## A04：任务维度与参考进度

- `config/oscbf_controller.yaml:38` 已选择 `tool_axis_5d`。
- `production_config.py:62` 的 `shared_path_reference` 默认 `False`，生产 YAML 未覆盖。
- `jax_kernel_factory.py:485` 因而从实际 TCP 投影取得工具轴目标，位置使用独立参考进度；`:515` 的该分支工具轴前馈为零。
- `static_control.py:69` 使用同一 `progress_m` 查询五维移动参考。
- `perception_demo.py:143` 没有指定 `task_mode`，采用 `jax_control_facade.py:121` 的 `pose_6d` 默认；`:270` 直接采用 `q_next` 作为下一状态。

[decision-143:26](../../../../docs/planning/nineaxis-full-project/decision-143-joint-qp.md)已确认位置和工具轴共用同一进度及完整参考导数。六维感知演示仍是可运行的独立入口，其保留用途需要纳入入口决定。

## A05：关节运动限制的数值来源

| 使用范围 | J1 | J2–J6 | J7–J9 |
|---|---|---|---|
| 已确认研究配置 | 500 mm/s；5000 mm/s²；62500 mm/s³ | 120 °/s；120 °/s²；600 °/s³ | 150 °/s；150 °/s²；750 °/s³ |
| 当前 OSCBF 速度与仿真执行器限制 | 500 mm/s；5000 mm/s²；62500 mm/s³ | 168 °/s；约 1799.09 °/s²；约 22488.59 °/s³ | 210 °/s；约 1799.09 °/s²；约 22488.59 °/s³ |
| MoveIt 过渡规划应用默认比例后 | 30 mm/s；40 mm/s² | 约 9.167 °/s；17.189 °/s² | 约 9.167 °/s；17.189 °/s² |

第二行的加速度和 jerk 描述 plant 配置；当前 QP 的执行器 box constraints 直接使用速度上限。第三行是规划层限额，实际回放另有时间缩放。

共同数值位于 [nineaxis_research_limits.yaml](../../../../portable_oscbf/config/nineaxis_research_limits.yaml)，由 `process_limits.py:162` 加载。当前默认链为 `JaxControlLoop` → `NineaxisManipulatorJAX` → `NineaxisKinematics` → `load_actuator_limit_profile`，继续读取 [actuator_modules.yaml](../../../../portable_oscbf/config/actuator_modules.yaml) 的额定速度乘 `0.70` 和转轴加速度 `31.4 rad/s²`。

`oscbf_plant.py:51` 和 `driver_simulator.py:43` 使用 `jerk_time=0.08` 计算 jerk。MoveIt 使用 `models/ninezzhou_moveit_config/config/joint_limits.yaml`，再应用 `transition_planning_server.py:385` 的 `0.2` 比例。

共同模型和限制的模块文档已经标明后续接入范围；本次记录默认入口实际采用的数值。研究限值已有确认，不需要重新询问这些数值。

## A06：FK、IK 与机器人模型

- NumPy POE 读取 `kinematics_data.py`；`nineaxis_manipulator_jax.py:142` 从 NumPy 运动学取得相同螺旋轴、零位矩阵和连杆变换。两种后端共享模型来源。
- `NineaxisModel.load` 已有模型来源、TCP 修正、标定身份和高阶导数；默认控制直接创建 `NineaxisManipulatorJAX`。当前业务消费者包括 `joint_table_cli.py`，共同模型身份尚未贯通全部消费者。
- 默认起点 IK：`task_target.py:114` → `ContinuousIK` → `/compute_ik` → KDL。`kinematics.yaml:4` 选择 KDL 且 `position_only_ik=false`；六个自然姿态初值加当前状态，取一个最佳完整位姿解。`transition_planning_server.py:370` 默认只求起点。
- `perception_demo.py:106` 和 `repro_orient_loop.py:41` 调用 `NineaxisKinematics` 的 PSO；每群 128 粒子与 multi-start 的 128 初值具有不同含义。
- 已确认 [decision-140](../../../../docs/planning/nineaxis-full-project/decision-140-entry-methods.md)要求五维 JAX multi-start、128 初值、最多 32 个有效不同构型。所查工作源码尚无对应生产入口。

源码依据：[运动学](../../../../portable_oscbf/work/nineaxis_kinematics.py)、[MoveIt IK](../../../../src/robot_safecontrol_moveit/continuous_ik.py)、[任务起点](../../../../src/robot_safecontrol_moveit/task_target.py)。

## A07：规划与独立 benchmark

默认入口启动 MoveIt `move_group`，配置 `ompl/AEBRRTstarFaithfulConfigDefault`，由 C++ `aeb_rrtstar_ompl` 执行规划。碰撞使用 PlanningScene/FCL，时间处理采用 `AddTimeOptimalParameterization` 和 `AddRuckigTrajectorySmoothing`。共享 CollisionSafety 的 AEB-RRT* 及 JAX trajectory optimization 在已确认决定中具有后续任务。

另一套 [src/aeb_rrtstar](../../../../src/aeb_rrtstar) 供 `single_run.py` 和 `benchmark_runner.py` 使用。`robot_model.py:149` 采用独立近似 FK，各转动关节按绕 Z 旋转组合；`:17` 复制关节限位，`collision_checker.py:22` 使用稀疏连杆点。其文件明确限定 standalone benchmark。默认 ROS 控制链没有调用该近似模型。

正式三方法的共同模型要求已有决定；近似模型 benchmark 是否继续保留属于实验范围问题。

## A08：作业参考与轨迹重建

默认 `oscbf_controller.py:409/424` 使用 `surface_normal` 并要求连续参考。`ik_data_loader.py:801` 核验连续包来源，`jax_path_following.py:145` 查询连续 NURBS。

`fixed` 模式及 `perception_demo.py:139` 仍可使用样本折线／SLERP。过渡目标和 viewer 经 `oscbf_trajectory.py:119` 从 MAT 读取样本并执行共同变换和圆柱投影。改变参考身份时这些消费者需要共同验证；当前不同读取方式本身不足以判定默认起点错误。

MoveIt 平滑后的过渡由 `trajectory_execution.py:122/190` 进行位置线性重采样，默认回放时间放大三倍且至少六秒，再由执行器根据位置目标产生运动。最终实际运动与规划输出具有各自表示。

## A09：执行、反馈与进度

- 普通 plant：`oscbf_plant.py:181` → `driver_simulator.py:73`，以 Ruckig Position interface 跟踪位置目标。
- 静态 plant：`static_experiment_nodes.py:82` 接收 `ScheduledMotion`；`static_motion.py:136` 使用 Ruckig Velocity interface，保存运动及停止，并检查完整区间。
- 普通跟踪：`tracking_run.py:93` 在求解后立即更新 `_path_state`；`jax_path_following.py:331` 按 `feedrate*dt_path` 推进。
- 静态执行：`static_execution.py:97` 按激活运动结算，参考仍为单周期常值进给；停止后续运动不继续推进参考。
- [decision-141](../../../../docs/planning/nineaxis-full-project/decision-141-progress.md)已确定 `ExecutionReceipt`、`ProgressLedger`、共同变速参考及停止余运动结算。所查 `src` 和 `portable_oscbf/work` 未发现这些共同对象的实现。

## A10：Runtime、安全检查与设备入口

`TrackingRun` 管理 waiting/tracking/holding/closed。终点检查位置、工具轴和保持时间；当前流程未形成已确认十二阶段 Runtime、实际停止与退出的完整判断。

`oscbf_controller.py:501` 向 TrackingRun 提供位置。`CommandSafetyGate` 在 `hardware_bridge.py:86` 构造，shadow 才订阅命令；sim 不建立控制实体，live 明确拒绝。普通仿真命令没有经过这个对象。

独立单关节设备入口由 `motor_cli.py:117` 在显式 `--execute` 后调用 `run_single_joint`，具有自己的反馈和停止处理。其用途与九轴 HardwareBridge 分别记录。

已确认目标见 [decision-151](../../../../docs/planning/nineaxis-full-project/decision-151-runtime.md)。当前检查没有验证任何真实设备停止能力。

## A11：启动、显示与状态来源

`run_demo.sh:55` 显式开启 plant、等待交接，并将回放送到 `/transition_replay_viz`。直接运行 final launch 时，默认关闭 plant、不等待交接，回放话题为 `/mujoco_joint_states`。

`mujoco_viewer_with_cylinder.py:219` 把接收的位置复制到 MuJoCo 并执行 `mj_forward`。显示的状态由上游入口决定，可能来自执行器或轨迹回放。

`run_demo.sh` 加载 `install/setup.bash`。本次查看的安装配置和控制 facade 为指向 build 资源的符号链接；本次证据未通过实际启动验证安装运行环境。

## A12：方法比较与报告

历史静态实验的 A/B/C/D/E 含 separated、joint、SNS，相关文档明确静态实验已取消；代码仍有可运行入口。正式 `pointwise_ik/realtime_coordination/predictive_coordination` 的分工已有确认，所查工作源码没有三个对应正式入口。

普通跟踪报告在 `oscbf_controller.py:127` 明确记录 `kind=model`、`boundary=kernel_candidate`；静态评价读取 plant 数据并标注 `simulated_actuator_feedback`。本次没有发现这两种报告把模型控制步标成真实设备反馈。

## 已确定的要求与入口组织

以下要求已有决定：OBB 目标、完整非相邻对象对、共同碰撞含义及不可松弛条件、双源场景、五维同进度参考、共同模型和限值、三种正式方法、实际执行进度、十二阶段 Runtime。相关接入缺口直接按已有要求记录。

用户已接受 Q01、Q02 建议：正式闭环统一使用 `run_demo.sh`，显式选择三种正式比较方法；感知与过渡演示复用共同模块及配置。移除已被替换且没有独立用途的业务实现、配置、入口和专用测试；独立数值验算和设备检查保留明确用途。

用户补充要求整合共同职责、优先复用，并在一个方法能够满足要求时采用一个方法，使代码框架简洁清晰。对应复用与清理范围见 [决策记录](../design.md)，验证与依赖见 [执行计划](../implement.md)。

本记录提供静态证据，当前源码与默认入口尚未按目标结构修改。
