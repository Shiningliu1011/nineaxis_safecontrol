# 完整项目基线与 M01–M12 能力核验

核验日期：2026-10-27。责任任务为[核验完整项目基线与 M01–M12 资产](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/148)，需求基准为[总体项目规格](../../specs/predictive-coordination-control.md)。本页记录当前本机证据；完整工件、画笔、MATLAB 程序和真实样本的来源与使用条件见[资产清单](asset-inventory.md)。

## 代码、环境与资源身份

- 工作区：`/home/lsn/robot/robot_safecontrol`，分支 `main`，HEAD 为 `be963a6bdac16ae48a8284e2a093dc3f41be248d`。
- 核验开始时已有用户改动为 `docs/static_experiment_storage.md`、`docs/static_obstacle_experiments_handoff.md`；核验保留其内容。
- 实际执行环境为 `.scratch/host-recovery/venv/bin/python3`，Python 3.10.12、ROS 2 Humble、Linux `6.8.0-138-generic`。ROS 和安装资源由 `.scratch/host-recovery/env.sh` 加载；JAX 使用 CPU，`enable_x64` 由对应控制代码或测试设置。
- 核验进程采用 `ROS_LOCALHOST_ONLY=1`、`ROS_DOMAIN_ID=186`；`TMPDIR`、`ROS_LOG_DIR`、pytest `--basetemp` 和控制内核输出全部位于 `.scratch/full-project-baseline/`。
- 默认 shell 的 `python3` 指向 Miniconda Python 3.13.12，缺少本项目的 MuJoCo 和 CPython 3.10 FCL 扩展；直接使用该解释器不能复现本页命令。系统 Python 3.10 的依赖也未达到完整运行配置。三种环境分别保存于 `environment-conda.json`、`environment-system.json`、`environment-runtime.json`。
- 实际验证依赖：NumPy 1.26.4、SciPy 1.15.3、JAX/JAXLIB 0.6.2、cbfpy 0.0.1、qpax 0.1.4、OSQP 1.1.3、Ruckig 0.12.2、trimesh 5.1.0、pymoo 0.6.2、jsonschema 4.23.0、MuJoCo 3.16.0、msgpack 1.1.2、python-can 4.6.1、PyYAML 6.0.2、pytest 8.3.5。

[完整 SHA-256 清单](../../../.scratch/full-project-baseline/baseline-sha256.json)固定 208 个源码、测试、模型、配置和已构建二进制文件。原始环境与日志同在 `.scratch/full-project-baseline/`；这些文件受到 `.gitignore` 的本地保留规则保护，跨设备复现时需要连同该目录一起传输。

关键文件的 SHA-256：

| 文件 | SHA-256 |
|---|---|
| `models/ninezzhou/urdf/ninezzhou.urdf` | `766ea5870c41f110d7d57b46e6b31153335f45358faf15e641d465b3b31587a3` |
| `portable_oscbf/config/nineaxis.yaml` | `c9e54de9f3de95e1733a1e4e60de24289eb3363d50d8789d243f0be5c64e6286` |
| `portable_oscbf/config/collision_geometry_artifact.json` | `7ca3a860c7859ecc8aa8c5a1685a4a52d4004a4781697477b8e7c60004a180c6` |
| `portable_oscbf/config/collision_geometry_sources.json` | `7417e54808ddceb370441b193a5ead549271565ddef86a176f456c3cb69802a8` |
| `config/oscbf_controller.yaml` | `5f45e74285921208d8d5904f24232e1cdb87922db85020dc9640abc4930f35ac` |
| `config/oscbf_plant.yaml` | `9579e4d89bb4b19a8d959803d9dbb00827eb526332f55d28cf1ec59a5c885e00` |
| `config/static_experiment_s0.yaml` | `2b2859312fca886a235b30d20727665b841e2f26e826f86fd26cda79a724c08e` |
| `data/nurbs/ik_input.mat` | `79868b03afd68f9d4cca1cba568320f0f48efd355d020a9372f2037549c3293f` |
| `portable_oscbf/data/ik_input.mat` | `d681d67f519f99599185eb2a1b9f432dc8f951b058ad6fcfcfc07662b4c5e922` |
| `config/sensor_extrinsics.yaml` | `f891526a64a8fbfdffcbc465956bd70d50c4165bf5901f55816d1b225507da5a` |
| `config/hardware_joint_zero.yaml` | `752bafed5d1cf9ffc06981a5cdd3bd6352b28a953155ba4bb4729bc4c6f99f57` |

两份 `ik_input.mat` 内容身份不同，调用入口必须记录实际选择的路径。当前 S0 的 `ExperimentConfig.load()` 使用根目录 `data/nurbs/ik_input.mat`，去重后的 `PathGeometry` 为 14,984 个点、总弧长 `1.8167597440585153 m`。这与原始 MAT 样本数量分别记录。

S0 的配置语义 SHA-256 为 `5296a0ea296f9ff0993d3defecbe2c09572c5af7e55ec0b9d5d54cdbfc932182`；场景 canonical JSON SHA-256 为 `8a04159783c8eeb4aae22883aa2453c1e6b6e02458323069a952881ecd99d38c`。场景包含圆柱工件、工作台和自碰撞 mesh，附加障碍物为空。路径半径为 `0.2706 m`，工件半径为 `0.2206 m`，轴向为 `base_link +Y`；外部间距 `0.03 m`，自碰撞间距 `0.01 m`。完整场景、资源路径、资源校验值和初始状态见[运行 manifest](../../../.scratch/full-project-baseline/scenario-s0-manifest.json)，执行限制见[场景摘要](../../../.scratch/full-project-baseline/scenario-s0-summary.json)。

## 实际运行入口与状态来源

- 常规仿真入口为 `run_demo.sh` 与 `launch/mujoco_transition_final.launch.py`。`run_demo.sh` 会清理旧 demo 进程，本轮没有执行。脚本启用的 `oscbf_plant.py` 接收 `/oscbf_command`，经 Ruckig 执行器模型更新状态并发布 `/mujoco_joint_states`。`mujoco_viewer_with_cylinder.py` 消费状态并显示模型。
- 直接使用 final launch 时，`start_oscbf_plant` 默认关闭，过渡回放默认发布到 `/mujoco_joint_states`；此配置的状态来源需要按具体启动参数解释。
- 常规控制链为 `oscbf_controller.py` → `tracking_run.py` → `portable_oscbf/work/jax_control_facade.py`。生产配置的数值积分步长为 10 ms，路径周期为 20 ms，发布频率为 50 Hz。这些是当前配置值，本轮没有测量其完整周期期限。
- 独立静态实验入口为 `scripts/static_obstacle_experiment.py` / console entry `static_obstacle_experiment` → `static_experiment_nodes.py`。该链路拥有独立的 plant、候选、准入回复与反馈记录；`ExecutionSchedule` 每 2 ms 更新。当前 S0 配置控制周期为 100 ms。
- MuJoCo 模型由当前 URDF 和 STL 转换生成。本轮调用真实转换器、`MjModel.from_xml_string()` 和 `mj_forward()`，确认 `nq=nv=njnt=9`、`J1`–`J9` 顺序一致、`nbody=10`、`nu=0`。派生 MJCF 的 SHA-256 为 `365224357f953c84e05901ffecd79e1b75695367d1f92c8a50cf57e6e141ac1b`。没有执行 viewer 或渲染。

## M01–M12 的代码与验收边界

### M01 路径参考

- 源码和资源：`ik_data_loader.py`、`path_following.py`、`jax_path_following.py`、`oscbf_trajectory.py`，以及 `data/nurbs/ik_input.mat`、`portable_oscbf/config/nineaxis.yaml`。
- 当前行为：加载已保存路径，建立弧长采样、局部投影、路径参考和圆柱工具轴向；现有任务参考轴为工具 `+X`。`moving_reference_task.py` 提供移动五维参考及对进度的导数。
- 本轮证据：移动参考真实角度、Jacobian 和参考进度导数测试通过；S0 资源与场景成功解析。
- 待补充：完整工件与画笔身份、原始 NURBS 的任意进度求值与导数、给定工具轴与原始表面法向关系、名义运动字段和连接处连续性。输入位置及使用条件由[资产任务](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/135)记录；接口与实现分别由[M01 路径定义](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/137)、[M01 路径实现](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/152)管理。

### M02 机器人模型

- 源码和模型：`robot_spec.py`、`kinematics_data.py`、`nineaxis_kinematics.py`、`nineaxis_manipulator_jax.py`、`models/ninezzhou/urdf/ninezzhou.urdf`、MoveIt SRDF、碰撞 mesh 和 geometry artifact。
- 当前行为：单一九轴关节链，`J1` 为移动关节，其余八个为转动关节；控制点与 URDF `tool0` 一致，位于 `Link9` 的局部 `+X` 方向 0.235 m。
- 本轮证据：URDF 与 JAX 的关节类型、限位、FK、控制点和独立数值 Jacobian 一致；工具轴五维任务测试通过；真实 URDF 可由 MuJoCo 编译。
- 待补充：可配置机构接口、其他机构独立模型、TCP 与工件真实标定、导数需求和模型版本消费规则。责任为[M02 模型定义](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/136)、[M02/M07 共用模型与约束](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/153)。

### M03 环境模型

- 源码和配置：`perception_bridge.py`、`obstacle_extractor.py`、`livox_mid360/`、`point_cloud_obstacles*.py`、`dynamic_clustering.py`、`fusion_engine.py`、`_collision_scene.py`，以及 `config/perception_runtime.yaml`、`config/sensor_extrinsics.yaml`。
- 当前行为：既有感知链发布几何障碍观测；`CollisionSafety.prepare_scene()` 另有固定容量、checksum、身份、时间、覆盖和 tracking 准入结构。当前 LiDAR 外参 `calibrated=false`，矩阵为占位值。
- 本轮证据：CollisionSafety 场景构造、非法状态与 identity 拒绝通过；没有连接 LiDAR，也没有执行真实点云回放。
- 待补充：已标定 LiDAR 到 `base_link` 的外参、逐点时间 mesh ray self-filter、occupied/free/unknown 与固定环境并集、动态 reachable support tube、`collision_scene_server` 与双缓冲场景。责任任务见[任务依赖](dependencies.md)中的感知分支。

### M04 碰撞安全

- 源码和几何：静态实验使用 `static_collision.py`、`static_collision_meshes.py`、`static_swept_geometry.py`、`static_motion.py` 和 `robot_static_fcl`；通用接口使用 `collision_safety.py`、`_ellipsoid_dcol.py`、`_ellipsoid_point.py`、`_collision_scene.py` 与 `CollisionPolicy`。
- 本轮证据：FCL 分离、接触、相交、最近点、独立圆柱距离、关节梯度、batch 和位移界限通过。通用接口的拒绝路径通过；生产 geometry artifact 的 45 个 self primitive pair、官方 Julia 保存参考、独立高精度距离和九轴梯度通过。
- 当前限制：`CollisionSafety.certify()` 在合法场景仍返回 `CERTIFICATE_FAILED`，有效及证明 mask 均为 false。`src/robot_safecontrol_moveit` 和 AEB 插件当前没有调用该公共接口；生产常规链和静态实验继续各自使用已有碰撞计算。
- 待补充：固定容量活动集合、通用连续区间证明、最新场景准入、预测时刻障碍、目标设备 deadline、统一切换与旧路径处理。静态 Ruckig 区间检查的通过结果仅支持当前静态几何与候选运动；通用切换由[CollisionSafety 唯一来源验收](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/123)管理。

### M05 状态快照

- 源码：`static_experiment_nodes.py`、`static_execution.py`、`tracking_run.py`、`tracking_contract.py`；`runtime_snapshot.py` 保存配置与代码身份。
- 当前行为：静态反馈含 simulation step、位置、速度、加速度、active command、执行参考和阶段，控制器检查场景 identity、递增 step 与 200 ms freshness。常规控制读取 JointState。
- 本轮证据：真实 `ExecutionSchedule` 状态与参考结算、真实 JAX 反馈收敛通过。hardware bridge 缺少真实反馈时维持 `feedback_ok=false`、`watchdog_ok=false`。
- 待补充：跨模块的统一快照、来源和测量/估计条件、实际采用结果、计划起点 identity，以及 `s_plan/s_ref/s_tool` 同时记录。责任为[数据与时间定义](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/142)、[M05/M06 实现](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/155)。

### M06 执行参考

- 源码：`ScheduledMotion`、`ExecutionSchedule`、`static_execution_evaluation.py`。
- 当前行为：`reference_start_m + feed_m_s * effective_s` 在获准运动控制区间内推进，区间后的自动停止保持参考；重复、迟到、pending、period、初态和参考不一致均有拒绝条件。
- 本轮证据：获准运动激活、迟到和参考不一致拒绝、停止后参考保持、调度时刻前进测试通过。
- 待补充：随加减速变化的完整执行参考曲线、部分执行结算、所有切换与取消语义、归一化弧长转换，以及计划和投影进度之间的关系。责任为[M06 进度定义](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/141)、[M05/M06 实现](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/155)。

### M07 运动与工艺限制

- 源码和配置：`actuator_limits.py`、`joint_limit_contract.py`、`actuator_modules.yaml`、`static_experiment_config.py` 与各控制 profile。
- 当前仿真限制：J1 速度 `0.5 m/s`、加速度 `5 m/s²`；J2–J6 速度 `2.93215314336 rad/s`、J7–J9 速度约 `3.6651914292 rad/s`；转动关节加速度 `31.4 rad/s²`。当前 `jerk_time=0.08 s` 派生 J1 jerk `62.5 m/s³`、其余关节 `392.5 rad/s³`。
- 本轮证据：真实 Ruckig 运动与停止和独立 Ruckig 计算一致，连续位置极值、速度/加速度/jerk 界限及停止后运动检查通过。
- 待补充：实际工具工艺速度、加速度和 jerk 的定义与限值、阶段适用条件、真实关节能力与传动资料、其他机构能力。`hardware_validated=false` 及仿真假设保持原标记。责任为[M07 公共限制](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/138)、[模型与约束实现](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/153)、[设备测量](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/165)。

### M08 预测协调

- 当前控制目录中未发现多时段关节运动与进给联合 MPC、计划数据类型或异步计划消费者。`static_experiment_nodes.py` 将已有执行运动外推到下一命令生效时刻，再调用一次当前求解。
- 本轮没有 MPC 验收结果。需要确定状态、输入、执行模型、预测参考、障碍更新时间、时域、后端、热启动、原约束复核和完整计算期限。
- 责任为[联合预测模型与求解方案](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/150)、[M08 联合 MPC 与热启动](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/161)、[异步协同](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/162)。

### M09 实时协调

- 源码：`moving_reference_task.py`、`static_velocity_solver.py`、`static_control.py`、`static_recovery.py`；常规内核入口为 `jax_control_facade.py`。
- 当前行为：`joint` 模式共同求取九个关节速度与一个归一化进给变量，目标使用 `J*u-reference_rate*feed`；约束残差和原始不等式在求解后复核，随后生成并检查 Ruckig 运动。现有模式为 `separated/joint/sns`。
- 本轮证据：实际 QP 与独立 SciPy SLSQP 一致，五维任务保持、限位附近构型辅助、间距约束、求解失败拒绝和已保存进给边界输入通过；JAX 真实反馈在五维与六维任务中从参考前后两侧收敛。
- 待补充：新工艺与电机限制、可变速参考协调、预测建议的当前一步消费、修正原因与实际反馈、跨机构及完整路径验收。责任为[M09 运行规则](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/143)、[M09 联合 QP](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/157)。

### M10 执行准入

- 源码：`static_motion.py`、`static_motion_bounds.py`、`static_swept_geometry.py`、`static_experiment_nodes.py`、`hardware_contract.py`。
- 当前行为：静态链检查候选和停止运动的关节界限、外部间距与自碰撞；plant 再次检查候选并回复接受或拒绝。记录层分别保存候选、命令、准入和反馈。
- 本轮证据：真实 Ruckig/FCL 区间核验通过，包含速度仍在限值内但停止运动超出位置界限的拒绝场景。direct live 和 final launch live 均在控制 I/O 创建前拒绝。
- 待补充：实际 TCP 工艺与高阶导数、动态场景连续证明、统一时序和计算期限、异常后的全部阶段行为。责任为[执行准入定义](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/151)、[M10/M11 执行检查](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/156)。

### M11 执行反馈

- 源码和配置：`driver_simulator.py`、`oscbf_plant.py`、`static_execution.py`、`hardware_bridge.py`、`drempower_can.py`、`socketcan_backend.py`、`unit_conversion.py`、`config/drempower.yaml`、`config/hardware_joint_zero.yaml`。
- 本轮证据：真实 ROS 对象的 sim 模式没有命令订阅、控制状态 publisher 或 timer；shadow 只订阅命令并记录拒绝，命令和 acknowledgement 均不能生成反馈；direct live 与 final launch live 拒绝通过。核验没有创建 CAN backend 或收发设备数据。
- 待补充：bridge 的真实协议接入、反馈来源与时间基准、关节身份映射、零位/正方向、传动与行程、独立 watchdog、物理停止及现场准入。`config/drempower.yaml` 已记录 500 kbit/s 和读到节点 ID 1–9；节点与关节对应关系仍待核实，零位表仍为待验证值。
- 责任为[真实设备协议与执行保护](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/164)、[TCP、工件标定与伺服测量](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/165)及两项受控实机验收。

### M12 记录评价

- 源码：`static_experiment_recording.py`、`static_experiment_provenance.py`、`static_experiment_evaluation.py`、`static_execution_evaluation.py`、`static_clearance_evaluation.py`、`static_cycle_timing.py`，以及常规 `tracking_evaluator.py`。
- 当前行为：SQLite/MessagePack 记录、原始资源和源码身份、获准运动重算、参考结算、局部有序投影、几何路径距离、真实轴向夹角、工作距离、关节运动界限、最小间距和完整周期统计已有入口。
- 本轮证据：真实反馈状态的全部 56 对间距评价与独立查询一致。本轮没有重新评价历史完整实验，也没有运行正式统计。
- 待补充：三类进度、实际工具高阶导数、计划采用与修改、各关节能力利用率、新三方法共同条件、全程失败分类与正式配对统计。责任为[M12 记录回放评价](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/159)、[全项目验收定义](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/144)、[三方法验证与正式统计](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/169)。

### Runtime

`transition_executor.py`、`transition_planning_server.py`、`motion_planning.py`、`continuous_ik.py`、`trajectory_execution.py` 和 AEB-RRT* 插件提供既有入场目标、MoveIt 过渡、Ruckig 处理与跟踪交接。静态实验的 `static_recovery.py` 提供暂停、确认、恢复、终点与 fault 状态。统一加工阶段、入场退出边界、异步计划和异常监督由[独立作业 Runtime](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/158)及后续验收管理。

## 旧执行任务与新路线的连接

- [运动学数据单一来源](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/84)、[依赖锁定](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/85)、[QoS 与时延测量](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/93)保留原实现和历史测量；新模型接口、跨模块时间及目标设备期限分别需要当前任务证据。
- [可重复 S0 仿真与反馈评价](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/172)、[共享路径五维跟随](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/173)、[臂身避让](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/174)、[mesh 自碰撞](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/175)为 M01/M02/M04/M09/M12 提供已有代码与原始记录。
- [Ruckig 命令检查与进度结算](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/176)、[构型调整与基础恢复](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/178)、[联合进给求解](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/179)、[持续确认恢复](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/180)为新 M06/M09/M10/M11 提供复用起点；新参考曲线、实际工具限制与 MPC 接入继续由对应任务完成。
- [S2–S5 场景及见证](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/177)、[Generalized-SNS 外部参考](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/181)保留场景和方法来源。当前三方法标识为 `pointwise_ik/realtime_coordination/predictive_coordination`，其完整关节轨迹比较、全路径轨迹优化和统一统计尚待当前任务实施。
- [CollisionSafety 公开接口](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/103)、[参数与 policy](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/102)、[场景准入](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/100)、[几何资产](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/106)、[self DCOL](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/107)、[环境 barrier 和毫米距离](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/109)为通用碰撞分支提供接口与数值代码。真实感知、连续 certify、生产接入和唯一来源切换仍有开放任务。

## 本轮命令与实际结果

核验入口脚本为[run_checks.sh](../../../.scratch/full-project-baseline/run_checks.sh)。脚本加载已存在的独立环境，按核验组保存完整 stdout/stderr 和退出码；没有调用 `run_demo.sh`、设备检查或全量测试脚本。

```bash
bash .scratch/full-project-baseline/run_checks.sh inventory
bash .scratch/full-project-baseline/run_checks.sh scene
bash .scratch/full-project-baseline/run_checks.sh geometry
bash .scratch/full-project-baseline/run_checks.sh kinematics
bash .scratch/full-project-baseline/run_checks.sh collision
bash .scratch/full-project-baseline/run_checks.sh collision-numeric
bash .scratch/full-project-baseline/run_checks.sh coordination
bash .scratch/full-project-baseline/run_checks.sh feedback
bash .scratch/full-project-baseline/run_checks.sh runtime-probe
bash .scratch/full-project-baseline/run_checks.sh ctest
```

| 核验组 | 实际范围 | 结果与原始日志 |
|---|---|---|
| geometry | `test_moving_reference.py`、`test_static_geometry.py`、`test_static_motion.py`、`test_static_execution.py`、`test_static_clearance.py` | [31 passed，55.62 s](../../../.scratch/full-project-baseline/geometry.log) |
| kinematics | `test_fk_matches_urdf.py`、`test_tool_axis_task.py` | [10 passed，2.17 s](../../../.scratch/full-project-baseline/kinematics.log) |
| collision | `test_collision_safety.py` | [22 passed，7.07 s](../../../.scratch/full-project-baseline/collision.log) |
| collision-numeric | 生产 self pair、官方 Julia 保存参考、球体有符号距离、旋转 ellipsoid 与九轴梯度 | [18 passed，56.89 s](../../../.scratch/full-project-baseline/collision-numeric.log) |
| coordination | `test_static_velocity_solver.py` | [8 passed，11.31 s](../../../.scratch/full-project-baseline/coordination.log) |
| feedback | `test_jax_path_position_feedback.py` | [4 passed，31.35 s](../../../.scratch/full-project-baseline/feedback.log) |
| runtime-probe | 真实 MuJoCo 编译/FK、真实 ROS sim/shadow、live 双入口拒绝 | [退出码 0；结构化结果](../../../.scratch/full-project-baseline/runtime-probe.json) |
| ctest | 现有 `test_aeb_rrtstar`、`test_aeb_full`、`test_plugin_load`、`test_plugin_init` | [4/4 passed，3.30 s](../../../.scratch/full-project-baseline/ctest.log) |

以上 93 项 pytest、4 项 ctest 及运行探针全部通过。核验没有使用 monkeypatch、mock 或替换求解器。ctest 使用 2026-10-20 已构建的本机二进制，文件 SHA-256 已固定；本轮没有重新构建。插件初始化实际读取的安装 URDF 链接指向当前源码模型。

未执行：全量 pytest、真实 MoveIt 过渡闭环、LiDAR 现场/回放、完整曲面三方法、新 MPC、其他机构、设备动作、实机停止、目标设备完整周期和正式统计。相关能力与准入结论继续以各自开放任务的直接证据为准。

本任务所需的代码与依赖身份、M01–M12 入口、当前真实测试和差异清单已经保存。资产取得状态及可执行的 MATLAB/Python 来源由[资产清单](asset-inventory.md)独立登记，缺失资产和现场测量保留为后续任务的前置条件。
