# 九轴曲面作业完整项目

研究目标：研究冗余机械臂的在线约束协调方法，在满足工艺和电机限制、保持末端路径与工具轴向的条件下，实现实时全身避障，并充分利用电机能力，提高作业效率。

项目负责人为 `lsn`，覆盖 M01–M12、Runtime、LiDAR、CollisionSafety、系统集成、仿真、受控实机、正式实验及个人成果展示。仓库由 `lsn` 在不同设备上使用，项目开发和跨设备接手使用 `main`。

- [完整项目 Wayfinder 地图](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/147)：唯一当前地图，保留原任务身份、完成证据和已有指派。
- [总体项目规格](../../specs/predictive-coordination-control.md)：目标、模块、约束、三类进度、三种方法及验收；[GitHub 正文](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/145)。
- [实现方案](implementation-design.md)：共同输入、实际运动与参考、MPC 研究、时序及实施顺序。
- [需求与模块覆盖](coverage.md)：需求、决定、实现和验收的对应。
- [任务编号与依赖](dependencies.md)：59 项主线任务的连续编号、主要依赖图、全部直接前置和可领取入口。
- [事实依据](requirements-evidence.md)：当前控制代码、NURBS ZIP 参数与独立核查、实验状态和数据保存。
- [基线核验](baseline-audit.md)：当前 M01–M12 能力、实际运行与待补充输入。
- [原始资产与 Python 运行核验](asset-inventory.md)：来源身份、资料包、真实样本与完整生成证据；[Python 使用说明](../../pipe_assets.md)。
- [电机与 NURBS 约束数值依据](motor-nurbs-constraint-limits-research.md)：厂家能力条件、源程序限制、独立数值核验及尚待确定的研究参数。
- [MPC求解研究](mpc-solver-research.md)：九轴连续参考、导数接口、真实候选测量及完整约束/预算的待测条件。
- [地图维护验证](verification-20261027.md)：原生关系、取消状态、文档与本地远程同步检查。
- [前期工作证据](../../static_obstacle_experiments_handoff.md)：已完成基础能力、原始记录和复用范围。
- [九轴技术路线来源](../../../九轴技术路线/九轴机械臂曲面作业_技术路线.md)：保留师兄提供的模块与候选设计，当前要求由总体规格管理。

## 任务使用

开放任务从地图的原生子 issue 查询，原生 blocking 表示前置关系。开放、没有开放前置且未指派的任务可以领取。标题按依赖使用 01–59 编号，GitHub issue 编号和评论保持永久身份。

[核验完整项目基线与 M01–M12 资产](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/148)和[取得完整工件、路径与 MATLAB 程序资产](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/135)已完成核验与运行交付。[[04] 完整项目技术复用与实施依赖（需你拍板）](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/149)、[[05] M02 九轴模型、TCP 与标定接口（需你拍板）](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/136)及[[06] M01 工件路径与五维任务参考（需你拍板）](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/137)已经用户确认并完成。研究机构采用现有九自由度机械臂，跟踪现有蝴蝶曲线，缺少信息使用新 NURBS 程序生成新文件。

[[07] M07 公共限制与完整作业容差（需你拍板）](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/138)与[[09] M06 执行记录与共享进度结算（需你拍板）](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/141)已确认并完成决定，完整规则见[共同限制](decision-138-limits.md)和[执行进度](decision-141-progress.md)。[[21] 建立 LiDAR occupied/free/unknown 场景与固定模型并集](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/108)已集成至main工作区，272项回归和独立双线审查通过。

[[08] M03/M04 场景与预测碰撞接口（需你拍板）](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/139)的[完整方案](decision-139-scene.md)、#140的[入场与比较方法方案](decision-140-entry-methods.md)、#142的[共同数据与时间方案](decision-142-data-time.md)、#151的[Runtime方案](decision-151-runtime.md)、#143的[联合QP方案](decision-143-joint-qp.md)及#144的[验收与正式实验方案](decision-144-acceptance.md)已经用户确认并完成。#150的[MPC研究](mpc-solver-research.md)已保存，完整约束、后端及周期仍待测量，任务保持开放。#155状态/进度、#156仿真执行和#159记录评价均已解除阻塞，可以领取。

[[17] 实现 M01 工件、画笔、NURBS 与路径参考](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/152)已完成main集成与验收，[完成记录](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/152#issuecomment-6445339517)保留两份新生成包、原样本核验和全部测试条件。[[18] 实现 M02/M07 共用模型、导数与约束](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/153)已完成main集成，79项相关检查、隔离安装和独立双线审查通过，[完成记录](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/153#issuecomment-6442936431)保存具体证据。#105逐点时间过滤已完成main集成，311项相关检查与安装导入通过，见[完成记录](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/105#issuecomment-6445352584)。该阶段共同功能覆盖为主包847项、内核706项及AEB4项通过，40项原条件跳过；计时仅在分别记录的CPU条件下通过，全部CPU结果仍失败。

[[23] 建立动态 tracking 与 reachable support tube](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/101)已完成main集成和独立双线审查；最终组合456项相关消费者、30项独立及组合检查、43项安装公开检查通过，36份安装资源身份一致，见[完成记录](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/101#issuecomment-6446396500)和[模块接口](../../modules/dynamic_support.md)。原9ms静态个案在四个P-core条件保留原断言通过。标定和基础文档沿用现有指派；MPC的数学模型、求解器与周期由[联合预测模型与求解方案](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/150)研究确定。

[[19] 实现 M11 真实设备协议与执行保护](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/164)已领取。用户确认全部电机资料位于项目 `大然电机/`；厂家协议、SDK 和现有通信记录的[核验结果](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/164#issuecomment-6459917358)已保存。命令实际采用、反馈采样时间、全轴身份及独立保护仍缺少确认依据，任务保持开放。

[[24] 实现 collision_scene_server 与双缓冲共享场景](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/104)已完成并关闭，22 份文件已集成至 `main`，本次代码、测试及相关文档纳入提交与远程同步范围。322 项受影响真实测试、69 项纯逻辑检查、独立 Spec/Standards 审查及安装后的默认 ROS 入口验收通过，13 份安装文件与源码身份一致，见[完成记录](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/104#issuecomment-6460673198)及[运行说明](../../modules/collision_scene_server.md)。验证覆盖真实进程、共享内存和 JAX；真实 LiDAR 回放、现场参数、目标设备 deadline 与正式控制来源切换分别由对应任务验收。

总体规格保持开放，负责参数、决定与验收对应的完整性核对。三方法的共同条件、周期判据、正式规模与统计定义由 #144 确认，#169 负责三方法联调、完整周期测量、正式采集和统计。#169 的直接前置为 #110、#144、#145、#168。

静态实验任务 #171、#182、#183 已取消；#172–#181 保留已完成证据。这 13 项任务均退出当前地图，其原始记录用于既有能力复核。

## 项目阶段与证据

实施顺序为资产与公共定义、参考和模型、实时协调与运动参考、预测反馈、三方法九轴仿真、系统与实机、正式统计及复现。原地图中的感知和通用碰撞分支按各自前置并行推进。M12 从独立闭环开始参与。

每项完成依据包括运行入口、环境与配置、模型和数据身份、原始日志及实际验收结果。规格交付、源码检查、仿真通过和实机准入分别记录。历史结果用于其原运行条件，新要求通过后由对应任务记录新证据。

每次领取任务均读取其当前原生前置和验收条件。现场测量、真实设备运行及正式采集按对应任务的授权和准入要求执行。
