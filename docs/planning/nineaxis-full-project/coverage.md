# 完整项目的模块与阶段覆盖

[完整项目地图](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/147)保存任务与状态。本页提供模块、阶段与个人学习证据的导航；具体定义、阈值和验收以对应任务的确认评论为准。

## 模块覆盖

| 模块 | 需要完成的能力 | 实现与验收任务 |
|---|---|---|
| M01 | 工件、画笔、NURBS、名义速度、同步位置／轴向及导数 | [实现 M01 工件、画笔、NURBS 与路径参考](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/152) |
| M02 | 九轴模型、坐标、TCP、工具轴、运动学与 Jacobian | [实现 M02/M07 共用模型、导数与约束](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/153)、[完成 TCP、工件标定与伺服时序测量](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/165) |
| M03 | LiDAR 环境扫描、时间、外参、自体过滤、固定模型与场景服务 | [建立 LiDAR occupied/free/unknown 场景与固定模型并集](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/108)、[实现逐点时间 mesh ray self-filter](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/105)、[实现 collision_scene_server 与双缓冲共享场景](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/104) |
| M04 | 连杆、工具、自碰撞、工件与环境距离、梯度及连续检查 | [实现 M03/M04 工件场景与预测碰撞接入](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/154)、[独立几何与 kernel 正确性验收](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/125) |
| M05 | 真实反馈、一致时间与状态快照 | [实现 M05 状态快照与 M06 执行进度结算](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/155) |
| M06 | 唯一共享进度、有效执行区间、重复及部分执行处理 | [实现 M05 状态快照与 M06 执行进度结算](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/155) |
| M07 | C01/C02/C03/C05/C06 与工作距离判据的共用配置 | [实现 M02/M07 共用模型、导数与约束](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/153) |
| M08 | 构型与进度 NMPC、热启动、未来约束、期限与复核 | [实现 M08 联合 MPC 与热启动闭环](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/161)、[实现未来冗余构型优化与消融入口](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/163) |
| M09 | 五维跟踪、联合 `u,ν` QP、受限减速、局部构型与协同引导 | [实现 M09 五维任务与进度联合 QP](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/157) |
| M10 | 最终指令校验、异常处置、watchdog 与恢复条件 | [实现 M10/M11 仿真执行与最终指令校验](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/156)、[实现碰撞故障锁存与受限恢复](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/114) |
| M11 | MuJoCo、真实设备协议、执行反馈与保护 | [实现 M10/M11 仿真执行与最终指令校验](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/156)、[实现 M11 真实设备协议与执行保护](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/164) |
| M12 | 日志、回放、整段质量、对照统计与图表 | [实现 M12 记录、回放与整段评价](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/159) |
| Runtime | 初始 IK、入场退出、调度、PlanReceiver、状态转换 | [实现独立作业 Runtime 与安全入场退出](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/158)、[实现异步计划接收与上下层协同](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/162) |

M01、M05、M08、M10、M11、M12 和 C05/C06 均由刘世宁负责完整实现与理解。原资料中的分工可作为资料来源，项目任务不依赖另一位开发者交付这些模块。

## 阶段与可检查结果

| 阶段 | 项目结果 | 个人能力证据 |
|---|---|---|
| S0 参数与接口 | 资产与基线、全模块接口、模型和数值来源、设备测量计划 | 能说明每个量的单位、坐标、时间及有效条件，独立运行基线 |
| S1 参考与模型 | 三类工件路径、模型与导数、场景查询、初始 IK 与入场退出 | 能生成新路径，独立核验 Jacobian、距离与导数，解释工具轴自由度 |
| S2 独立 QP | 无 MPC 的完整仿真闭环，受限减速、静态避障、暂停恢复及异常；准入后低速实机 | 能解释联合 QP、进度结算与约束，利用实际日志定位问题 |
| S3 预测协同 | MPC 与同一 QP 闭环、异步计划接入、提前减速与冗余构型安排 | 能推导预测模型，解释热启动、时间网格、计划期限及构型作用 |
| S4 实机与鲁棒性 | 实测标定、伺服与时序；独立／协同实机及受控动态场景 | 能解释仿真与设备差异，评估测量误差、停止条件与适用范围 |
| S5 正式实验 | 冻结开发／测试范围，配对对照、消融、统计及全部失败记录 | 能从原始数据复现图表，说明公平条件和结果的限制 |
| S6 成果与复现 | 技术报告或论文、演示、运行手册、数据与最终证据索引 | 能完整讲解系统，独立运行与复现实验，展示有依据的个人贡献 |

设备资料、现场准备与测量从 S0/S1 安排；满足对应准入条件后开展低速实机。阶段代表前置能力与验收顺序，日期由资料、设备和实际结果确定。

阶段验收入口：[独立 QP 完整曲面仿真闭环](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/160)、[双速率协同与动态异常仿真](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/166)、[低速独立 QP 实机作业](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/167)、[MPC 协同实机与受控动态作业](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/168)、[正式对照、消融与统计实验](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/169)、[完整项目复现与个人成果展示](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/170)。

## 项目验收依据

独立基线 B 与协同方法 M 使用相同路径、初态、五维任务、名义速度、已启用边界、碰撞余量和执行接口。质量与约束合格后比较整段完成时间；首次规划与入场开销单独记录。

整段时间包含允许的减速、暂停与在线重规划。记录成功率、停顿、位置／轴向／工作距离误差、最小间隙、超时和完整失败。以场景及初态为独立实验单位，控制采样点不能代替重复实验。

最终交付应能追踪从原始需求到决定、实现和验收的关系。真实 TCP 精度需独立测量支持；真实运行、live 准入与安全边界调整在对应阶段取得具体授权。
