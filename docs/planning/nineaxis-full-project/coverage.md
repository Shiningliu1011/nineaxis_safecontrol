# 完整项目需求与模块覆盖

本页对应[总体规格](../../specs/predictive-coordination-control.md)中的 R01–R12。任务状态由[完整项目地图](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/147)管理；下面的对应关系表示责任范围。

## 需求追踪

| 需求 | 定义与决定 | 实现 | 验收 |
|---|---|---|---|
| R01 路径、轴向与导数 | [M01 路径参考](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/137) | [M01 路径实现](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/152) | [完整曲面仿真](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/160) |
| R02 工具工艺与电机限制 | [M07 公共限制](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/138) | [M02/M07 模型与约束](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/153) | [完整曲面仿真](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/160)、[设备测量](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/165) |
| R03 当前运动与进给共同求解 | [M09 定义](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/143) | [M09 实时协调](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/157) | [独立闭环](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/160) |
| R04 加减速、参考与恢复 | [M06 执行参考](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/141) | [M05/M06](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/155)、[M10/M11](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/156)、[M09](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/157) | [独立闭环](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/160) |
| R05 全身与停止安全 | [M03/M04 定义](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/139) | [场景与碰撞](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/154)、[连续检查](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/113) | [独立几何](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/125)、[规划控制恢复](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/127) |
| R06 多时段联合 MPC | [预测模型与求解方案](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/150) | [M08 实现](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/161) | [协同仿真](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/166) |
| R07 计划、修正与实际反馈 | [数据与时间](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/142) | [异步协同](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/162) | [协同与异常](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/166) |
| R08 三方法公平比较 | [已确认验收方案](decision-144-acceptance.md) | [逐点逆解与轨迹](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/110)、[M09](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/157)、[M08](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/161)、[M12](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/159) | [三方法验证、周期与正式统计](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/169) |
| R09 九轴模型独立验证 | [模型接口](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/136) | [九轴模型、几何与约束](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/153) | [独立闭环](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/160)、[协同仿真](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/166)、[三方法验收](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/169) |
| R10 静态与规则动态障碍 | [场景接口](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/139) | [动态场景](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/101)、[预测碰撞接入](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/154) | [完整闭环](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/160)、[动态仿真](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/166)、[受控实机](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/168) |
| R11 真实反馈与停止 | [执行准入](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/151) | [设备协议](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/164)、[标定与测量](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/165) | [独立实机](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/167)、[协同实机](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/168) |
| R12 完整项目复现 | [总体规格](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/145) | [M12](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/159)及全部模块任务 | [复现与个人成果展示](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/170) |

## 专项规格与实施责任

[DCOL 与双传感器规格](../../specs/dcol-collision-core.md)管理碰撞专项要求，[模块边界与清理规格](../../specs/module-boundaries-and-reuse.md)管理现有功能整合。两份规格在各自范围内优先；同一能力共用一个实施任务，既有完成记录保持原验证范围。

| 当前要求 | 实施与独立验收责任 |
|---|---|
| 碰撞 R01/R02、T01/T02：OBB、拓扑、附件与完整覆盖 | [OBB 几何资产](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/198)、[独立几何验收](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/125) |
| 碰撞 R03/R07、T03–T05：凸体 DCOL、几何外扩及导数 | [DCOL 后端及导数](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/199)、[独立 query 行](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/112)、[独立数值验收](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/125) |
| 碰撞 R04/R05、T06–T09/T11：双源、占据、时间和覆盖 | [PCL/OctoMap 共享场景](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/200)、[双源基座标定](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/95)、[真实回放](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/122) |
| 碰撞 R07/R08、T12/T13：独立行、连续运动及停止 | [query](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/112)、[certify](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/113)、[CBF 行与期限](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/118)、[场景接入](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/154) |
| 碰撞 R09、T16：完整周期、参数和适用条件 | [参数测量](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/117)、[目标设备完整周期](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/119) |
| 碰撞 R06/R10、T10/T14/T15/T17：回放、仿真、在线与装置 | [双源回放](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/122)、[规划控制恢复仿真](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/127)、[在线双源及丝杠障碍](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/205) |
| 碰撞全部 R/T 与来源切换、清理及文档 | [唯一碰撞来源](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/123)、[碰撞清理](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/124)、[当前文档](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/120)、[通用碰撞证据核对](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/126) |
| 模块 R01–R03、T01/T02：独立边界、共同模型/限制/参考 | [现有消费者复用](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/201)，其余消费者任务使用相同公共接口 |
| 模块 R04、T03：配置、方法可用性与共同入口 | [运行入口、配置与演示](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/202)，完整新增方法由原任务注册和验证 |
| 模块 R05、T05：OBB 与共同碰撞 | 共用本表的 OBB、DCOL、双源、query/certify 和专项证据 |
| 模块 R06、T04/T09：替换后的清理与文档 | [共同入口](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/202)、[碰撞清理](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/124)、[当前文档](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/120)；KDL/PSO 接入等待[五维 IK](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/111)，正式[AEB-RRT*](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/115)继续保留 |
| 模块 R07、T04：独立 Python benchmark 清理 | [benchmark 专用实现清理](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/203) |
| 模块 R08、T06：速度、加速度、jerk | [共同联合 QP](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/157)与[公共运动/最终准入](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/156)，限值与必要约束沿用原确认 |
| 模块 R09、T07：Ruckig 边界与完整停止 | [公共运动与仿真执行](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/156)，取消前逐项满足 I08 |
| 模块 T08：反馈、执行进度、恢复与只读评价 | [状态与进度](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/155)、[Runtime](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/158)、[记录评价](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/159) |
| 模块 R10 及全部 R/T：本轮完整整合验收 | [模块边界、复用与清理验收](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/204)；完整 MPC、完整逐点方法和正式三方法比较各自继续验收 |

需求确认、源码实施、数值通过、仿真通过和现场证据分别记录；必要替换尚缺前置时相应项目保持未完成。当前原生关系见[依赖表](dependencies.md)。

## 系统与阶段

M01/M02/M07 提供共用路径、模型和限制；M03/M04 提供环境及碰撞；M05/M06 提供反馈与执行参考；M08/M09 负责预测和实时协调；M10/M11 负责检查、执行和反馈；M12 负责独立评价。Runtime 管理工艺阶段、入场退出、异步计划、异常和完成条件。

[已确认Runtime规则](decision-151-runtime.md)明确阶段、工艺许可、最终运动与停止准入、执行证据及异常恢复；软件数值检查、仿真、设备保护和物理停止分别取得证据，真实反馈及时序由#164/#165验证。

原 LiDAR 占据、自体过滤、动态 tracking、场景服务、primitive query、连续检查、CollisionSafety 参数测量、shadow、目标设备期限、模块切换和文档任务继续保留。各任务的完整原生依赖见[依赖表](dependencies.md)。

| 阶段 | 必需交付与个人能力证据 |
|---|---|
| 参数与资产 | 能解释 NURBS 数据、坐标、工具轴和限值来源，独立核验原始资料 |
| 参考与模型 | 能生成蝴蝶曲线的连续参考文件，核验九轴运动学、Jacobian、距离、路径导数及模型配置 |
| 实时协调 | 能说明五维共同求解、加减速与参考关系，依据实际日志定位受限原因 |
| 预测协同 | 能推导预测问题，说明求解器选择、运动状态、时间网格、有效期及反馈作用 |
| 系统与实机 | 能验证感知、通信、标定、停止和执行条件，解释仿真与测量差异 |
| 三方法正式实验 | 能复现合规性、完成率、配对统计和消融，解释电机能力使用与效率关系 |
| 最终交付 | 能完整说明系统，运行和复现实验，展示有数据支持的个人贡献 |

## 验收原则

三个方法共享当前九轴机械臂的任务、工艺、关节限制、几何、初态和执行条件。共同满足路径、轴向、工具运动、安全与时间要求后比较效率，报告全部成功和失败。仿真和真实设备各自保留独立结果。

M12 记录 `s_plan/s_ref/s_tool`、实际工具导数、每个关节的速度/加速度/加加速度利用情况、限制对象、停顿、恢复和计算耗时。真实 TCP 精度由独立测量支持；正式结论通过原始数据重算核验。
