# 离线 ticket 执行顺序

更新：2026-09-12。用户已要求把当前可以独立处理的离线部分拆成新票。本文件是离线队列入口；`OFF-xx` 是标题票号，GitHub 数字仅用于链接和原生依赖。

本轮拆出 18 张离线票，关联 19 张开放原票。新票全部直接归入 Wayfinder 地图；原票仍管理整体交付、剩余决策和相应现场验收，不重复实施已拆出的部分。原票不自动关闭，也不一概变成只能等硬件的票。

## 如何执行

- 下表是单人串行推荐顺序；“硬前置”才是必须完成的依赖，其余相邻位置只是优先级。事实/接口已足够的本地实现和隔离测试可按票推进。
- 新票的硬前置只指向其他离线票；原票作为来源和后续验收引用，不反向阻塞新票。原有整票依赖保留，原票另依赖其拆出的新票，避免产生循环。
- 先核对已有代码、PR 和实验，只补缺项。原有 11 的 PR #43、几何第二轮证据、性能报告和已修测试均继续复用。
- 任务票的代码和相关测试通过后才可关闭；研究原型可交付有依据的负结果，但不能把“未验证”传播成下游能力已具备。
- 所有合成阈值/时间/负载都必须注明，未知现场参数保持未知。离线完成不开放 live，不触发实机动作。

## 推荐串行顺序

| 执行票 | 本轮交付 | 硬前置 | 来源原票 |
|---|---|---|---|
| [OFF-15](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/44) 跟踪评价指标与证据等级落地 | 先统一离线实验的评价口径，防止不同起点、测量边界和失败状态被混为成功。 | 无 | [13](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/19) |
| [OFF-01](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/45) 标定记录加载与运行时身份统一 | 让标定记录成为可验证的单一数据来源，完成不需要真实传感器的加载、迁移和诊断部分。 | 无 | [T7](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/27) |
| [OFF-02](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/46) OBB 重叠判据与任务关节域离线验证 | 把已发现的重叠盲区和省略碰撞对问题转化为可复核的离线几何证据及准入接口。 | 无 | [12](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/8)、[08B](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/18) |
| [OFF-09](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/47) OSCBF 求解后准入与冻结恢复状态机 | 落实已接受的准入与锁存决定，修复 QP 失败仅影响单 tick 的连接缺口。 | [OFF-02](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/46)、[OFF-15](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/44) | [08B](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/18)、[E0](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/33) |
| [OFF-04](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/48) 执行会话与 SocketCAN 协议离线验证 | 完成执行会话的软件契约及模拟故障验证，复用已存在的 SocketCAN 草稿。 | 无 | [11](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13) |
| [OFF-05](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/49) 双源官方驱动离线构建与消息边界核验 | 把官方驱动的可复现软件准备与设备在线验收分开。 | 无 | [P0](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/31) |
| [OFF-03](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/50) 环境占据几何与九轴距离导数对照 | 为环境 OBB—占据体素查询提供可靠的距离、身份、有效性和九轴导数接口证据。 | [OFF-02](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/46) | [12](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/8) |
| [OFF-06](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/51) 逐源时间与显式障碍观测契约 | 实现可由合成数据验收的观测和时间契约，替代不携带充分身份/有效性的隐式输入。 | [OFF-01](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/45)、[OFF-03](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/50)、[OFF-05](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/49) | [10](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/7)、[T1](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/21) |
| [OFF-07](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/52) 逐源 MoveIt 自体过滤接入与误删回归 | 在明确采集时刻和标定身份的输入上，完成可离线验证的逐源本体过滤。 | [OFF-06](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/51) | [T2](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/22) |
| [OFF-08](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/53) 感知无效状态到停止请求的离线接线 | 把感知失效转换成明确的停止/锁存事件，供控制和执行端消费。 | [OFF-06](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/51)、[OFF-07](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/52) | [T3](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/23) |
| [OFF-10](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/54) 5D 精度约束下的九轴协调与进给对照 | 验证外部障碍下保持末端精度、尽量保持进给的控制方法，而非重复无障碍局部冗余实验。 | [OFF-03](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/50)、[OFF-06](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/51)、[OFF-09](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/47)、[OFF-15](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/44) | [C0](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/32)、[08A](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/6) |
| [OFF-11](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/55) 约束构造优化与同条件性能比较 | 针对已有证据定位的约束构造成本，寻找保持数学/几何语义的可复现优化。 | [OFF-10](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/54) | [C0](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/32)、[01A](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/3) |
| [OFF-18](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/56) 仿真退出异常与完整生命周期回归 | 处理现有测试票剩余的 launch 退出验收缺口，不重复已通过的历史修复。 | 无 | [02](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/9) |
| [OFF-13](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/57) 滤波后最终命令检查与执行保护闭环 | 把控制器候选、后处理、停止事件和执行会话接成一条可离线验证的命令链。 | [OFF-04](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/48)、[OFF-08](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/53)、[OFF-09](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/47)、[OFF-10](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/54) | [E0](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/33)、[11](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13) |
| [OFF-14](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/58) 合成双源到控制执行的闭环场景验证 | 形成不依赖设备的完整场景通道，验证真实软件接线和任务/故障行为。 | [OFF-07](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/52)、[OFF-08](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/53)、[OFF-11](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/55)、[OFF-13](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/57)、[OFF-18](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/56) | [T4](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/24)、[08A](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/6) |
| [OFF-12](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/59) 完整软件时延与参数化停止预算 | 在集成后的离线软件链上给出可追溯耗时和预算输入，保留实际执行器参数的未知性。 | [OFF-14](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/58) | [01A](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/3)、[01B](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/17) |
| [OFF-16](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/60) 标定工具离线适配与退化数据验证 | 完成标定工具输出到统一记录的离线通路，不等待现场采集。 | [OFF-01](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/45) | [T5](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/25) |
| [OFF-17](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/61) 启动自检离线汇总与模式拒绝验证 | 把已实现的软件与证据条件汇入自检，保证缺少现场依据时明确拒绝相应模式。 | [OFF-01](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/45)、[OFF-12](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/59)、[OFF-13](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/57)、[OFF-16](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/60) | [T6](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/26) |

## 顺序的理由

先做 OFF-15 统一评价口径，随后 OFF-01 固定标定身份、OFF-02 建立几何判据，OFF-09 才能把“求解成功”与“准入通过”分开。OFF-04 的执行会话、OFF-05 的驱动软件准备和 OFF-18 的退出修复都可独立开始，不必等待整条算法链。

OFF-03 给出环境距离与导数接口，OFF-06 汇合时间/来源/几何契约，再做 OFF-07 过滤和 OFF-08 感知健康。OFF-10 以这些输入对比活跃障碍下的九轴协调；OFF-11 随后对同一控制对象优化约束构造，避免先优化一个马上更换的测量对象。

OFF-13 汇合控制、感知和执行会话，检查滤波后的最终命令。OFF-14 把真实软件组件接成合成闭环，OFF-12 再测整链时延并形成参数化预算。OFF-16 可在 OFF-01 之后穿插执行；OFF-17 最后汇总身份、状态、保护和预算条件，验证缺证据时的明确拒绝。

## 可独立推进的分支

- 起始无开放前置：[OFF-15](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/44)、[OFF-01](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/45)、[OFF-02](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/46)、[OFF-04](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/48)、[OFF-05](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/49)、[OFF-18](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/56)。这表示依赖关系允许启动，不代表已实施、已认领或无需处理票内明确的事实缺口。
- 控制主线：OFF-02 → OFF-09；OFF-03/06 齐备后 → OFF-10 → OFF-11。
- 感知主线：OFF-01/03/05 → OFF-06 → OFF-07 → OFF-08。
- 执行主线：OFF-04 与 OFF-08/09/10 → OFF-13。
- 验证收口：OFF-07/08/11/13/18 → OFF-14 → OFF-12 → OFF-17；OFF-01 → OFF-16 → OFF-17。
- “可独立推进”是工作依赖说明，不自动启动多 agent、多个任务或多个 ROS 进程。

## 原票与离线票的对应

| 原票 | 已拆出的离线执行票 | 仍需原票管理 |
|---|---|---|
| [[01A] 19.4ms 回归剖面与成本分布测量 (prototype)](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/3) | [OFF-11](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/55)、[OFF-12](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/59) | 生产后端/周期定案、目标实机负载与最坏响应上界继续归 01A/01B/C0；实测制动、执行器/通信最坏响应和最终控制周期/有效期由 01B、11、H0 承接。 |
| [[08A] CBF/QP 可行性实证测量：障碍 + 自碰撞工况 (prototype)](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/6) | [OFF-10](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/54)、[OFF-14](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/58) | 控制方法最终选用、现场任务精度与执行器一致性仍由 C0/08A 及实机验收确认；真实双源数据、标定、执行反馈及物理故障处理的证据留在 T4/08A 和现场票。 |
| [[10] 感知时间同步与延迟模型（点云融合 → CBF 输入）](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/7) | [OFF-06](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/51) | 实际时钟映射、曝光/扫描语义、运动界及最终 age 阈值仍由 10/T1 和预算票确认。 |
| [[12] 自碰撞 OBB 与环境几何模型核对（研究）](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/8) | [OFF-02](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/46)、[OFF-03](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/50) | 实物附件覆盖、真实误差及停止能力未进入此离线几何结论；完整实机运行域和最终逐对阈值仍由 12/08B 验收；实际点云覆盖、真实装配和在线尾延迟仍需原 12 票与整链/现场验收。 |
| [[02] 主包测试套件修复（settle harness / perf 孤立性 / e2e steps 断言）](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/9) | [OFF-18](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/56) | 最终性能预算仍由 01B 决定；根因未修时原 02 票仍保留退出失败验收项。 |
| [[11] 真机执行链路:SocketCAN 后端与参数注入](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13) | [OFF-04](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/48)、[OFF-13](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/57) | vcan 未运行时继续是 11 的 OS 传输验证缺口；真实设备/固件、全轴标定、独立看门狗和物理停止由原票承接；真实执行器响应、独立硬件看门狗、承重/制动能力继续阻止 E0/11/H0 的现场验收。 |
| [[01B] 选定控制率/延迟预算与实现策略 (grilling)](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/17) | [OFF-12](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/59) | 实测制动、执行器/通信最坏响应和最终控制周期/有效期由 01B、11、H0 承接。 |
| [[08B] 选定不可行/裕度/降级策略 (grilling)](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/18) | [OFF-02](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/46)、[OFF-09](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/47) | 实物附件覆盖、真实误差及停止能力未进入此离线几何结论；完整实机运行域和最终逐对阈值仍由 12/08B 验收；实机数值阈值和物理停止不在本票定案；最终滤波/发送接线由 OFF-13，整体责任保留在 08B/E0。 |
| [[13] 更新 tracking_evaluator 指标定义与阈值 (impl)](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/19) | [OFF-15](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/44) | 最终阈值及现场任务验收仍依赖原 13、01B、08B 的证据和决定。 |
| [[T1] /perception/tracks 契约升级: 显式 obstacle observation (impl)](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/21) | [OFF-06](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/51) | 实际时钟映射、曝光/扫描语义、运动界及最终 age 阈值仍由 10/T1 和预算票确认。 |
| [[T2] 自体过滤 bridge 接线：逐源 MoveIt 几何过滤](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/22) | [OFF-07](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/52) | 真实传感器遮挡、附件/线缆覆盖和现场误删率留给 T2 的实测验收。 |
| [[T3] 感知健康接线: perception_valid→零速+锁存 (impl)](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/23) | [OFF-08](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/53) | 物理停止、现场有效覆盖及真实传感器故障响应由 T3/E0/H0 继续验证。 |
| [[T4] 合成传感器仿真: 闭环验证通道 (impl)](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/24) | [OFF-14](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/58) | 真实双源数据、标定、执行反馈及物理故障处理的证据留在 T4/08A 和现场票。 |
| [[T5] 标定工具链：复用现有工具 + AX=YB 输出](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/25) | [OFF-16](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/60) | 实际配对数据、夹具、独立误差和现场标定仍由 T5/T6 承接。 |
| [[T6] 启动自检程序: 矩阵校验+provenance 比对+20mm 验收 (impl)](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/26) | [OFF-17](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/61) | 已知物体精度验收、真实状态覆盖、设备响应和 live 开启属于 T6/H0 后续工作。 |
| [[T7] 标定 SSOT 接线: sensor_extrinsics.yaml 唯一真源 (impl)](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/27) | [OFF-01](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/45) | T7 继续承接完整运行入口/部署验收及未定模式接口；真实外参和现场误差验收由 T5/T6/H0 承接。 |
| [[P0] 双传感器官方驱动接入与版本冻结](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/31) | [OFF-05](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/49) | 固件/序列号、实际时间域、双源同步和长期采集稳定性继续由 P0 现场核验。 |
| [[C0] 控制模块复用与九轴适配](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/32) | [OFF-10](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/54)、[OFF-11](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/55) | 控制方法最终选用、现场任务精度与执行器一致性仍由 C0/08A 及实机验收确认；生产后端/周期定案、目标实机负载与最坏响应上界继续归 01A/01B/C0。 |
| [[E0] 最终命令、后处理与执行保护](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/33) | [OFF-09](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/47)、[OFF-13](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/57) | 实机数值阈值和物理停止不在本票定案；最终滤波/发送接线由 OFF-13，整体责任保留在 08B/E0；真实执行器响应、独立硬件看门狗、承重/制动能力继续阻止 E0/11/H0 的现场验收。 |

## 尚未由离线票解决的事项

- 11/E0/H0：设备及固件身份、真实全轴标定、可信反馈时间、实际通信/驱动反应、独立看门狗、承重与制动能力。Linux vcan 若未运行也必须单独保留缺口。
- 12/08B/01B：真实工具/附件、误差与停止包络，以及最终运行域、逐类阈值、周期和时限。OFF-02 的模型内几何界不等于真机运行域。
- P0/T5/T6/10：设备时间域、双源同步和覆盖、真实外参及独立现场误差验收。
- T7 的调试/正式模式入口、C0 候选方法选择、01B/08B 新数值取舍仍按具体可审阅结果对齐；不重新投票已接受的总体原则。
- H0 受控低速实机验收不拆成“离线验收通过”；M0 删除旧模块仍待替代和相关验收完成。

## 每票完成时留下什么

记录实际代码/配置/模型版本、已复用成果、此次增量、运行命令与结果、失败或跳过项、适用范围及回退方式。把结果回填本票及来源原票；新票完成后重新审视原票剩余门槛，不机械关闭原票，也不机械等待所有现场工作。

本次交付为票据拆分、依赖和执行说明；没有执行这些新票的产品代码、硬件或算法验收。

## 本轮票据核验

已回读 GitHub：18 张新票唯一且归属地图，29 条离线依赖、26 条来源关系与本文一致，完整依赖图无环；19 张来源原票的原有依赖、状态及认领保留。HTML 地图已刷新为 50 张票，OFF 筛选显示 18 张，可认领离线前沿为 6 张。

本段核验的是票据、文档与地图，不是这些新任务的代码或算法验收。
