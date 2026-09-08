# 新窗口研究、决策与编码顺序

本文件取代 EXECUTION-ORDER.md 作为“下一窗口做什么”的入口。旧文件保留为实施阶段参考。本轮复用与迁移原则已经确认；2026-09-07 已对齐 GitHub 正式地图、原票和本表缺失票。票据同步不代表研究、实测或编码前置已经齐全，不能将文档合并或 `ready-for-human` 标签当成通过证据。

## 使用规则

一张正式 ticket 一个窗口。默认按下表串行推进；已完成的研究/决策只核对和引用，不重新投票。研究结论、用户决定和编码验收分别记录，不能相互代替。沿用原 ticket 身份，标题与正文可根据已接受决议修订；新增票已在确认无同义 backlog 后建立，后续仍不得重复创建。

前置不足时，在该窗口完成不依赖缺失证据的工作，写出剩余阻塞；下游依赖该证据的窗口不得伪称通过。后续实现改变测量对象时，回到原票补验。不是所有窗口都需要用户选择：事实由研究回答，已有明确规格的实施工作直接按规格验收。

每票必须明确开源复用结论：可保留的现有实现、可直接使用的上游及版本/许可证、必要薄适配或最小补丁、替换收益与验证依据。通用能力已有成熟实现时优先复用；已有实现效果更好则保留，不能为了迁移而替换。无需新增算法的文档/修复票注明不适用及理由即可。

首次前置与后续复验分开：#16 依据 #5/#14 收敛目标，交给 #32 后再由 #6 实测；#17 输出预算供 #18 汇编/裕度决策，后续变化回原票补验；#22 提供过滤结果给 #21。不能反向设置循环阻塞。GitHub 原生依赖表达整票完成门槛，部分不依赖缺失证据的工作可先进行。

## 推荐窗口顺序

| 顺序 | 本窗口 ticket / 原票链接 | 研究或决策内容 | 何时允许编码 |
|---|---|---|---|
| 0 | [正式地图与原票对齐](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/1)（2026-09-07 已同步；后续只核对漂移） | 将已接受决议关联原地图；修正球模型、强制移植等过时方向；核对全部原票归属、依赖和状态，补齐缺失实施票 | 只整理 tracker 和文档；不改运行代码 |
| 1 | [可复现基线与证据目录](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/30) | 固定当前含未提交改动的代码快照、依赖、配置、模型、已知失败及交接格式；确认新窗口起点 | 可编写离线记录与回放辅助脚本，不改变控制行为 |
| 2 | [真机执行链路：SocketCAN 后端与参数注入](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13) | 先核查模式、单位、反馈、独立超时与停止能力；将设备未知项列出，区分离线验证和运动操作 | 接口与设备协议核验后，可实现离线/vcan 部分；不凭此开始真机运动 |
| 3 | [spec 与参数文件坐标系/术语修订](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/20) | 核对已接受的 base_link、Y-up、标定权威、5D 语义；划清文档修订与运行参数修改 | 已定语义可直接修订；运行默认值变化须有对应验证 |
| 4 | [标定 SSOT 接线](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/27) | 核对外参来源、provenance、TF 发布和失败行为 | 输入/输出与失败契约明确后即可实现 |
| 5 | [标定工具链](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/25) | 按已定复用原则比较可用工具与离线适配，补足设备和数据证据；不默认强制 ROS2 移植 | 选型及输出契约明确后实现适配；实际标定缺失继续作为准入阻塞 |
| 6 | [双传感器官方驱动接入与版本冻结](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/31) | 核验精确设备型号、Ubuntu/Humble 版本、输出字段、时间来源、许可证与固定版本 | 兼容及接口核验后接入，保留设备未到位的验证缺口 |
| 7 | [感知时间同步与延迟模型](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/7) | 研究逐源时钟、采集时间、观测年龄、变换时刻及误差预算；定义测量方法 | 可写诊断/测量代码；最终有效期不能凭经验伪定案 |
| 8 | [自体过滤 bridge 接线](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/22) | 将旧球过滤方向更新为已定 MoveIt 基础过滤；确定逐源处理、误删及错误 TF 验收 | 契约明确且前置可用后接入；不重写通用过滤算法 |
| 9 | [自碰撞/障碍几何模型核对](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/8) | 验证自碰撞 OBB、环境全臂/TCP/附件包围；核验占据体素/FCL 距离、接触和导数误差 | 先写隔离验证原型；几何/导数契约和误差证据明确后才接正式链 |
| 10 | [tracks 契约升级](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/21) | 明确连续几何与速度增强分工，来源、时间、未知覆盖、容量与导数接口 | 前置观测/几何契约确定后可实现并离线对照 |
| 11 | [感知健康接线](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/23) | 落实逐源有效性、覆盖丢失、空帧、故障锁存与恢复；沿用已定原则 | 有明确状态转换和输入契约后可实现 |
| 12 | [合成传感器仿真：闭环验证通道](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/24) | 确定回放/合成场景覆盖什么，哪些必须实际传感器或执行器验证；复用现有工具 | 场景和证据边界明确后可写验证通道，不把合成数据当实机证据 |
| 13 | [冗余策略对比：9-DOF 对 5D 工具轴任务](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/5) | 复用已有研究，补本项目所需策略对照与证据缺口，区分普通权重和严格优先关系 | 允许隔离比较原型，不自行启用生产零空间策略 |
| 14 | [选定冗余目标与优先级](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/16) | 只决策剩余具体目标、任务容差、优先关系表达及验收指标；九轴/5D、冗余优先和必要降进给不重问 | 用户确认真实剩余取舍后，可交接控制实施票 |
| 15 | [控制模块复用与九轴适配](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/32) | 对照现有 CBFpy/qpax/JAX 与 OSCBF 模块缺口；确认最小适配，运动学替代按效果择优 | 输入契约、优先关系和比较标准明确后编码；先回放，不启用真机 |
| 16 | [CBF/QP 可行性实证测量](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/6) | 测冗余可解、降进给可解、任务冲突及关键故障；区分不可行/数值失败/超时 | 可写测量原型；未经接受的安全松弛结果不得进入实机 |
| 17 | [19.4ms 回归剖面与成本分布测量](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/3) | 更新目标版本剖面，比较 CPU/GPU 同精度、预热、拷贝、组合负载和尾延迟；历史标题不是当前结果 | 可写测量与隔离优化原型，优化进入正式链前补正确性验证 |
| 18 | [选定控制率/延迟预算与实现策略](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/17) | 引用已接受历史口径与工况推导方法，结合新测量选择适用周期、后端和实施策略 | 有证据且剩余取舍确认后改对应配置；实际制动未测时明确实机预算仍暂定 |
| 19 | [选定不可行/裕度/降级策略](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/18) | 结合几何、可行性和时延证据选择约束分类、数值容差、裕度与汇编；不重问实机不放松安全约束、故障停止原则 | 决策与适用工况明确后编码；涉及真实停止能力的数值不能提前宣称通过 |
| 20 | [最终命令、后处理与执行保护](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/33) | 明确滤波/限幅/积分后的候选校验、反馈偏差、命令期限、独立看门狗及停止通路 | 输入/失败契约齐全后编码，先离线与台架故障注入；复验受影响时延和可行性 |
| 21 | [更新 tracking_evaluator 指标定义与阈值](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/19) | 沿用已定任务语义，核对指标、阈值来源和报告真实性 | 指标与适用阈值明确后实现；可提前做不依赖新阈值的部分 |
| 22 | [启动自检程序](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/26) | 汇总标定、状态、覆盖、后端、执行保护与工况门；核验旧标题数值的来源和适用范围 | 各准入条件明确后实现；缺失证据应阻止相应模式启动 |
| 23 | [受控低速实机验收](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/34) | 先确认设备能力、试验条件与操作边界，再测实际响应、停止距离、动态障碍和恢复 | 本票以验收为主；仅在对应准入与用户操作授权满足后运行硬件 |
| 24 | [旧模块逐项退役](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/35) | 根据收益、替代验收、调用方迁移、回退复现决定具体删除清单 | 条件满足后在可审查变更中逐项删除，保留更优现有实现及独立验证基线 |

## 基线阶段就应核对的已有修复票

以下仍各开独立窗口，但不必等主队列末尾：由基线窗口确认缺陷仍存在后，阻碍测量的修复先做。它们不需要重新讨论架构。

- [run_all_tests.sh 在 set -euo pipefail 下直接退出](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/10)：缺陷复现且修复范围明确即可编码。
- [主包测试套件修复](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/9)：先修有明确语义的测试问题；性能断言等依赖最终预算的部分保留阻塞。
- [portable 测试容差回归修复](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/11)：先核精度与误差来源，再确定合理断言，不为通过随意放宽。
- [配置一致性清理](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/12)：可先盘点；涉及 alpha、安全裕度或运行行为的修改等相关决议与验证。

## 什么时候开始编写程序代码

不需要等所有 ticket 决策结束。每个模块单独经过以下门槛即可编码：相关原则已接受，接口/输入输出及失败行为明确，前置事实足够，验收方法可执行。真正新增的目标或取舍再由用户决定，例行实现细节由执行者解决。

第一批可开始的是基线记录、离线测量脚本和复现后的测试工具修复；标定 SSOT、官方驱动和基础过滤等在各自接口核验后即可进入实现。控制接入须等观测/几何接口和具体优先关系收敛。生产参数及后端选择须等对应测量与决策。

研究原型可以在产品决策前编写，但应隔离、可丢弃、不持有真机命令发布权。开始编写代码、允许合入代码、允许真机运行是三个不同进度；实机必须另满足执行保护、观测覆盖、工况与停止能力准入。测量导致预算/裕度修订时回到原票复验，不虚构一次串行执行即可证明所有工况。

## 每个新窗口的提示词

```text
本窗口只研究并收敛【ticket 名称 + URL】。
项目目录：/home/lsn/robot/robot_safecontrol。
先读 docs/planning/oscbf-reuse/DECISION-WINDOW-ORDER.md、MAP.md、相关已接受决议，以及 GitHub 本票正文、评论和依赖；读取前一窗口交接，核对实际代码起点和未提交改动。
使用 wayfinder 梳理本票剩余决策；需要外部事实时使用 research，需要取舍时使用 grilling。已经接受的九轴/5D、OBB、双源协作、复用择优、故障和恢复规则直接继承，不重复提问。
区分：已决定、需要研究/测量、需要用户选择、可以编码。先完成事实核验，只提出尚未解决的取舍并给推荐。不要将标签 ready-for-human 当作前置证据齐全。
本窗口默认研究和决策，可做隔离测量原型；不自动进入产品代码实施或实机运行。收敛后给出明确的“可以开始编码/尚不能编码”结论及原因、实现范围、前置、验收和回退条件，等待我的实施指令。
将交接写入 docs/planning/oscbf-reuse/handoffs/ 下以本票身份命名的文件；记录已完成证据、用户原始决定、剩余缺口及下一窗口。不要自动启动下一票。
```

要在同一窗口进入编码，接着发送：“按本票已确认范围实施，完成必要验证并更新交接；不自动启动实机。”不同窗口若并行编码应使用独立 worktree；当前未提交改动必须通过基线交接带入，不能假设远端 main 已包含本地工作。
