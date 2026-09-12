# OSCBF 复用优先决策文档

这组文档回答“哪些地方需要先决定，决定后改哪里、凭什么验收”。用户已逐轮确认复用边界、候选验证路线、验收方法与迁移顺序；本地规划已收敛，具体替换及生产后端仍须实测择优。本文不是实施授权清单。

## 阅读入口

- [离线 ticket 执行顺序](OFFLINE-EXECUTION-ORDER.md)：2026-09-12 新拆分的 OFF 离线队列入口，列出推荐顺序、硬前置、验收与原票剩余项。

- [新窗口研究、决策与编码顺序](DECISION-WINDOW-ORDER.md)：原票整体研究、决策及现场阶段的接续顺序；当前离线部分使用上面的 OFF 队列。

- [按独立窗口执行的推荐顺序](EXECUTION-ORDER.md)：实施票顺序、前置条件和新窗口提示词。

- [项目地图](MAP.md)：目标、已知结论、未明确范围。
- [决策前沿与依赖图](FRONTIER.md)：先讨论哪项、哪些仍被阻塞。
- [代码事实、复用职责和原票据接续](REBASE.md)：现状及其影响。
- [复用优先 ADR](../../adr/0005-upstream-reuse-first.md)：记录用户已经给出的架构方向。

## 改动决策索引

| 决策 | 涉及改动 | 状态 |
|---|---|---|
| [确定本项目独有能力与上游维护边界](issues/03-reuse-boundary.md) | AEB-RRT*、项目扩展、上游补丁维护范围 | 决议已发布；实施证据待补 |
| [确定连续障碍观测契约与距离表示](issues/04-safety-observation.md) | 感知输出、静动态覆盖、距离/有效性接口 | 决议已发布；实施证据待补 |
| [确定上游控制接入方式与九轴扩展边界](issues/05-control-adoption.md) | CBFpy/OSCBF、机器人模型、5D 任务适配 | 决议已发布；选型待实测 |
| [确定端到端验证门与故障处理边界](issues/06-validation-contract.md) | 控制后处理、执行端、失败锁存和验收 | 决议已发布；数值/实机证据待补 |
| [确定渐进迁移顺序与旧实现退役条件](issues/07-migration-order.md) | 依赖、接入入口、旧模块及原实施票 | 决议已发布；退役须逐项验收 |

每张待决票都包含问题、推荐/选项、影响文件、决策后的验证门和决议记录要求。答案只在对应票的 resolution 中记录，地图只索引；推荐不是已接受决定。

部署约束见[上位机事实](HOST.md)。观测票第一轮已确认并形成[架构记录](../../adr/0006-continuous-obstacle-observation.md)，第二轮已确认 CPU/PCL 基线路线，双源协作、基础本体过滤和接口交接现已确认，整票本地已解决。

## 已有研究

[九轴冗余与安全优先级研究](research/nineaxis-redundancy-safety-priority.md)：正常避障先利用冗余保持任务，必要时降进给，任务冲突与关键故障分开；行为路线已接受，故障恢复见验收决议，具体求解实现仍需验证，未启用零空间策略或更换求解器。

[环境点云碰撞处理研究](research/environment-pointcloud-collision.md)：解释逐源处理、占据体素、全臂距离、未知区域和控制适配；PCL＋FCL 及环境 OBB 的优先验证路线已接受，自碰撞保留 OBB 已定，实际替换仍需证据。

[控制上游研究](research/control-upstream.md)和[感知上游研究](research/perception-upstream.md)已完成资料核验；版本、许可证及源码引用均在研究文件中。研究完成不代表构建、模型等价验证或目标机器验收通过。本轮仅整理文档，没有运行算法迁移或真机实验。

补充研究：[双传感器协作、冲突与自体过滤](research/dual-sensor-cooperation.md)，回应用户的互补合作目标；两项协作推荐已由用户确认。

## 测试工具诊断交接（2026-09-11）

- [run_all_tests.sh 在 set -euo pipefail 下直接退出](handoffs/10-test-entrypoint.md)：环境加载与双套件结果汇总已实施，验收记录见交接文档。
- [portable 测试容差回归修复（roll-only 参考起点）](handoffs/11-roll-only-tolerance.md)：已修订精度与执行后报告断言，并加入倾斜对照；完整内核套件 148 passed、34 skipped，修订已归档。

入口修复与 portable 测试容差修复均已完成本机验收并关闭；代码合入状态与本机验收分开记录。

## 文档与 tracker 的关系

远程仓库为 [nineaxis_safecontrol](https://github.com/Shiningliu1011/nineaxis_safecontrol)，接续既有 [OSCBF Wayfinder Map — 9DOF LiDAR+depth OSCBF 复用与迁移地图](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/1)。GitHub 是正式 tracker，本目录是仓库内已经发布的可审阅决策与交接材料；本地文件名不是远端 issue 身份。

2026-09-07 已恢复 GitHub 写入并完成同步：PR #29 将窗口顺序合入 `main`，地图 #1 和全部开放原票均已补同步记录，#8、#16–#19、#22、#25 的过时正文已修正。未有证据的票保持开放；创建新票前仍须按[同步与接续](REBASE.md#tracker-同步与后续接续)核对现有 backlog 和依赖。研究的历史分支提交保留原临时路径，当前阅读以本目录为准。

## 后续访谈与落档规则

依 wayfinder 每次最多解决一张非研究票。“确定本项目独有能力与上游维护边界”已收到用户对两项建议的接受答复并记录决议；“确定连续障碍观测契约与距离表示”也已完成逐轮决议；“确定上游控制接入方式与九轴扩展边界”已本地收敛，“确定端到端验证门与故障处理边界”也已逐轮确认并本地收敛，“确定渐进迁移顺序与旧实现退役条件”也已获接受；本地规划阶段完成，下一阶段从可复现基线与执行能力核查开始交接实施。未回答的票保持 open，不能把文档整理请求解释成选项批准。

CONTEXT.md 只维护领域词汇，不存实现选型；已确认的障碍观测术语已写入词汇表。具有长期架构影响的真实取舍确定后再写 ADR，并从决策票链接，避免把每项待办都变成已接受 ADR。

新增[CPU/GPU 部署研究](research/cpu-gpu-deployment.md)：回答模块分工、版本兼容和实测选择；[对照实测与验证准入路线](issues/05-control-adoption.md)已接受，生产控制后端仍待实测定案。
