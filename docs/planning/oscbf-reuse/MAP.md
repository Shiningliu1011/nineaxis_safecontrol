---
title: OSCBF 复用优先地图 — 九轴人机协作安全控制集成
label: wayfinder:map
status: published-tracker-aligned
tracker_target: https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/1
---

## Destination

形成可交接的开源复用与渐进迁移决策：逐层明确直接依赖、薄适配、必要上游补丁与本项目独有扩展，确定感知到执行端的接口和验收门，避免继续重复实现通用基础能力。
这次到达的是迁移路线清晰，不是完成代码替换或真机验收。

## Notes

- 2026-09-09 用户补充：重要的决策票必须向用户展示后再定案。先完成事实研究与可审阅方案，列明剩余取舍、推荐及影响，等待用户明确决定；不得代替用户回答或关闭尚未获确认的重要决策票。已接受的决议不重复征询，例行事实核验和已授权工作继续自主推进。此规则尤其适用于冗余目标与优先级、控制率/延迟预算、不可行/裕度/降级策略，以及新增的安全阈值、控制复用边界与实机准入取舍。

- 2026-09-07 用户明确约束：成熟开源框架优先直接复用，必要时在其基础上修改完善。原 ChatGPT 讨论仅作为候选线索，采用前必须核验上游源码、兼容性与许可证。
- 本文已发布到仓库并通过 PR #29 合入 `main`；GitHub #1 是正式地图，现有开放票已补同步记录，关键过时正文已修正。本地决策文件是仓库记录，不另冒充正式 issue 身份；新增票仍须先核对 backlog，再绑定原生 sub-issue 与 blocking。
- 默认保留现有九轴 1P8R、5D 工具轴路径跟随、base_link 固定 Y-up、标定 provenance 与失败锁存等已定需求；不把当前实现等同于目标已实现。
- 独有能力与维护边界已由用户确认，见 Decisions so far；具体选型仍按前置决策推进。
- 每次继续使用 wayfinder；涉及取舍使用 grilling 和 domain-modeling；外部事实用 research。每次最多解决一张非研究决策票。
- 优先级规则：现有上游可满足→直接依赖；接口不同→薄适配；存在具体缺口→最小 fork/补丁并记录升级策略；只有无可用上游且缺口被证实时才保留自研。上游研究代码不自动等于工业成熟或本机实时性达标。
- 上述优先级已由用户补充为效果择优：成熟开源优先评估，已有实现可凭比较优势保留；替换须有验证与收益依据，详见[维护边界补充决议](issues/03-reuse-boundary.md#用户补充决议按实际效果择优保留已发布)。自碰撞保留 OBB 的阶段决议见[控制接入](issues/05-control-adoption.md#阶段决议自碰撞保留现有-obb)。
- 历史票据处置与当前事实见 [重梳说明](REBASE.md)，票据及阻塞视图见 [决策前沿](FRONTIER.md)。这些是草案辅助材料，不是 map 内复制的决策答案。

- 文档入口与状态约定见 [决策文档导读](README.md)。已确认的复用原则见 [通用能力优先复用上游框架](../../adr/0005-upstream-reuse-first.md)。

- 部署平台由用户指定为当前 Ubuntu 22.04 笔记本，允许 CPU/GPU；[本机事实与验证边界](HOST.md)。

- [CPU/GPU 准入路线](issues/05-control-adoption.md)已获用户阶段确认；控制接入已本地收敛，生产后端待实测选择。

## Decisions so far

- [[T0] spec 与参数文件坐标系/术语修订 (impl)](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/20#issuecomment-5618602929)：剩余 J1 用词与双源模板注释已提交并通过双轴审查；参数解析结果不变，已推送远端 main（提交 5b23222）。

- [[04] portable 测试容差回归修复（roll-only 参考起点）](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/11#issuecomment-5617873811)：测试精度与起点/执行后报告语义已修订，倾斜对照及完整内核验收通过（148 passed、34 skipped）；修订已归档。

- [[03] run_all_tests.sh 在 set -euo pipefail 下直接退出](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/10#issuecomment-5616759377)：入口修复及本机验收完成，双套件如实汇总退出码；内核既有失败保留原票，代码尚未合入。

- [[07B] 选定冗余目标与优先级](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/16#issuecomment-5614751672)：外障下利用九轴协调保持末端精度；满足精度及经验证余量时优先保持进给，必要时降速/暂缓，无法兼顾则任务冲突处理；数值与实现交后续验证。

- [[07A] 冗余策略对比：9-DOF 对 5D 工具轴任务](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/5#issuecomment-5614572282)：三候选六组局部对比与复用审计完成；中点/现有梯度未改善最小限位余量，生产目标与优先关系交后续人审。

- [[B0] 可复现基线与证据目录](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/30#issuecomment-5585757007)：已固定同机代码/环境/输入快照及恢复交接；主包250通过，内核146通过/34跳过/1容差失败，入口故障与依赖缺口留证。

本轮复用与迁移规划已收敛并发布；实施证据仍待逐票完成。此前已定的需求从原票读取，不复制答案：

- [选定目标跟踪语义与验收标准](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/14)：保留既有任务与验收约束作为迁移基线。
- [选定规范世界坐标系与标定策略](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/15)：保留规范坐标系及标定来源约束。

- [核验 OSCBF 上游能否承接九轴控制与碰撞模型](issues/01-control-upstream.md)：可复用通用框架；5D、机器人接口与版本兼容需要明确适配。

- [核验双传感器感知与环境距离链的复用方案](issues/02-perception-upstream.md)：官方驱动有支持证据；过滤与可选距离场需核发行版，观测覆盖仍需验证。

- [确定本项目独有能力与上游维护边界](issues/03-reuse-boundary.md)：保留研究插件并接受受控的最小上游补丁维护；决议已发布。

- [确定连续障碍观测契约与距离表示](issues/04-safety-observation.md)：双源连续观测、MoveIt 基础本体过滤与 CPU/PCL 距离基线已定；验证门交接后续票，本地已解决。

- [确定上游控制接入方式与九轴扩展边界](issues/05-control-adoption.md)：OBB 复用、几何查询与九轴适配分层、统一求解及择优验证路线已定；本地已解决。

- [确定端到端验证门与故障处理边界](issues/06-validation-contract.md#resolution-comment已发布为仓库决策记录)：任务与故障恢复、工况预算、实机松弛边界和实际执行验收已定；数值证据仍为启用前置。

- [确定渐进迁移顺序与旧实现退役条件](issues/07-migration-order.md#resolution-comment已发布为仓库决策记录)：分段对照、受控切换、停止后回退及验收后退役已接受；决议已发布。

## Not yet specified

当前规划范围无未归属决策。实施中才能获得的兼容补丁、数值预算和逐文件退役证据，已交接至[迁移决议](issues/07-migration-order.md#规划收束与剩余证据)及其引用的原票；这些原票继续保持开放。若证据改变架构前提，再重开对应决策。

## Out of scope

- 本轮不实施驱动接线、内核替换、标定、实机运行或删除现有代码；交给后续 implementation backlog。
- 通用 SLAM、GPU 建图平台升级、加速度/力矩控制重构不是默认目标；若复用可行性揭示必要条件，再显式重定范围。
- 不以论文 kHz 指标、最近点距离或纯几何仿真推断本机人机协作安全已获验证。
