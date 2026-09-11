# 本轮 report-only Retro

Scope：source-of-truth cleanup、#13 状态同步、explicit-only runtime acceptance、launch-level live fail-fast。用户显式指定 `.agents/skills/retro/SKILL.md`；本任务读取并遵循其 report-only 指令，不将手工加载等同于宿主 `$retro` 分派验收。未 apply 任何 Retro 候选。

## Evidence ledger

日期：2026-09-11（Asia/Shanghai）。起始 `main` / `f16cc35c43f4c3472adaf43b875d64d723460a3e`，干净工作区，无用户未提交修改。测试时同 HEAD 加本轮 diff，精确快照见 `tested.diff`；最终实现 diff 见 `final.diff`。

- `fast-phase2.txt`：文档/注释阶段 fast-check exit 0，62 pure contracts；controller AST 与 HEAD 一致。
- `launch.txt`：真实 LaunchService 调度 + mock Node.execute，7 passed，exit 0；default/sim/shadow 可调度，live 返回 1 且没有 Node.execute 调用。
- `focused.txt`：bridge、launch、hardware contracts，50 passed，exit 0。bridge 使用拒绝 I/O 的替身，真实 backend 构造被禁止。
- `full.txt` / `full.exit`：受安全约束的主包与 portable 回归，准确结果以归档日志为准。`test_final_launch_runtime.py` 因启动真实节点进程而在 collection 前排除，不能宣称无删减完整回归通过。
- `environment.txt`：Python 3.10.12、ROS Humble、JAX/jaxlib 0.6.2、CPU。
- [#13 本轮状态评论](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13#issuecomment-5630878593)：发布时清理尚未提交；评论是 Phase 3 快照，不代表后续 launch 实施状态。重新读取确认 OPEN、正文与本地草稿一致（比较忽略末尾空白）。
- `retro-runtime.txt`：A/B 注入证据不可观察。仓库元数据未修改，不能以 metadata / skills/list 代替行为证据。

## Findings

No new P0/P1. 本轮范围内无新增需要永久环境改动的候选，不为产生结果增加规则或拟议 diff。

观察：README 将旧完整验收绑定到 e4d9a268；CONTEXT 明确 viewer subscriber 与真机目标/containment；controller 仅改注释。final launch 的拒绝由真实调度测试保证，不依赖 CLAUDE 常驻指令。direct hardware bridge guard 保留且 focused 测试通过。

证据局限：不能观察宿主正常请求/显式 `$retro` 的独立注入行为；真实节点进程集成测试按本轮约束未运行。这两项是验收缺口，不是已证明的运行错误。

## Do not change

保留 direct hardware bridge guard：直接节点入口不经过 final launch，单独拒绝不可删除。未来解除 containment 时，两层必须独立审查。保持无 CAN 的 sim/shadow、未知反馈、acknowledgement 不制造健康的门禁；保持 fast-check 的小范围启发式定位。保留控制算法、参数、坐标系、sensor extrinsics SSOT 和现有 QoS，不扩展 Retro，不复制 reviewer 标准到 CLAUDE。

## Source-of-truth conflicts

本轮修复的三处 drift 已消除。旧 #13 评论及上一轮 REPORT / launch-fail-fast.patch 保留其历史时间/版本语义，不能当作当前实现；本轮报告记录实际已应用行为。已有 LESSONS 中“实机”表述无本轮可访问的真实硬件证据，继续只作为历史记录，不证明准入；本轮不扩展修改范围。

## Checked with no finding

本轮改动范围的状态流 ownership、topic/QoS 未变、配置 authority 未变、launch/bridge 双层门禁、reviewer 与 steering 分工、文档版本绑定、已有测试复用与无重复昂贵重跑。

## Not assessed

真实 CAN/设备/标定/watchdog/物理停止与准入；宿主 native `$retro` A/B 行为；被排除的真实进程 launch 集成测试；与本轮无关的感知、控制算法、C++、历史 session 工具成本。缺失证据不记作通过。

下一步仅保留独立验收边界：#13 真正 live 实现、真实硬件准入、受允许环境下补齐 runtime 验收证据。本轮不自动开始。

归档补充：最终 fast-check exit 0；受限主包 351 passed，portable 148 passed / 34 skipped，总 exit 0。实现已提交 `8283a28` 与 `5c6b3b4`，报告归档前工作区干净。复盘未 apply 候选；本报告写入是用户授权的交付归档。
