# First Retro — report only

> 归档说明：本报告保留 2026-09-11 验收时点的 HEAD、未提交状态及结论。三份文档随后已在 `c3b3f186038ea2c8374f22959f1d3844eb3e8987` 提交并推送；本次按用户要求归档报告及相关测试日志。文中的临时目录是原始证据位置，随附文件已可跨机器读取。`skills-list.json` 仅保留 Retro 条目，其他运行时技能信息未归档。launch diff 仍为未应用提案。GitHub 评论前后快照未随附，相关进展可通过报告中的公开评论链接读取。

Scope：ec8fd9e（containment）、9a483d7（Retro）、cbcec40（fast-check）、e4d9a26（reviewer standards），加本轮三文档 diff。HEAD `e4d9a268c0dee49455c0e397c4910958cc166032`，main；开始 clean，结束仅 CLAUDE.md、README.md、docs/ONBOARDING.md 未提交。2026-09-11，Linux / ROS Humble / Python3.10.12 / JAX0.6.2 CPU。

本次由用户明确指定 `.agents/skills/retro/SKILL.md` 后读取并依其流程执行，非自动触发。报告不 apply；此前文档修改及 #13 评论属于用户单独授权的 Phase 2/4，不是 Retro 自动修改。

## Evidence ledger

全部本地证据位于 `/tmp/nineaxis-validation-20260911/`，未上传且可能被系统清理。

|证据|时间/版本/状态|命令与结果/适用范围|
|---|---|---|
|evidence.json、四项 commit diff、当前 final.diff|2026-09-11；上述 HEAD，测试前 clean/文档后 dirty|git status、rev-parse、log、show、diff；exit0；仅本轮范围|
|fast-before.txt / fast-after.txt|同 HEAD，修改前/后|bash scripts/agent_check.sh；exit0；compileall、6项 shell syntax、diff、62 pure contracts PASS|
|focused.txt|同 HEAD，修改前|source ROS和install后运行指定三个测试；49 passed，exit0；禁止 backend 的替身保护|
|full.txt|同 HEAD，修改前；CPU|ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=217 bash run_all_tests.sh；351 passed；148 passed,34 skipped；总exit0。既有禁用/optional skip；newaxis缺失。未取得每个 skip 的运行时 reason 明细，不把skip计为通过|
|issue13-before.json / issue13-after.json|历史评论2026-09-07/08，本轮2026-09-11|gh issue view 13 --json ...（exit0）；默认--comments命令因旧CLI projectCards GraphQL失败，改显式字段后读全评论|
|skills-list.json、SKILL.md:1、agents/openai.yaml:5|codex-cli0.153.4，本轮|实际 app-server skills/list forceReload 返回 retro scope=repo enabled=true errors=[]；yaml布尔值已解析；仅 loader discovery、metadata 证据|
|proposal-focused.txt、launch-fail-fast.patch|临时拟议代码，不属于工作区 diff|真实 LaunchService、mock Node.execute；7 passed，exit0；无节点进程。早先探针未正确注入LaunchConfiguration，已修正测试夹具，非产品失败|
|hardware_bridge.py:61/67/92/114/139；test_hardware_bridge.py；test_launch_structure.py|当前HEAD|只读模式、live先拒绝、shadow唯一控制订阅、无健康反馈、ack门禁；源码+49 focused验证|
|CONTEXT.md:19/93；viewer.py:148；ONBOARDING当前diff|当前HEAD及文档dirty|viewer只有订阅；CONTEXT仍称其发布；真机目标缺实现限定|
|CODING_STANDARDS.md、agent_check.sh、setup.cfg、run_all_tests.sh|当前HEAD|完整阅读；前者reviewer动作/pointer，后者小型纯合同检查，明确不替代完整回归|

也已阅读 CLAUDE、README、ONBOARDING、CONTEXT、LESSONS_LEARNED、domain/issue-tracker流程、相关ADR、ros_conventions、robot_spec、oscbf_trajectory及生产YAML相关入口。未穷举无关产品实现，未访问此前完整会话日志；不从聊天推断重复工具浪费。

## P0

没有发现当前 containment 的新 P0 regression。49项focused、当前源代码与完整回归支持限定范围结论；不代表后端或物理安全验收。

## P1

没有足够证据提出新的 P1。真实 backend/反馈/watchdog/协议/制动等仍是 #13 的既有未完成工作，不复制成新票，也不以本轮绿色测试将其销项。

## P2

### RETRO-001 — 领域词汇仍描述旧状态发布者与未实现真机路径

- Evidence（观察）：CONTEXT.md:19–20 写“被控对象与查看器发布”；viewer 当前仅订阅。CONTEXT.md:93–94 把 CAN 收发链路写成无实现限定的定义，和本轮 ONBOARDING 区分目标/当前的口径不一致。
- Failure mode（推断）：后续 agent 经 canonical pointer 读取领域文档时可能重新采用旧 ownership 或误认为 live 可用。
- Permanent change：单独修订这两个领域定义，区分目标与当前；不重复扩充 CLAUDE。
- Target：CONTEXT.md 的“状态流”“真机执行端”，仅提案。
- Validation：逐一对照 viewer/plant/replay 的实际实体与 final launch/run_demo 配置；定义不再把viewer列为publisher，CAN链路标为未来目标；git diff --check通过。
- Why this layer：产品代码及测试已正确；增加代码/文本扫描测试无法合理验证自然语言术语。现有 reviewer standards 已要求核对，直接修正 canonical 领域文档最小且足够。

拟议 diff（未应用）：
```diff
--- a/CONTEXT.md
+++ b/CONTEXT.md
@@
-被控对象与查看器发布、控制器订阅的关节状态话题流（BEST_EFFORT、深度 20），约定统一在共享模块中。
+控制器订阅的关节状态话题流。run_demo 仿真闭环由被控对象持续发布，查看器只订阅；默认 launch 未启用被控对象时，过渡回放可发布该状态话题。各实体当前 QoS 与配置差异见 docs/ONBOARDING.md，共享约定见 ros_conventions.py。
@@
-订阅命令流（/oscbf_command）、经安全网关校验后换算为 DrEmpower CAN 帧发送给电机，并把反馈帧解码换算后发布状态流（/mujoco_joint_states）的 ROS 2 节点。
+目标是订阅命令流（/oscbf_command）、经安全网关校验后发送 CAN，并把真实反馈转换为状态流。当前 hardware_bridge 仅提供 containment：sim 无控制 I/O，shadow 记录且不收发 CAN、不发布硬件状态，live 禁用；真实链路实现和准入见 GitHub #13。
```

### RETRO-002 — live 拒绝的启动语义只覆盖单节点

- Evidence（观察）：final launch 无全局退出处理；ROS Humble ExecuteLocal 仅记录非零退出并发 ProcessExited，默认不触发 Shutdown。bridge 自身 fail closed 正确。
- Failure mode（推断）：用户指定live后看到其他仿真/控制进程继续，容易误读启动状态。没有证据表明当前因此能打开CAN。
- Permanent change：单独审阅方案B：在任何 Node/TimerAction 前用 OpaqueFunction 拒绝live，让 LaunchService返回1。
- Target：launch/mujoco_transition_final.launch.py 与 tests/test_launch_structure.py；完整拟议diff见 launch-fail-fast.patch，未应用。
- Validation：真实LaunchService下live返回1且Node.execute调用0；默认/sim/shadow返回0并继续调度，所有Node.execute均mock；临时提案7项测试通过。
- Why this layer：这是入口设计问题；在启动入口解决并加接缝测试即可，不能用更多CLAUDE警告代替，也不恢复live。

### RETRO-003 — 显式限定的运行时行为验收尚缺桌面端证据

- Evidence（观察）：repo-local loader发现成功，两项metadata正确；当前会话可用skill目录未列retro；app-server响应不回传policy，桌面proxy不可用。
- Failure mode（推断）：仅看到文件或loader条目就宣布“普通请求不触发、$retro原生注入成功”，会把部分验证当作完整验收。
- Permanent change：在新的目标桌面会话做一次普通请求/显式$retro对照，保存注入/调用证据；失败时先查会话缓存或宿主发现路径。
- Target：验收记录与运行时诊断，无仓库代码拟议diff、无全局skill复制。
- Validation：普通请求无Retro注入/执行；显式请求加载准确repo路径并遵守report-only；仅selector可见不够。
- Why this layer：当前metadata无错误证据，缺的是实际宿主观察；修改技能或steering不能填补信息缺口。

## Do not change

- 保留live无条件拒绝、sim无I/O、shadow无CAN、feedback unavailable与acknowledge门禁；不把已验证的规则再复制进CLAUDE。
- 保留fast-check的小型启发式定位；62项纯合同加语法/compile/diff适合快速反馈，ROS接缝及JAX全量由本轮独立命令验证；不自动把全套昂贵检查塞进去。
- CODING_STANDARDS定义reviewer检查动作并链接canonical来源，没有理由把表格移入常驻steering。
- 不修改OSCBF算法/参数、reference_lead_m、sensor extrinsics SSOT、全面QoS、ADR目标决策或任何真实硬件能力。

## Source-of-truth conflicts

- CLAUDE旧测试失败、live=CAN发送及重复架构：本轮三文档diff已清理。
- README待实施/未发布与ONBOARDING旧viewer/硬件链路：本轮已对齐。
- #13历史评论被本轮[containment进展](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13#issuecomment-5630688954)明确supersede；保留历史，issue OPEN。
- CONTEXT两个当前定义：仍存在，RETRO-001；本轮不在允许编辑范围。
- LESSONS_LEARNED.md:71/103 的“实机”历史措辞缺可访问的物理验收证据；不判断历史事件真假，也不作为当前真机准入证据。
- QoS depth20/5差异客观存在，已如实记录；未证明构成新功能缺陷。

## Checked with no finding

containment入口/反馈/模式合同、fast-check范围、reviewer与steering分离、当前三文档的已修正事实、共享identity/轨迹/profile入口本轮未变、既有ADR目标与当前实现的区分。

## Not assessed

真实CAN/backend qualification、设备映射/传动/标定/独立watchdog/物理停车、真实受控低速验收；桌面原生$retro注入和普通请求不触发的端到端行为；全部历史会话工具成本与实机记录；全面感知/QoS/配置审计；未单独重跑C++基准或硬件脚本。可访问证据不足不等于没有风险。

优先候选：RETRO-003补宿主验收证据；RETRO-001独立领域文档修订；RETRO-002独立启动语义审阅。三者均未apply。
