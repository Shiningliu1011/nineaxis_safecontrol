---
name: retro
description: "仅在用户明确调用 $retro 或指定本 skill 时，基于 coding session 的真实证据提出 agent 环境的永久改进；默认只提案。"
disable-model-invocation: true
---

# Retro

只在用户主动调用时执行。普通 coding、review、session 结束或发现错误均不触发。
目标是改善下一次工作的环境，不是评价 agent 表现或复述聊天。

## 权限与边界

默认只读，输出候选与拟议 diff；不修改文件、tracker、配置或 Git 状态。
只有用户明确 apply 指定候选后，才实施获批的非硬件/非安全环境改进。
应用前重新核对当前 HEAD、工作区和证据，保留用户已有修改。
真实硬件、live 或安全行为发现只能形成独立实施任务提案：
即使用户说“apply retro”，也不能由本 skill 修改这些行为。
任务提案不等于创建 GitHub issue；外部写入另需明确授权。
不连接设备，不执行 live/CAN 路径，不运行校准、使能或自动重连脚本。
sim/shadow 标签本身不能证明无 I/O；先检查调用链，包括 polling。
不降低门禁、放宽测试或使用仿真/command 数据伪装真实反馈以消除失败。

## 建立 evidence ledger

先定位 Git 根目录，记录 HEAD、branch、工作区状态和本次 session 范围。
检查 staged/unstaged diff、相关未跟踪文件及范围内的 commit。
从实际可访问的 session/tool log、命令输出、测试报告、runtime snapshot、
配置 provenance、issue 最新状态/评论和 handoff 读取原始证据。
只读取本任务相关日志；脱敏凭据、设备身份和无关个人信息。
每项证据记录路径/行号或 URL、时间、commit/dirty 状态、命令、退出码、
环境和适用范围；未知字段写 unknown，不补造。

聊天用于定位证据，不能单独证明重复错误、昂贵调用或当前测试失败。
分开记录：本次观察、历史证据、推断、尚未验证的风险。
缺失 session 日志时仍可审查仓库，但必须说明不能归因于本次 session；
不把没有访问权限当作没有问题，也不自动安装日志工具。

优先复用同一代码版本和环境下已有检查结果。
必要补查先确认无设备/写入副作用；不默认重跑完整或昂贵测试。
记录每个搜索/检查解决了什么未知，避免无新假设的重复调用。

## 阅读与导航

阅读 CLAUDE.md、CONTEXT.md、LESSONS_LEARNED.md、docs/ONBOARDING.md、
docs/agents/domain.md、docs/agents/issue-tracker.md 及相关 docs/adr/。
检查 run_all_tests.sh、setup.cfg 和以下 source of truth 的相关实现：

- ROS transport：src/robot_safecontrol_moveit/ros_conventions.py。
- 机器人身份/关节顺序：src/robot_safecontrol_moveit/robot_spec.py。
- 共享轨迹变换：src/robot_safecontrol_moveit/oscbf_trajectory.py。
- 生产 OSCBF base profile：config/oscbf_controller.yaml。
- 坐标语义：CONTEXT.md、ADR 0002 的 canonical base_link / Y-up 约定。

按实际变更再读取相应模块、测试、配置和 ADR，避免遍历整仓库。
若存在 CODING_STANDARDS.md 或 scripts/agent_check.sh，读取其适用范围；
它们不是本 skill 的安装前置条件。不要凭文件存在推断检查已执行。

## 必查类别

逐类检查，但只对有证据的 finding 展开：

- source-of-truth drift：CLAUDE、README、GitHub issue、handoff、
  测试结果和代码是否冲突；比较版本/时间/环境，不机械采用最新文字。
  GitHub 是当前 tracker；旧 issue body 中的测试结果可能只是历史。
  ADR 的目标决策与实现进度分开记录；目标尚未落地不等于 ADR 失效。
- navigation：找模块、配置、ADR、测试或 ticket 的反复成本及缺失入口。
- automated checks：是否缺 regression/contract/static/script 检查；
  先找已有覆盖，并判断失败发生在被测模块还是入口接线。
- ROS transport semantics：topic、QoS、remap、callback group、状态和
  命令 publisher/subscriber ownership；raw topic 创建实体，resolved
  topic 用于验证/诊断，是否重复 remap。
- configuration authority：生产 YAML、launch override、portable/offline、
  legacy profile 是否混淆，实际加载的来源能否证明。
- safety/control contracts：publisher、timer、CAN/live 启动前门禁；
  sim/shadow/live 区别，真实反馈 freshness/watchdog 和失效行为。
- coding standards：哪些语义需要 reviewer 判断；不要把 reviewer
  检查放进常驻 steering，也不复制 canonical modules 的架构事实。
- steering-file hygiene：CLAUDE/AGENTS 的过期状态、重复、no-op、
  review-only rule；高波动 ticket/测试状态优先改成 tracker/test pointer。
- tool economy：依据 tool evidence 找反复昂贵搜索或测试。
- information access：缺日志、runtime snapshot、配置 provenance 等
  是否导致猜测；诊断应来自实际运行数据。
- lessons：仅沉淀非显然、可迁移且有证据的经验；检查已有 lesson，
  保留适用条件，不把单次参数结果推广为所有 profile 的常量。

## 选择永久解决层

每个 finding 按以下顺序选择最早足够解决问题的一层：

1. 代码/接口设计消除失败模式。
2. 自动化 test/check。
3. 诊断与信息访问。
4. Reviewer coding standard。
5. Navigation pointer。
6. 最后才是常驻 CLAUDE.md / AGENTS.md。

解释较早的层为何不适合或已足够覆盖。
不能因单次错误增加一句“以后注意”，也不重复已有测试保证的规则。
portable OSCBF 控制内核保持无 ROS 依赖；改进不能把 ROS 检查移入内核。

## 输出

使用用户语言，先写 scope、HEAD、证据局限和已执行/未执行的检查。
候选按 P0、P1、P2 排序，同级按影响与证据强度排序；没有候选就直说。

- P0：真实硬件/安全边界失效或严重不可恢复风险；独立任务提案。
- P1：反复正确性、契约或工作流失败，需要永久反馈机制。
- P2：导航、文档卫生和工作效率改进。

每个候选使用稳定 ID，并包含：
Evidence；Failure mode；Permanent change；Target；Validation；
Why this layer。Evidence 区分观察与推断，Validation 给失败/成功判据。
拟议 diff 只针对候选明确范围，不自动执行。

额外输出：
- Do not change：已有机制正确防住的问题及其证据，不重复加规则。
- Source-of-truth conflicts：冲突来源、版本/时间、具体事实、
  已知结论或仍缺的验证；无冲突写 none。
- Checked with no finding：有足够证据检查过的类别集中一行列出，
  不生成空模板。
- Not assessed：证据不足/不可访问的类别与原因，不混入 no finding。

最后给最多三个优先候选和各自独立的实施边界。
CLAUDE.md 瘦身作为单独提案；不夹带 sensor extrinsics SSOT、
全面 QoS 统一或 reference_lead_m 语义实现。
