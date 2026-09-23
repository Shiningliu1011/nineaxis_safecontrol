# robot_safecontrol — Codex 工作约定

## 执行与优先级

在系统、开发者指令和实际工具权限允许的范围内：当前任务的具体要求及已有授权 > 本项目和更具体目录的 AGENTS.md > 全局默认约定 > 外部通用 Skill。

只读检索、本地草稿、可恢复编辑、已核实隔离的本地测试和相关失败修复可自主推进。已有授权不重复确认；未经授权的不可逆或难以撤回的操作、对外发送、发布、付费、扩大共享范围或真实硬件动作需要明确授权。关键事实无法查明时完成其余可做部分，再报告具体缺口。

从本次问题相关的入口、配置、调用点和测试开始。修改任务持续到相关验证与交付，普通查询不扩大为全仓接手、强制访谈或成套文档创建。保留用户未提交改动，提交和远程操作按具体授权处理。

## 项目入口与按需资料

ROS 2 Humble / MoveIt2 / MuJoCo 的九轴仿真闭环，控制内核位于 `portable_oscbf/work`。

- 架构、入口和状态流 ownership：查 `docs/ONBOARDING.md` 对应章节。
- 控制内核依赖：查 `docs/modules/portable_oscbf.md`；生产配置和入口说明：查 `config/oscbf_controller.yaml` 与 `docs/PROJECT_GUIDE.md`。
- 领域词汇、坐标语义和既有设计：查 `docs/CONTEXT.md` 与相关 `docs/adr/`，区分设计目标与实际完成进度。
- ROS topic/QoS、关节身份/顺序、共享轨迹变换分别以 `src/robot_safecontrol_moveit/ros_conventions.py`、`robot_spec.py`、`oscbf_trajectory.py` 为规范来源。
- 参数与接口教训按主题检索 `docs/LESSONS_LEARNED.md`；代码审查参考 `docs/CODING_STANDARDS.md`。
- 需要 tracker 或领域文档操作时才读 `docs/agents/issue-tracker.md`、`docs/agents/domain.md`。
- 实机评估参考 `docs/real_robot_runbook.md` 与当前实现。`docs/agents/legacy/CLAUDE.md` 是历史导航资料。

## 文档维护

- 面向读者的文档统一放在 `docs/`；根目录 `README.md` 只提供项目简介和文档索引。文档总目录见 `docs/README.md`，编排要求见 `docs/documentation.md`。
- 文档按主题保持简短。导读只写概要和目录，详细内容链接到对应主题页面；新增或移动页面时检查相对链接。
- 修改功能时必须在同一次工作中修改相关文档。检查运行步骤、配置、接口、当前能力与验收说明，交付时说明文档修改和验证结果。
- 讨论方案按正常流程进行；方案确认并执行时同步修改文档。维护人员每月核对近期功能变更与文档，补充遗漏内容。

## 构建和验证

- 修改 AEB C++ 插件后使用 `bash build_aeb_moveit.sh` 构建，再重新加载环境并重启本任务的 launch；运行中的 move_group 不会热加载 `.so`。
- `source install/setup.bash` 加载项目环境；`bash scripts/agent_check.sh` 是快速启发式检查，不能代替行为测试。
- 按变更运行受影响测试；需要完整集成验证时运行 `bash run_all_tests.sh`。当前结果以实际命令输出为准，不沿用历史测试数量或 issue 中的结论。
- `bash run_demo.sh` 是仿真演示入口，脚本会清理旧 demo 进程；运行前核对清理范围和进程归属，避免终止其他任务。不要为纯文档修改启动仿真。

## 当前硬件安全边界

- sim：hardware bridge 不创建真实控制 I/O。
- shadow：仅记录命令请求与安全拒绝，不收发 CAN；无真实反馈时不发布硬件状态或报告健康。
- live：final launch 与 direct hardware bridge 均保持 fail closed。解除限制属于独立的硬件工作，不因普通修复或整理而开启。
- command、补零、重打时间戳及人工 acknowledgement 均不能替代真实反馈。
- containment、测试通过不等于真机准入或物理停车能力。验证应覆盖反馈来源、时间基准、freshness、watchdog 与恢复条件；不能放宽安全断言来使测试通过。

## 交付

默认使用初学者容易理解的中文，说明结果、原因、涉及入口、实际验证与重要限制；不要求小修改填写固定长模板。

## Agent skills

### Issue tracker

Issues and specs are tracked in GitHub Issues for this repository. See `docs/agents/issue-tracker.md`.

### Domain docs

This repository uses a single-context layout with `docs/CONTEXT.md` and `docs/adr/`. See `docs/agents/domain.md`.

<!-- TRELLIS:START -->
# Trellis Instructions

These instructions are for AI assistants working in this project.

This project is managed by Trellis. The working knowledge you need lives under `.trellis/`:

- `.trellis/workflow.md` — development phases, when to create tasks, skill routing
- `.trellis/spec/` — package- and layer-scoped coding guidelines (read before writing code in a given layer)
- `.trellis/workspace/` — per-developer journals and session traces
- `.trellis/tasks/` — active and archived tasks (PRDs, research, jsonl context)

If a Trellis command is available on your platform (e.g. `/trellis:finish-work`, `/trellis:continue`), prefer it over manual steps. Not every platform exposes every command.

If you're using Codex or another agent-capable tool, additional project-scoped helpers may live in:
- `.agents/skills/` — reusable Trellis skills
- `.codex/agents/` — optional custom subagents

Managed by Trellis. Edits outside this block are preserved; edits inside may be overwritten by a future `trellis update`.

<!-- TRELLIS:END -->
