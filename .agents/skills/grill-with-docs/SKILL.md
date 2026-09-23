---
name: grill-with-docs
description: 在复杂 issue 或当前 Trellis 任务的规划阶段检查 spec、PRD 与设计文档，澄清影响验收的产品规则或领域决定并记录结果。
---

# Grill with Docs

此项目采用 Matt Pocock `grill-with-docs` 的提问与领域记录方法。来源版本记录在 `.agents/skills/trellis-local/SKILL.md`。

读取 issue、现有 spec、当前任务的 `prd.md` 和已有 `design.md`；涉及领域规则时再查 `CONTEXT.md` 与相关 ADR。列出影响验收且尚未决定的问题，给出建议答案，并与用户确认。收到答案后检查规划文件，直到验收条件足够明确。没有来源文档时，先依据 issue 整理草稿，再检查草稿中的未定决定。

把用户确认的需求和验收条件写入当前任务的 `prd.md`，把技术决定写入 `design.md`。稳定、可复用、已验证并经用户确认的项目知识，按项目规则写入 `CONTEXT.md` 或 ADR。Trellis 管理任务状态和阶段；本技能承担 Codex Phase 1.1 的文档检查与需求讨论。

在 `task.py start` 前检查规划文件是否反映已确认的决定。没有当前 Trellis 任务时，按用户要求交付审查结果或更新已有文档，不自行创建任务。
