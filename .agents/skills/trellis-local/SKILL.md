---
name: trellis-local
description: 在本项目维护 Matt skills 与 Trellis 的阶段路由、产物交接和来源记录时使用。
---

# 本项目的 Skill 路由

`.trellis/workflow.md` 管理阶段与路由，`.trellis/skill-integration.json` 记录验证字段，现有 `.codex/hooks.json` 的 `UserPromptSubmit` hook 注入当前阶段提示。Codex 使用 `codex.dispatch_mode: inline`。Trellis 管理任务状态、项目规范和验收；Matt skills 提供文档检查、代码实现、故障诊断、测试驱动开发与独立审查方法。

简单任务直接按 Inline 处理。其余尚未进入 Trellis 生命周期的工程任务，按当前可用的 Matt skill description 选择方法，由主会话完成；只有已经进入 Trellis 生命周期的任务才要求把阶段产物写入任务目录。

已选择的路由为 `grill-with-docs`、`implement`、`diagnosing-bugs`、`tdd` 和 `code-review`。Codex 规划阶段用 `grill-with-docs` 检查 issue 与 spec，决定写入 `prd.md` 和 `design.md`；执行阶段先用 `trellis-before-dev` 读取项目规范，再由主会话使用 Matt `implement`，随后运行 `trellis-check`。故障证据写入 `research/diagnosis.md`，测试循环写入 `implement.md`。记录了高风险范围时，再执行 `code-review` 并写入 `research/review.md`。稳定项目知识仍需经过用户确认才写入规范或领域文件。

本项目设置 `session_auto_commit: false`。任务归档和会话记录只修改本地文件；只有用户明确要求提交时才执行 `git add` 与 `git commit`。Phase 3.3 先检查知识是否满足保存条件，用户确认后才调用 `trellis-update-spec` 修改规范。

来源核验日期：2026-09-23。Matt Pocock `mattpocock/skills` 的已核验提交为 `c55ee46073ed923f86ce59a5eb3b6d895095d1b7`；本机相关 skill 与该提交一致。当日查询到的 Trellis CLI 和 npm 版本均为 `0.6.17`。本项目的五个 Matt skill 入口包含适用于当前仓库的使用说明；上游版本号只说明核验来源，不表示这些入口与上游文件逐字相同。
