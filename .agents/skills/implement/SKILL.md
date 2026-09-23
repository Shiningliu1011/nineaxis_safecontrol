---
name: implement
description: 根据已审阅的 issue、spec 或 Trellis 规划文件完成代码实现与相关验证；当前 Trellis 任务进入 in_progress 后在 Phase 2.1 使用。
---

# Implement

采用 Matt Pocock `implement` 的方法，从明确的需求和验收条件执行编码。来源版本记录在 `.agents/skills/trellis-local/SKILL.md`。

当前任务由 Trellis 管理时，读取 `prd.md`、已有 `design.md`、`implement.md` 和相关 `research/` 资料，并先使用 `trellis-before-dev` 读取涉及代码层的项目规范。按已审阅的范围完成实现；发现需求变化时，更新规划文件并重新检查影响。

实现期间运行受影响的单项测试及适用的类型检查、构建检查；改动涉及多个模块或集成行为时运行相应的集成验证。记录实际执行的命令和结果。完成编码后交给 `trellis-check` 检查；记录了高风险范围时，再按项目工作流使用 `code-review`。

没有 Trellis 任务时，根据用户授权的 issue 或 spec 直接实现与验证。提交、推送和创建 PR 仅按用户明确要求执行。
