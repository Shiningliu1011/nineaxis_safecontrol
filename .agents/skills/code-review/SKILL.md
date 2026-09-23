---
name: code-review
description: 用户要求审查变更，或当前 Trellis 任务记录了共享控制内核、安全门、ROS 公共接口或迁移改动等高风险范围时使用。
---

# Code Review

此项目采用 Matt Pocock `code-review` 的规范与需求双线审查。来源版本记录在 `.agents/skills/trellis-local/SKILL.md`。

在 Trellis 任务中，先完成 `trellis-check`，再从任务记录的基准分支或用户指定的 Git 引用获取变更。确认引用存在且变更非空。读取任务的 `prd.md`、`design.md`、相关 ADR、`CODING_STANDARDS.md` 和受影响的 Trellis 规范。

安排两个独立审查者并行检查：一个核对项目规范及代码质量，另一个核对需求、验收条件和改动范围。每条发现给出具体文件位置、证据与影响；两方面的结论分开呈现。缺少基准引用或需求文件时，说明具体缺口，不推断完整审查已经完成。

用户仅要求 review 时，报告发现，不修改文件。当前 Trellis 任务授权继续修改时，把发现、修复情况和仍需处理的事项写入 `research/review.md`；阻碍验收的问题处理后，再进入规范审查和提交授权检查。审查不代替 `trellis-check`，也不扩大用户已授权的改动范围。
