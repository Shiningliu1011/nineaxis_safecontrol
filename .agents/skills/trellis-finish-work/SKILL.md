---
name: trellis-finish-work
description: 用户已授权归档任务或记录会话时，核对验收结果与文件范围，并完成相应的本地 Trellis 记录。
---

# Finish Work

仅处理用户已授权的任务归档或会话记录。先确认 `.trellis/workflow.md` 的 Phase 3.3 规范审查和 Phase 3.4 提交授权检查已经完成。当前项目设置 `session_auto_commit: false`；此技能不执行提交、推送或创建 PR。

## 核对任务与改动

```bash
python3 ./.trellis/scripts/get_context.py --mode record
git status --porcelain
```

核对任务验收。代码任务检查 `trellis-check` 的验证结果；规划会话检查相应的规划文件。区分当前任务与其他工作已有的改动，保留后者。当前任务仍有未提交改动时，记录文件范围并继续执行已授权的归档或会话记录；提交只在用户明确要求时由 Phase 3.4 处理。

## 归档已授权的任务

仅当用户授权归档指定任务，且任务验收已经完成时执行：

```bash
python3 ./.trellis/scripts/task.py archive <task-name> --no-commit
```

其他已完成任务只有在授权范围包含它们时才归档。归档会移动本地任务目录；`--no-commit` 保证本次操作不执行 Git 提交。

## 记录已授权的会话

仅当用户授权记录会话时调用 `add_session.py`。有已提交的工作时，`--commit` 填写对应的提交哈希；本次工作没有提交时填写 `-`：

```bash
python3 ./.trellis/scripts/add_session.py --title "会话标题" --commit "-" --summary "工作记录" --no-commit
```

最后报告任务状态、归档位置或会话记录位置、实际验证，以及当前工作区的相关改动。
