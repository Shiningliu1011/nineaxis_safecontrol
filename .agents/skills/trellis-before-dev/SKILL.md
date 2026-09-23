---
name: trellis-before-dev
description: 当前 Trellis 任务进入实现阶段时，读取受影响代码层的项目规范、规划文件和相关检查项。
---

当前 Trellis 任务开始修改代码前，读取相关开发规范。

Execute these steps:

1. **Read current task artifacts**:
   - `prd.md` for requirements and acceptance criteria
   - `design.md` if present for technical design
   - `implement.md` if present for execution order and validation plan

2. **Discover packages and their spec layers**:
   ```bash
   python3 ./.trellis/scripts/get_context.py --mode packages
   ```

3. **Identify which specs apply** to your task based on:
   - Which package you're modifying (e.g., `cli/`, `docs-site/`)
   - What type of work (backend, frontend, unit-test, docs, etc.)
   - Any spec/research paths referenced by the task artifacts

4. **Read the spec index** for each relevant module:
   ```bash
   cat .trellis/spec/<package>/<layer>/index.md
   ```
   Follow the **"Pre-Development Checklist"** section in the index.

5. **Read the specific guideline files** listed in the Pre-Development Checklist that are relevant to your task. The index points to the actual guideline files (e.g., `error-handling.md`, `conventions.md`, `testing.md`). Read those files to understand the coding standards and patterns.

6. **Always read shared guides**:
   ```bash
   cat .trellis/spec/guides/index.md
   ```

7. **For a non-trivial task, state the change boundary before writing code.** Non-trivial means it touches more than one file, crosses a layer, changes a public interface, or edits code you did not just write. Write down:
   - the smallest behavior gap between what happens now and what should happen
   - where that behavior actually lives (not where it is easiest to intercept)
   - which files you expect to change, and why each one is necessary
   - what you are explicitly not doing in this task
   - if a local refactor is needed, how you will show it did not change behavior

   A small, well-scoped change does not need this — do it directly.

   If the real scope turns out to be clearly larger than this, say so and why before continuing. Do not widen the change on your own.

8. Understand the coding standards and patterns you need to follow, then proceed with your development plan.

当前 Trellis 任务修改代码前必须完成此步骤。Inline 任务按当前需求读取相关文件和项目规则。
