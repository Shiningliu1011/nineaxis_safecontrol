---
name: trellis-update-spec
description: 当前 Trellis 任务产生稳定、可复用且经过验证的项目知识，并已由用户确认内容和目标文件后，将其写入项目规范。
---

# Update Code-Spec - Capture Executable Contracts

仅在 `.trellis/workflow.md` Phase 3.3 的规范保存条件满足，并且用户确认了拟保存的内容与目标文件后，修改相关规范。故障分析或代码检查可以提出候选内容；这些步骤本身不触发规范写入。

**Timing**: 完成验证并取得用户确认后。

---

## Code-Spec First Rule (CRITICAL)

In this project, "spec" for implementation work means **code-spec**:
- Executable contracts (not principle-only text)
- Concrete signatures, payload fields, env keys, and boundary behavior
- Testable validation/error behavior

用户确认的知识涉及 infra 或跨层接口时，按下方要求记录可以检查的技术内容。

### 需要详细记录的范围

获准保存的知识涉及以下内容时，使用下方完整结构：
- New/changed command or API signature
- Cross-layer request/response contract change
- Database schema/migration change
- Infra integration (storage, queue, cache, secrets, env wiring)

### Mandatory Output (7 Sections)

For triggered tasks, include all sections below:
1. Scope / Trigger
2. Signatures (command/API/DB)
3. Contracts (request/response/env)
4. Validation & Error Matrix
5. Good/Base/Bad Cases
6. Tests Required (with assertion points)
7. Wrong vs Correct (at least one pair)

---

## 可能产生规范候选内容的工作

以下工作可以产生候选内容。先确认其稳定、可复用并经过验证，再取得用户确认；单次实现或故障修复不会自动触发规范写入。

| Trigger | Example | Target Spec |
|---------|---------|-------------|
| **Implemented a feature** | Added a new integration or module | Relevant spec file |
| **Made a design decision** | Chose extensibility pattern over simplicity | Relevant spec + "Design Decisions" section |
| **Fixed a bug** | Found a subtle issue with error handling | Relevant spec (e.g., error-handling docs) |
| **Discovered a pattern** | Found a better way to structure code | Relevant spec file |
| **Hit a gotcha** | Learned that X must be done before Y | Relevant spec + "Common Mistakes" section |
| **Established a convention** | Team agreed on naming pattern | Quality guidelines |
| **New thinking trigger** | "Don't forget to check X before doing Y" | `guides/*.md` (as a checklist item) |

功能实现中的决定只有满足上述条件并经用户确认，才写入项目规范。

---

## Spec Structure Overview

- `.trellis/spec/<layer>/index.md`：对应代码层的入口与链接。
- `.trellis/spec/<layer>/*.md`：对应主题的具体规范。
- `.trellis/spec/guides/index.md`：跨代码层检查项的入口。
- `.trellis/spec/guides/*.md`：按主题组织的检查项。

### CRITICAL: Code-Spec vs Guide - Know the Difference

| Type | Location | Purpose | Content Style |
|------|----------|---------|---------------|
| **Code-Spec** | `<layer>/*.md` | Tell AI "how to implement safely" | Signatures, contracts, matrices, cases, test points |
| **Guide** | `guides/*.md` | Help AI "what to think about" | Checklists, questions, pointers to specs |

**Decision Rule**: Ask yourself:

- "This is **how to write** the code" → Put in a spec layer directory
- "This is **what to consider** before writing" → Put in `guides/`

**Example**:

| Learning | Wrong Location | Correct Location |
|----------|----------------|------------------|
| "Use API X not API Y for this task" | ❌ `guides/` (too specific for a thinking guide) | ✅ Relevant spec file (concrete convention) |
| "Remember to check X when doing Y" | ❌ Spec file (too abstract for a spec) | ✅ `guides/` (thinking checklist) |

**Guides should be short checklists that point to specs**, not duplicate the detailed rules.

---

## Update Process

### Step 1: Identify What You Learned

Answer these questions:

1. **What did you learn?** (Be specific)
2. **Why is it important?** (What problem does it prevent?)
3. **Where does it belong?** (Which spec file?)

### Step 2: Classify the Update Type

| Type | Description | Action |
|------|-------------|--------|
| **Design Decision** | Why we chose approach X over Y | Add to "Design Decisions" section |
| **Project Convention** | How we do X in this project | Add to relevant section with examples |
| **New Pattern** | A reusable approach discovered | Add to "Patterns" section |
| **Forbidden Pattern** | Something that causes problems | Add to "Anti-patterns" or "Don't" section |
| **Common Mistake** | Easy-to-make error | Add to "Common Mistakes" section |
| **Convention** | Agreed-upon standard | Add to relevant section |
| **Gotcha** | Non-obvious behavior | Add warning callout |

### Step 3: Read the Target Code-Spec

Before editing, read the current code-spec to:
- Understand existing structure
- Avoid duplicating content
- Find the right section for your update

```bash
cat .trellis/spec/<category>/<file>.md
```

### Step 4: Make the Update

Follow these principles:

1. **Be Specific**: Include concrete examples, not just abstract rules
2. **Explain Why**: State the problem this prevents
3. **Show Contracts**: Add signatures, payload fields, and error behavior
4. **Show Code**: Add code snippets for key patterns
5. **Keep it Short**: One concept per section

### Step 5: Update the Index (if needed)

If you added a new section or the code-spec status changed, update the category's `index.md`.

---

## Update Templates

### Mandatory Template for Infra/Cross-Layer Work

```markdown
## Scenario: <name>

### 1. Scope / Trigger
- Trigger: <why this requires code-spec depth>

### 2. Signatures
- Backend command/API/DB signature(s)

### 3. Contracts
- Request fields (name, type, constraints)
- Response fields (name, type, constraints)
- Environment keys (required/optional)

### 4. Validation & Error Matrix
- <condition> -> <error>

### 5. Good/Base/Bad Cases
- Good: ...
- Base: ...
- Bad: ...

### 6. Tests Required
- Unit/Integration/E2E with assertion points

### 7. Wrong vs Correct
#### Wrong
...
#### Correct
...
```

### Adding a Design Decision

```markdown
### Design Decision: [Decision Name]

**Context**: What problem were we solving?

**Options Considered**:
1. Option A - brief description
2. Option B - brief description

**Decision**: We chose Option X because...

**Example**:
\`\`\`typescript
// How it's implemented
code example
\`\`\`

**Extensibility**: How to extend this in the future...
```

### Adding a Project Convention

```markdown
### Convention: [Convention Name]

**What**: Brief description of the convention.

**Why**: Why we do it this way in this project.

**Example**:
\`\`\`typescript
// How to follow this convention
code example
\`\`\`

**Related**: Links to related conventions or specs.
```

### Adding a New Pattern

```markdown
### Pattern Name

**Problem**: What problem does this solve?

**Solution**: Brief description of the approach.

**Example**:
\`\`\`
// Good
code example

// Bad
code example
\`\`\`

**Why**: Explanation of why this works better.
```

### Adding a Forbidden Pattern

```markdown
### Don't: Pattern Name

**Problem**:
\`\`\`
// Don't do this
bad code example
\`\`\`

**Why it's bad**: Explanation of the issue.

**Instead**:
\`\`\`
// Do this instead
good code example
\`\`\`
```

### Adding a Common Mistake

```markdown
### Common Mistake: Description

**Symptom**: What goes wrong

**Cause**: Why this happens

**Fix**: How to correct it

**Prevention**: How to avoid it in the future
```

### Adding a Gotcha

```markdown
> **Warning**: Brief description of the non-obvious behavior.
>
> Details about when this happens and how to handle it.
```

---

## Interactive Mode

If you're unsure what to update, answer these prompts:

1. **What did you just finish?**
   - [ ] Fixed a bug
   - [ ] Implemented a feature
   - [ ] Refactored code
   - [ ] Had a discussion about approach

2. **What did you learn or decide?**
   - Design decision (why X over Y)
   - Project convention (how we do X)
   - Non-obvious behavior (gotcha)
   - Better approach (pattern)

3. **Would future AI/developers need to know this?**
   - To understand how the code works → 提出候选内容并检查证据
   - To maintain or extend the feature → 提出候选内容并检查复用价值
   - To avoid repeating mistakes → 提出候选内容并检查验证结果
   - Purely one-off implementation detail → 不写入项目规范

4. **Which area does it relate to?**
   - [ ] Backend code
   - [ ] Frontend code
   - [ ] Cross-layer data flow
   - [ ] Code organization/reuse
   - [ ] Quality/testing

---

## Quality Checklist

Before finishing your code-spec update:

- [ ] Is the content specific and actionable?
- [ ] Did you include a code example?
- [ ] Did you explain WHY, not just WHAT?
- [ ] Did you include executable signatures/contracts?
- [ ] Did you include validation and error matrix?
- [ ] Did you include Good/Base/Bad cases?
- [ ] Did you include required tests with assertion points?
- [ ] Is it in the right code-spec file?
- [ ] Does it duplicate existing content?
- [ ] Would a new team member understand it?

---

## 与其他技能的关系

- `trellis-break-loop` 分析重复故障并提出规范候选内容。
- `trellis-check` 检查本次改动和证据，记录可能需要保存的知识。
- `trellis-update-spec` 仅在 Phase 3.3 条件满足并经用户确认后修改规范。
- `trellis-finish-work` 只处理已授权的归档或会话记录；Git 提交另按明确要求执行。

---

## Core Philosophy

> **Code-specs are living documents. Every debugging session, every "aha moment" is an opportunity to make the implementation contract clearer.**

The goal is **institutional memory**:
- What one person learns, everyone benefits from
- What AI learns in one session, persists to future sessions
- Mistakes become documented guardrails
