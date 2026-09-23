---
name: trellis-break-loop
description: 当前 Trellis 任务反复遇到同类故障时，分析原因和预防办法，并审查是否产生可保存的项目知识。
---

# Break the Loop - Deep Bug Analysis

When debug is complete, use this for deep analysis to break the "fix bug -> forget -> repeat" cycle.

---

## Analysis Framework

Analyze the bug you just fixed from these 5 dimensions:

### 1. Root Cause Category

Which category does this bug belong to?

| Category | Characteristics | Example |
|----------|-----------------|---------|
| **A. Missing Spec** | No documentation on how to do it | New feature without checklist |
| **B. Cross-Layer Contract** | Interface between layers unclear | API returns different format than expected |
| **C. Change Propagation Failure** | Changed one place, missed others | Changed function signature, missed call sites |
| **D. Test Coverage Gap** | Unit test passes, integration fails | Works alone, breaks when combined |
| **E. Implicit Assumption** | Code relies on undocumented assumption | Timestamp seconds vs milliseconds |

### 2. Why Fixes Failed (if applicable)

If you tried multiple fixes before succeeding, analyze each failure:

- **Surface Fix**: Fixed symptom, not root cause
- **Incomplete Scope**: Found root cause, didn't cover all cases
- **Tool Limitation**: grep missed it, type check wasn't strict
- **Mental Model**: Kept looking in same layer, didn't think cross-layer

### 3. Prevention Mechanisms

What mechanisms would prevent this from happening again?

| Type | Description | Example |
|------|-------------|---------|
| **Documentation** | Write it down so people know | Update thinking guide |
| **Architecture** | Make the error impossible structurally | Type-safe wrappers |
| **Compile-time** | Strict type checking, no escape hatches | Signature change causes compile error |
| **Runtime** | Monitoring, alerts, scans | Detect orphan entities |
| **Test Coverage** | E2E tests, integration tests | Verify full flow |
| **Code Review** | Checklist, PR template | "Did you check X?" |

### 4. Systematic Expansion

What broader problems does this bug reveal?

- **Similar Issues**: Where else might this problem exist?
- **Design Flaw**: Is there a fundamental architecture issue?
- **Process Flaw**: Is there a development process improvement?
- **Knowledge Gap**: Is the team missing some understanding?

### 5. Knowledge Capture

记录已验证的原因与预防办法，并列出可能需要保存的项目知识：

- [ ] 当前任务的 `research/retrospective.md` 是否记录了分析证据？
- [ ] 是否有稳定、可复用且经过验证的规范候选内容？
- [ ] 是否有需要用户授权的 GitHub issue 或其他外部记录？

---

## Output Format

Please output analysis in this format:

```markdown
## Bug Analysis: [Short Description]

### 1. Root Cause Category
- **Category**: [A/B/C/D/E] - [Category Name]
- **Specific Cause**: [Detailed description]

### 2. Why Fixes Failed (if applicable)
1. [First attempt]: [Why it failed]
2. [Second attempt]: [Why it failed]
...

### 3. Prevention Mechanisms
| Priority | Mechanism | Specific Action | Status |
|----------|-----------|-----------------|--------|
| P0 | ... | ... | TODO/DONE |

### 4. Systematic Expansion
- **Similar Issues**: [List places with similar problems]
- **Design Improvement**: [Architecture-level suggestions]
- **Process Improvement**: [Development process suggestions]

### 5. Knowledge Capture
- [ ] [规范候选内容与可能需要的后续工作]
```

---

## 分析目标

记录经过验证的故障原因、修复证据和可能有效的预防办法。

Three levels of insight:
1. **Tactical**: How to fix THIS bug
2. **Strategic**: How to prevent THIS CLASS of bugs
3. **Philosophical**: How to expand thinking patterns

## Thinking Framework: Bayesian Reasoning

When multiple root causes are plausible and evidence is incomplete, update your beliefs proportionally to new evidence rather than clinging to initial assumptions.

### Step 1: Establish Priors

Before investigating, state what you believe and why:

| Hypothesis | Prior | Reasoning |
|------------|-------|-----------|
| H1: [cause A] | 40% | Most common for this pattern |
| H2: [cause B] | 30% | Plausible given environment |
| H3: [other] | 30% | Catch-all |

Priors must sum to 100%. If you can't assign probabilities, investigate first.

### Step 2: Observe Evidence

Document what you found — be specific about reliability:

- What exactly did you observe?
- How reliable? (test output > log message > user report > hunch)
- Could multiple hypotheses explain this?

### Step 3: Update Beliefs

For each hypothesis, ask: **How likely is this evidence if this hypothesis were true?**

Direction of update matters more than calculation:
- Evidence strongly predicted by H1 → H1 probability increases
- Evidence contradicts H2 → H2 probability decreases
- Evidence equally likely under all → no update

### Step 4: Seek Discriminating Evidence

Don't gather more of the same. Find evidence that **differs strongly** between top hypotheses.

> If H1 and H3 are close: "What would I see if H1 is true but not if H3 is true?" Then check for that.

### Step 5: State Confidence

| Confidence | Action |
|------------|--------|
| 90%+ | Proceed with fix, monitor |
| 70-90% | Gather evidence for the remaining uncertainty |
| 50-70% | Test hypothesis before committing |
| <50% | Need more evidence, don't guess |

Never express binary certainty when evidence is incomplete. Use "most likely", "plausible but unlikely", "worth investigating".

### Common Fallacies

| Fallacy | Example | Correction |
|---------|---------|------------|
| **Base rate neglect** | "Test failed → code is broken" | How often do tests fail for other reasons? |
| **Confirmation bias** | "Must be a race condition, let me find race evidence" | Actively seek evidence AGAINST your top hypothesis |
| **Anchoring** | "Last time it was caching, probably caching again" | Establish priors from current context, not yesterday's bug |

---

## 分析交接

当前 Trellis 任务已授权记录工作内容时，将复现证据、原因和预防办法写入任务的 `research/retrospective.md`。用户仅要求分析时，在答复中报告结论，不修改文件。

按照 Phase 3.3 审查规范候选内容。只有知识稳定、可复用、已经验证，且用户确认了内容和目标文件后，才调用 `trellis-update-spec`。创建或修改 GitHub issue、提交、推送以及创建 PR 仍按用户的明确授权执行。
