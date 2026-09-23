---
name: trellis-start
description: "Initializes an AI development session by reading workflow guides, developer identity, git status, active tasks, and project guidelines from .trellis/. Classifies incoming tasks and routes to the project planning, direct edit, or task workflow. Use when beginning a new coding session, resuming work, starting a new task, or re-establishing project context."
---

# Start Session

Initialize a Trellis-managed development session. This platform has no session-start hook, so manually load the equivalent compact context by following these steps.

---

## Step 1: Current state
Identity, git status, current task, active tasks, journal location.

```bash
python3 ./.trellis/scripts/get_context.py
```

If this output includes a line beginning `Trellis update available:`, copy the full line verbatim when summarizing session context. Do not shorten operational command hints.

## Step 2: Workflow overview
Compact Phase Index, request triage rules, planning artifact contract, and the step-detail command.

```bash
python3 ./.trellis/scripts/get_context.py --mode phase
```

Full guide in `.trellis/workflow.md` (read on demand).

## Step 3: Decide next action
From Step 1 you know the current task and status. Check the task directory:

- **Active task status `planning` + no `prd.md`** → Phase 1.1. Review the issue/spec with `grill-with-docs` and write `prd.md`.
- **Active task status `planning` + `prd.md` exists** → stay in Phase 1. Lightweight tasks can be PRD-only; complex tasks need `design.md` + `implement.md`. Load the relevant Phase 1 step detail before `task.py start`.
- **Active task status `in_progress`** → Phase 2 step 2.1. Load the step detail:
  ```bash
  python3 ./.trellis/scripts/get_context.py --mode phase --step 2.1 --platform codex
  ```
- **No active task** → classify first. Handle simple work inline. For other engineering work, use the relevant Matt method in the main session. Use Trellis for work needing cross-session state or several durable decisions when task creation has been authorized.

## Step 4: Read guidelines when coding

For an active Trellis task that will modify code, use `trellis-before-dev` to read the relevant package indexes and guideline files. For Inline work, read only the files and project rules needed for the current change. Read-only answers do not require a package-wide guideline search.

---

## Skill routing (quick reference)

| User intent | Skill |
|---|---|
| Complex issue or spec needing review | `grill-with-docs` |
| Active Trellis task about to write code | `trellis-before-dev` |
| Reviewed issue or spec ready for implementation | Matt `implement` |
| Active Trellis task done coding / quality check | `trellis-check` |
| Active Trellis task with repeated debugging | `trellis-break-loop` |
| Stable, reusable, verified knowledge confirmed by the user | `trellis-update-spec` |

Full rules + anti-rationalization table in `.trellis/workflow.md`.
