# Issue tracker: GitHub

Issues and specs for this repo live as GitHub issues. Use the `gh` CLI for all operations.

## Conventions

- **Create an issue**: prepare `.scratch/<feature>/issue-body.md` with the file editing tool, then run `gh issue create --title "..." --body-file .scratch/<feature>/issue-body.md`.
- **Read an issue**: `gh issue view <number> --comments`, filtering comments by `jq` and also fetching labels.
- **List issues**: `gh issue list --state open --json number,title,body,labels,comments --jq '[.[] | {number, title, body, labels: [.labels[].name], comments: [.comments[].body]}]'` with appropriate `--label` and `--state` filters.
- **Comment on an issue**: use `gh issue comment <number> --body "..."` for one line. For multiple lines, prepare a Markdown file under `.scratch/<feature>/` and pass it with `--body-file`.
- **Apply / remove labels**: `gh issue edit <number> --add-label "..."` / `--remove-label "..."`
- **Close**: `gh issue close <number> --comment "..."`

Infer the repo from `git remote -v`; `gh` does this automatically when run inside a clone.

## Pull requests as a triage surface

**PRs as a request surface: no.** _(Set to `yes` if this repo treats external PRs as feature requests; `/triage` reads this flag.)_

When set to `yes`, PRs run through the same labels and states as issues, using the `gh pr` equivalents:

- **Read a PR**: `gh pr view <number> --comments` and `gh pr diff <number>` for the diff.
- **List external PRs for triage**: `gh pr list --state open --json number,title,body,labels,author,authorAssociation,comments` then keep only `authorAssociation` of `CONTRIBUTOR`, `FIRST_TIME_CONTRIBUTOR`, or `NONE` (drop `OWNER`/`MEMBER`/`COLLABORATOR`).
- **Comment / label / close**: `gh pr comment`, `gh pr edit --add-label`/`--remove-label`, `gh pr close`.

GitHub shares one number space across issues and PRs, so a bare `#42` may be either: resolve with `gh pr view 42` and fall back to `gh issue view 42`.

## When a skill says "publish to the issue tracker"

Create a GitHub issue.

## When a skill says "fetch the relevant ticket"

Run `gh issue view <number> --comments`.

## Wayfinding operations

Used by `/wayfinder`. The **map** is a single issue with **child** issues as tickets.

- **Map**: a single issue labelled `wayfinder:map`, holding the Notes / Decisions-so-far / Fog body. `gh issue create --label wayfinder:map`.
- **Child ticket**: an issue linked to the map as a GitHub sub-issue (`gh api` on the sub-issues endpoint). Where sub-issues aren't enabled, add the child to a task list in the map body and put `Part of #<map>` at the top of the child body. Labels: `wayfinder:<type>` (`research`/`prototype`/`grilling`/`task`). Once claimed, the ticket is assigned to the driving dev.
- **Blocking**: GitHub's **native issue dependencies**, the canonical, UI-visible representation. Add an edge with `gh api --method POST repos/<owner>/<repo>/issues/<child>/dependencies/blocked_by -F issue_id=<blocker-db-id>`, where `<blocker-db-id>` is the blocker's numeric **database id** (`gh api repos/<owner>/<repo>/issues/<n> --jq .id`, _not_ the `#number` or `node_id`). GitHub reports `issue_dependencies_summary.blocked_by` (open blockers only, the live gate). Where dependencies aren't available, fall back to a `Blocked by: #<n>, #<n>` line at the top of the child body. A ticket is unblocked when every blocker is closed.
- **Frontier query**: list the map's open children (`gh issue list --state open`, scoped to the map's sub-issues / task list), drop any with an open blocker (`issue_dependencies_summary.blocked_by > 0`, or an open issue in the `Blocked by` line) or an assignee; first in map order wins.
- **Claim**: `gh issue edit <n> --add-assignee @me`, the session's first write.
- **Resolve**: `gh issue comment <n> --body "<answer>"`, then `gh issue close <n>`, then append a context pointer (gist + link) to the map's Decisions-so-far.

## 标签、命名与决定票约定

2026-09-19 经 [D4](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/70) 定案，所有建票与开票会话按本节执行。逐条记录见 `docs/planning/architecture-redesign/d4-decisions.md`。

**标签体系**。三根轴，共 14 个自建标签；GitHub 默认标签不使用。

| 轴 | 标签 | 含义 |
|---|---|---|
| 类型 | `wayfinder:map` | 地图本体 |
| 类型 | `wayfinder:research` | 研究票：读文档、上游源码或本机资源，回答一个待定的事实 |
| 类型 | `wayfinder:prototype` | 原型票：做一个粗糙但可以看的制品供讨论 |
| 类型 | `wayfinder:grilling` | 决策票：需要用户拍板 |
| 类型 | `wayfinder:task` | 人工工作票：决定之前必须完成的事 |
| 类型 | `wayfinder:impl` | 实现票：按 spec 改代码并留下验收证据 |
| 类型 | `wayfinder:verify` | 验收票：核对实现是否满足验收条件 |
| 领域 | `area:kernel` | 控制内核 `portable_oscbf/work` |
| 领域 | `area:core` | 纯逻辑层 |
| 领域 | `area:node` | 节点层 |
| 领域 | `area:plugin` | 规划插件层 |
| 领域 | `area:model` | 模型与配置 |
| 领域 | `area:driver` | 外部驱动 |
| 条件 | `needs-robot` | 必须上真机才能完成（标定、实机验收、接线核对） |

一张票可以带多个领域标签；「谁来做」由指派表达，不另设标签。

**命名规范**。标题写成 `[<类型字母><序号>] <短语>`：R 研究、D 决定、P 原型、S 规格、I 实现、V 验收、T 任务；序号在同一张地图内从 1 递增；决策票在短语后加「（需你拍板）」。票号 `#<编号>` 是唯一身份，跨地图出现相同句柄时用它区分；地图的 Decisions so far 每一行对应的正是票标题。

**决定票的呈现**。正文以评论发在票上，固定六段：标题、背景、现状核验（文件与行号、实测数字）、推荐形状（接口或清单的一览表）、逐问（做法 A/B/C，每条写清后果，再给推荐与理由）、供 spec 采用的要点。用户拍板后另发一条拍板记录评论：逐问定案表、每题的依据、本票产生的制品清单。

**决定记录与制品**。决定全文与依据以票上评论为准，打开票就能读完；工作区草稿与制品留在 `docs/planning/<effort>/` 下供本机会话使用，不提交、不推送。

**执行阶段的组织**。spec 定案后另开一张地图（`wayfinder:map`），执行票是它的子票；粒度按「一次可独立验收的改动」切，一票一个验收条件；顺序用原生阻塞关系连边，先退役与归位、再接入与新增；需要真机的票打 `needs-robot` 并集中到实机窗口；每票的验收命令与证据写在票尾评论。
