# 主线地图与同步核验

日期：2026-10-27。仓库 `/home/lsn/robot/robot_safecontrol`，交付分支 `main`。

本次用户授权合并主线、删除静态实验草稿、取消相关任务、整理全部阻塞关系，并提交同步文档。

## 检查范围

- 59 项主线子任务及原生顺序、标题编号、类型标签、状态和指派。
- 179 条原生依赖，包括 160 条地图内部依赖与 19 条已完成来源依赖；检查循环、完整前置和可领取入口。
- #171、#182、#183 的取消状态，#172–#181 的原完成状态，以及这 13 项退出地图后的父子关系、标签和依赖。
- #144、#145、#147、#169、#170 的远端正文与本地规格、依赖、覆盖表。
- 文档链接、Trellis 元数据、静态草稿清理、本地与远程 Git 提交。

## 实际结果

| 检查 | 结果 |
|---|---|
| 当前地图子任务 | 59 项，01–59 连续编号，原生顺序一致 |
| 原生依赖 | 179 条：160 条地图内、19 条指向已完成来源；没有循环 |
| 当前可新领取 | #148、#135；#95 由 `lsn` 继续 |
| 取消任务 | #171、#182、#183 均为 closed / not_planned，入向与出向依赖均为空 |
| 前期完成任务 | #172–#181 状态和验收正文保持原内容，13 项静态任务均退出父子关系和 map:147 标签 |
| 远端正文 | #144、#145、#147、#169、#170 及地图依赖表评论共 6 项逐字核对通过 |
| 本地文档 | 34 篇 Markdown、138 个本地链接、7 份发布正文检查通过 |
| Trellis | 三项静态任务为 cancelled，地图维护任务的 implement/check context 各 2 项通过 |
| 历史证据 | 9 份原文与 f3049ac 的对应来源逐字节核对通过，历史文档的入口提示独立检查 |
| 差异检查 | git diff --check 通过；本轮修改为文档与任务记录 |

核验命令：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 .scratch/main-map-20261027/check_documents.py
PYTHONDONTWRITEBYTECODE=1 python3 .scratch/main-map-20261027/fetch_graph.py --output .scratch/main-map-20261027/graph-after.json
PYTHONDONTWRITEBYTECODE=1 python3 .scratch/main-map-20261027/check_tracker.py
PYTHONDONTWRITEBYTECODE=1 python3 .scratch/predictive-coordination-spec/check_history.py
PYTHONDONTWRITEBYTECODE=1 python3 .trellis/scripts/task.py validate .trellis/tasks/10-26-predictive-coordination-spec
```

工具与 API 快照保存在 `.scratch/main-map-20261027/`，包括 `graph-before.json`、`expected.json`、`graph-after.json`、`tracker-check.json` 和 `documents-check.json`。原生依赖和正文均在写入后重新读取核对。

## 分支与资料

`codex/static-capacity`、`codex/static-evidence`、`codex/static-method-evidence`、`codex/static-s5-timing-diagnosis` 的全部提交已包含在 `f3049ac` 中。独立静态实验草稿的本地分支及工作目录已删除。

该工作目录中的前期本地测量资料保存在 `.scratch/main-map-20261027/static-scenes-local-evidence.tar.gz`，SHA-256 为 `32e38a2b9b90b99169129ab3a51657f51a0c4bf35c654fe645c05059b9482fc3`。原始实验数据仍按[证据入口](../../static_obstacle_experiments_handoff.md)保存。

本次合并提交为 `5645a61aea8de895fc9ce990da7126436c44bc6e`。同步后读取远程引用，本地 `main`、远程 `main` 和 `codex/static-obstacle-experiments` 均为该提交；工作区没有未提交改动。

`git branch --no-merged main` 与 `git log --branches --not main --oneline` 均无输出，全部有效分支提交已包含在 `main`。独立静态实验草稿的 `codex/static-scenes` 本地和远程分支均已删除。
