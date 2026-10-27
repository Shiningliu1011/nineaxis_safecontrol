# 规格与地图维护验证

核验日期：2026-10-26。工作区为 `/home/lsn/robot/robot_safecontrol`，分支 `codex/static-obstacle-experiments`，HEAD 为 `6a9705260f12db502eef23867ddf0c60febb8aad`。

## 地图与任务

- 当前唯一开放地图为[九轴曲面作业完整项目](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/147)。
- 原 72 项子任务身份全部保留，40 项子任务更新正文或标题，地图正文同步更新。
- 当前 62 项开放、10 项已完成，完成状态与原有指派经过逐项比对保持一致；总体规格任务由本轮领取。
- 每个子任务恰好具有一个类型标签及 `map:147` 归属；[总体规格](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/145)使用 `wayfinder:spec`。
- 195 条原生依赖全部读回核对，其中 176 条在当前地图内，19 条指向已经完成的来源任务。
- 原生子任务显示顺序与 01–72 标题顺序一致，全部当前地图内前置的标题编号小于后继，没有循环。
- [本地依赖表](dependencies.md)的 72 项任务及直接前置与 GitHub 一致，规格及修改正文与已准备文件逐项一致。
- [研究目标与项目要求记录](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/145#issuecomment-6407884662)和[静态实验接续记录](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/171#issuecomment-6407885586)已保存。

检查命令为 `.scratch/predictive-coordination-spec/snapshot_tracker.py` 和 `check_tracker.py`，原始读回保存在同目录 `snapshot-before.json`、`snapshot-after.json`、`expected-tracker.json` 和 `map-after.json`。这些文件是本机核查制品。

## 本地文档与来源

| 检查 | 结果 |
|---|---|
| 当前文档及任务规划 | 24 篇检查通过 |
| 本地文件链接 | 100 个存在性检查通过 |
| 技术路线来源入口 | 10 篇新增状态提示及当前入口检查通过 |
| Trellis 元数据与父子关系 | 4 项当前任务检查通过 |
| 总体规格需求覆盖 | R01–R12、三方法及三类进度均已登记 |
| 远端正文草稿链接 | 30 份仅使用有效形态的 GitHub 任务链接，无本地相对链接 |
| 历史原文保存 | 9 份文档和任务记录与原内容逐字节核对一致；文档新增的历史提示单独扣除 |
| NURBS 原 ZIP | SHA256 与核查值一致，原文件保持内容身份 |
| `git diff --check` | 通过 |
| Trellis context validate | `implement.jsonl` 和 `check.jsonl` 各 2 项，检查通过 |

本地检查入口为 `.scratch/predictive-coordination-spec/check_documents.py` 和 `check_history.py`。NURBS 数值与源码检查见[事实依据](requirements-evidence.md)。`九轴技术路线/` 由本机 `.git/info/exclude` 保留，跨设备仍需单独传输来源资料。

## 验证范围

本轮修改范围为规划文档、规格、任务元数据和 GitHub 地图。控制代码没有改动，没有启动控制测试、仿真或真实设备。旧实验成功与失败保持原证据身份，新算法和新工艺要求由后续任务运行验收。

本轮没有删除实验数据，没有执行 Git 提交或推送。两份历史实验卷未挂载，卷内当前内容尚未核查；访问条件及保存安排见[接手记录](../../static_obstacle_experiments_handoff.md)。
