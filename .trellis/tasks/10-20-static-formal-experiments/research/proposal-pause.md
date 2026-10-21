# 开题资料交付与用户暂停

用户于本次会话明确要求：提供可修改报告、绪论和 PPT 的数据、图片、已完成实验结论，
通过 Git 同步到另一设备；开题后再继续 #171。
实验按此要求暂停，#171、#182、#183 保持开放，正式采集 0/1080。
Trellis 的生命周期状态保留 `in_progress`，三项任务 `meta.execution_state=paused_by_user` 记录执行暂停。

## 运行状态

`static-complete-cycle-period-study-02.service` 已停止，检查结果为
`ActiveState=inactive`、`SubState=dead`、`MainPID=0`。
停止前等待 B 的 worker 退出与完整反馈报告保存，随后停止队列。
没有启动 C 或任何正式实验，没有自动继续服务。
原有 `execution.json` 是中断前状态，不用它推断当前是否仍有运行进程。

新完整周期研究的原始 A、B 均已产生 `result.json` 和 `feedback_report.json`，
实际 2/20 份原始运行报告完成；A 有 `.process.json`，B 的队列父进程在收取退出码前暂停，
因此没有 B `.process.json`，不补造进程退出元数据。
两组 controller/plant 的实际 `worker_exit_codes` 均为 `[0,0]`，任务和 runtime 均失败。
尚无完整 `period_report.json` 或最终共同周期。

| 完整周期来源 | A / 20 ms | B / 20 ms |
| --- | --- | --- |
| 报告 SHA-256 | `1e574f6878ef2be55db34f89e440f6fdbb85b4b1faaae26e3bd0ee467cd9b4b3` | `75719bcf35a6e98195be4d76b53c2348d19cacab9cd2bab11041f835bf505ee5` |
| manifest SHA-256 | `8d4d7a120bac0d94e1c6a3f03b378ae29f21f8c286bde2ebd24a9a0e92a408f5` | `cedb8340797b788e2487dc357fbc1afa5de2f55c4bb386980c514e1dabbb59d6` |
| result SHA-256 | `c1812d6b3aa14cb2031ea9f18dbbd0c794ec23243fabd57f88783565ed3b0ebb` | `acab1d935251a5436969560b7fa758d0d23ee1e224d78b0b3bae3d516a0d6289` |
| 周期数量 | 15,061 | 14,696 |
| P99.9 / ms | 60.845293 | 59.573316 |
| 超期次数 | 14,849 | 14,631 |

## Git 资料与大型原始记录

开题资料入口为 `docs/proposal-static-obstacles/README.md`，
独立数据与结论文档为 `docs/proposal-static-obstacles/results-and-conclusions.md`。
十份原始报告、配置、manifest、路径、来源身份，三组完整反馈指标序列，
21 张 PNG 与三组 SVG/PDF 随集成分支同步。
`checksums.json` 保存每个资料文件 SHA-256 和字节数，
每组 `data/<run>/source.json` 保存原始文件、两个数据库和运行源码逐文件 SHA-256。
原始 SQLite 数据库没有进入 Git，全部保留：

- 当前 S0/S4、原 NTFS S5：
  `/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/complete-cycle-preflight-01/`。
- 历史 S2 与其他 guarded 记录：同一卷 `guarded-preflight-01/`。
- 新共同周期 A/B：同一卷 `guarded-period-study-02/`。
- Windows S5：
  `/media/lsn/Windows/robot_safecontrol_static_experiments_20261021/complete-cycle-preflight-01/`。
- ext4 S5：主项目 `output/static-capacity-20261020/complete-cycle-preflight-01/d-s5/run/`。
- A/C 配对：主项目 `output/static-scene-validation-20261020/s4-ac-pair-01/{A,C}/run/`。
- 三卷控制日志、预算、状态和固定源码归档：主项目
  `output/static-capacity-20261020/complete-cycle-preflight-01/`、
  `output/static-capacity-20261020/guarded-period-budgets-02/`。

主项目绝对路径为 `/home/lsn/robot/robot_safecontrol`，环境入口 `.scratch/host-recovery/env.sh`。
当前固定运行源码 `504a274740a1e0c27c597c8d61a889fe39154aed`；
S2 历史来源 `40d43bb3712e3f99c11a59750a59d1ee65ecb525`；A/C 配对来源 `7b643c4`。
三卷容量和源码归档 SHA 见 `complete-cycle-preflight-results.md`，
来源归档 `sources.zip` SHA 为 `0096fbdca9da85de6933ac09997b9c4a3764b667072b780af7496412428d72b5`。

## 未完成工作与继续条件

共同周期剩余 18 项没有执行，已有两个失败原件完整保留。
恢复时检查队列是否支持已有原件的接续，不覆盖已有目录，按现有原件判断缺失内容。
当前工程容量十项检查通过，`formal_bound=false`，最终周期和正式清单绑定仍未完成。
继续前重新检查环境、设备身份、源码、三卷可用空间及全部来源 SHA。
完成 #182、正式配置与容量准入后，再进行 #183 的 1080 次 sim 及统计分析。

场景 agent 的未完成串行接续入口保存在独立 worktree
`.scratch/worktrees/static-scenes`，分支 `codex/static-scenes`，
提交 `81c6a5a546a60e15de7745099e417c392884a4c4`，已推送同名远程分支。
只读 prepare 草稿与暂停说明已保存；execute 尚未实现，一项 TDD 处于待实现状态，
仅语法与差异检查通过，不能用作正式准入入口。
该草稿未合入集成分支，继续时按需接续。保留全部 worktree，不启动自动运行。

## 本次资料验证

导出时核对十份实际运行报告与原始数据库身份；S0/S2/S4 重新进行独立评价，
核对保存报告后生成曲线。原始文件在导出前后 SHA-256 一致。
完整反馈和便于编辑的抽样 CSV 分别保存，所有摘要来自完整反馈。
图片检查字体身份、字符覆盖、PNG 编码与尺寸；没有进行人工图像判读。
资料完整性和全量反馈指标重算的实际执行结果见 `proposal-verification.json`。
本次仅增加资料与导出工具，没有变更机器人运行源码，也没有执行新的 sim 采集。
