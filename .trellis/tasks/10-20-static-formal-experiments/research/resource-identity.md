# 正式可行性资源身份

2026-10-20，`freeze_formal` 将可行性来源与当前正式配置统一交给
`_physical_identity(manifest, resource_root)`，使用已有 `relocate_manifest`
按明确的来源根目录重定位。完整场景、初态、执行器限制、轨迹和运动学身份
继续参与比较；碰撞来源、辅助文件、全部 mesh、轨迹和运动学配置逐项核对内容 SHA-256。
内容发生变化时立即拒绝。来源 manifest 和数据库保持原内容。

修改范围为 `static_formal_experiments.py` 的物理身份及 freeze 调用点，
独立测试文件 `tests/test_static_formal_resources.py` 和使用文档。
`run_formal`、容量检查、场景生成和见证代码保持原有实现。

## 实际来源

来源运行：
`/home/lsn/robot/robot_safecontrol/output/static-ticket-evidence-20261020/s1-current-01`。
该次实际运行的 resource_root 为
`/home/lsn/robot/robot_safecontrol/.scratch/worktrees/static-evidence`，
当前资源目录为 `/home/lsn/robot/robot_safecontrol`。
源 manifest 原件复制为 `tests/data/static_s1_manifest.json`，
其 SHA-256 与原件一致。

| 文件 | SHA-256 |
|---|---|
| `manifest.json` | `0983163905f0c4e5303ed688ee60dba683b9928fa558bbfe9bdb92894612a86e` |
| `feedback_report.json` | `546b4ec4b6832bcfc9be29aaa152f4bde42ff62d20aa8b316adee8607682dc6c` |
| `controller.sqlite3` | `335b3915bc6b6934b83141ae6667915f7fa3c88e4a881f9365bef094a874c7e4` |
| `plant.sqlite3` | `de155ad611342967eaa654ec1ad5a77b80f61a620fa9b42719303e7fda17527c` |

## 验证

输出目录：
`/home/lsn/robot/robot_safecontrol/output/static-method-evidence-20261020/formal-resources/`。

`tests/test_static_formal_resources.py` 使用真实 manifest 与模型资源副本，
核查重定位后的相同物理身份、原输入不变、十个 mesh 和全部 36 个组合。
分别修改副本中的碰撞来源文件、mesh、轨迹和运动学内容时，四项检查均拒绝；
实际 S0 与 S1 场景继续具有不同物理身份。
独立测试和已有正式编号、资源重定位测试合计 **8 passed，7.70 s**。
TDD 与最终日志分别为 `red.log`、`green.log`、`final.log`。
合并集成提交 `371e063` 后，包含资源重定位与新版记录读取依赖的相同检查
再次通过，**8 passed，7.86 s**，日志 `integrated.log`。

随后通过 `validate_run_report` 核查上述真实 S1 报告及原始数据库，
将其与主目录的同一 `source_configuration.yaml` 重新生成的 manifest 比较：
`physical_identity_matches=true`，十个 mesh 和全部 36 个组合保留；
原始四份文件 SHA-256 前后相同。
`verification.json` SHA-256：
`82f7ddcc62b56988fc3f08619905d9dc31a0611207a4dc70cea97561ebf2b3f1`。

环境入口仍为 `.scratch/host-recovery/env.sh`，离线验证限制在 CPU 16–25。
命令保存为 `run_formal_resource_checks.sh` 与 `verify_formal_resources.py`。
本次验证覆盖 freeze 使用的物理身份比较及真实资源，
完整正式 freeze 仍等待场景、方法、容量和共同周期前置验收。
