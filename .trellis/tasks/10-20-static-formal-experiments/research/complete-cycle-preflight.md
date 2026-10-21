# 完整周期计时集成后的 preflight

本文件属于 `.trellis/tasks/10-20-static-formal-experiments`。入口为同目录 `complete-cycle-preflight.py`，由主会话在完整计时修复集成、相关回归完成、独占实验窗口建立后执行。本次准备没有运行脚本、测试或 sim，实际结果必须由生成的证据确认。

## 来源与执行条件

- `--resource-root` 使用 `/home/lsn/robot/robot_safecontrol`。
- `--runtime-revision` 必须传入完整周期计时集成后的 commit。入口在开始及每项命令前后检查 `src`、`portable_oscbf` 相对该 commit 的差异、未登记文件及 `source_identity()`；同时绑定公共验证脚本、输入配置和工程证据的 SHA-256。
- 环境入口为 `.scratch/host-recovery/env.sh`。`PYTHONPATH` 显式包含主仓库 `src` 与 `portable_oscbf`。启动时可读取当前 `robot_static_fcl`，其二进制 SHA-256 进入 `source_identity()`。
- 采用新的 `load_cycle_timing()` 与 `cycle_timing_summary()`，要求 `controller_after_synchronous_records_and_recovery_v1`，并逐字段核对 evaluator 的 timing。历史计时不能通过此入口。
- 入口仅允许三个指定 UUID，分别为 ext4 `03b3e264-cb91-440b-b594-0bdc7c9d190c`、旧 NTFS `083CBB753CBB5BFE`、Windows `6C001EAA001E7B72`。
- 主会话需要确认其他 sim、period measurement 和文件系统验证均已结束。入口自身顺序执行全部命令，文件系统阶段使用 CPU 16–21，sim worker 使用 CPU 0–15/domain 200，monitor 使用 CPU 22–27。全部文件系统阶段结束后开始 sim，两个阶段没有并行执行。
- 默认仅检查准备条件并打印计划；`--execute` 才创建输出、启动公共验证脚本。任一新输出目录已存在时终止，原始记录保持原样。

```bash
source /home/lsn/robot/robot_safecontrol/.scratch/host-recovery/env.sh
export PYTHONPATH=/home/lsn/robot/robot_safecontrol/src:/home/lsn/robot/robot_safecontrol/portable_oscbf${PYTHONPATH:+:$PYTHONPATH}
python /home/lsn/robot/robot_safecontrol/.trellis/tasks/10-20-static-formal-experiments/research/complete-cycle-preflight.py \
  --resource-root /home/lsn/robot/robot_safecontrol \
  --runtime-revision FULL_INTEGRATION_COMMIT
```

将 `FULL_INTEGRATION_COMMIT` 替换为实际集成 commit；准备检查通过并取得实验窗口后，在相同命令末尾追加 `--execute`。脚本直接调用公共验证入口；ROS 启动、进程管理与采样由 `validate_static_lifecycle.py` 负责。

## 数据来源与新目录

原始记录目录：

`/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/guarded-preflight-01/e-s5/run`

真实记录数量为 plant 18,492、controller 1,099，共 19,591。每卷的 guard 与 filesystem 报告记录原始数据库 SHA-256，并检查复制后记录、SIGKILL 恢复及原始文件身份。新入口核对两角色数量和当前 recording 源码身份。

工程输入：

`/home/lsn/robot/robot_safecontrol/output/static-capacity-20261020/three-volume-capacity-01/engineering-plan/engineering_evidence.json`

SHA-256：`2119aa8150c5335858649f0066a08f6bd1a89b56426530b6c68b48cfae020422`

每项预算调用当前 `prepare_static_storage_budget.py`，通过上述证据中的 `audit_sha256` 校验原始 audit，并按当前 `engineering_model()` 与 `RecordingLimits` 重建。完整计时 sidecar 的数量上限、allocated bytes 和收尾保留空间进入当前模型。

新目录如下；控制计划、命令日志、预算、状态、汇总与源码归档保存在 ext4 目录。

| 卷 | 新输出目录 |
| --- | --- |
| ext4 | `/home/lsn/robot/robot_safecontrol/output/static-capacity-20261020/complete-cycle-preflight-01` |
| 旧 NTFS | `/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/complete-cycle-preflight-01` |
| Windows | `/media/lsn/Windows/robot_safecontrol_static_experiments_20261021/complete-cycle-preflight-01` |

每卷具有 `recording-guard/`、`filesystem/`、`d-s5/run/`。旧 NTFS 另有 `d-s0/run/`、`d-s4-initial3-feed0.5/run/`、`d-timeout/run/`、`d-wal-limit/run/`。各 run 保存原始 SQLite、worker 日志、manifest、result、ready、recording JSON 与 feedback report。

## 验证项目与判据

| 项目 | 覆盖与判据 |
| --- | --- |
| 三卷离线验证 | 每卷调用 `validate_static_storage_guard.py` 和 `validate_static_filesystem.py`；使用全部 19,591 条真实记录，核对精确值、随机读取、SIGKILL 恢复、atomic JSON、当前 recording 源码 hash 和卷 UUID |
| 三卷 D/S5 | 使用 `development-scenes-07/s5.yaml`、`s5_blockage.json`，66 pairs，预期 `pause_timeout`；worker 均正常退出，生命周期、存储预算与功能检查通过 |
| 旧 NTFS D/S0 | 使用 `development-scenes-07/s0.yaml`，56 pairs，预期 `completed`，功能通过 |
| 旧 NTFS D/S4 | 使用已具备完整恢复证据的 `s4-configurations/s4-initial-3-feed-0.5.yaml`，76 pairs；配置已经编码 nominal feed 0.025，额外 `feed_factor=1`；使用 `s4-paired-independent-witness-01/witness`，预期 `completed`，功能通过 |
| 旧 NTFS 420 秒 timeout | S0，`feed_factor=0.01`，预期 `timeout`；要求 `full_timeout_recording_verified`，保留功能失败与实际 worker 退出状态 |
| 旧 NTFS 8 MiB forced-WAL | S0，两个 writer 的 WAL 额度各为 8,388,608 bytes，预期 `budget_exhausted`；保留 validation 失败状态，要求实际终止 event、预算原因、完整 sidecar、统计 SHA 与收尾空间检查成立 |

每个 run 均通过新 loader 检查 SQLite cycle sequence 与完整计时一一对应、边界版本、result 绑定的 sidecar SHA-256，以及基础计时与完整计时的一致性。完整 timing 还包含由真实 plant admission 记录按 `command_id` 去重后的 `admission_rejections`；相同 `command_id` 使用最后一条记录，与 evaluator 的处理一致，再对完整 timing 逐字段核对。入口同时检查统计文件实际 allocated bytes、每类记录大小、WAL 写入上限与 completion reserve。

正常项目允许 runtime timing 为 `fail`，并在 `runtime_timing_statuses` 中独立显示；此时生命周期通过仅证明采样、保存和结束过程符合要求。完整周期计时是否满足 P99.9 阈值必须单独判定。timeout 与 budget_exhausted 保存为预期失败，不能报告功能通过。原始证据及持久文件校验值保存在 `status.json` / `completed.json`；持久文件包括 `.sqlite3`、`.json`、`.ready`、`.yaml`、`.log`，覆盖数据库、manifest、result、report、ready、recording、配置和日志。异常终止时状态为 `incomplete`，日志与已生成证据保留。

## 工程容量重建输入

全部验证完成后，入口生成 `source/sources.zip`、`source/snapshot.json`、`source/environment.json` 和 `capacity-inputs.json`。容量命令写入 JSON，执行状态为 `executed=false`；主会话核对证据后执行其中的命令。

容量输入为：

1. 上述固定工程输入的 `audit_sha256` 对应 audit 文件及其全部原始数据库；沿用原始记录的身份校验。
2. 新生成的六份生命周期报告：三个卷 D/S5，旧 NTFS D/S0、D/S4、420 秒 timeout。forced-WAL 报告作为补充证据保存。
3. 三份新 filesystem 报告，必须绑定当前 `static_experiment_recording.py` hash；三份 guard 报告作为补充证据保存。
4. 新源码归档 `source/sources.zip`，以及当前 `development-scenes-07/s0.yaml`。
5. 输出卷目录：ext4 `formal-runs-ext4`、旧 NTFS `formal-runs`、Windows `formal-runs`。这些目录必须仍对应上述 UUID。
6. `--retry-runs 0`；容量输出为 ext4 新目录内的 `engineering-plan/`。此阶段没有绑定正式清单，`formal_bound=false`。

当前模型的新统计容量必须进入重建后的工程证据。工程容量通过、完整周期共同周期验证通过及其余规格前置条件通过后，主会话才能继续正式 sim 采集。
