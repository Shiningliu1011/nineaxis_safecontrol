# 第三卷验证入口

2026-10-21：第三卷离线记录验证、完整 D/S5 生命周期和三卷工程容量检查
已经完成。详细数值、来源身份和当前边界见
[三卷容量验收记录](three-volume-capacity-acceptance.md)。正式清单绑定保持待完成。

## 目录与来源

- 第三卷：`/dev/nvme0n1p3`，UUID `6C001EAA001E7B72`，`ntfs3`。
- 唯一写入范围：`/media/lsn/Windows/robot_safecontrol_static_experiments_20261021/`。
- 准备时正常挂载选项含 `rw`；主会话此前读取空闲空间为 77,810,987,008 字节。
- 原始记录：第二卷 `guarded-preflight-01/e-s5/run`，使用其完整 controller、plant 数据库及配置。
- 执行使用主仓库 `.scratch/host-recovery/env.sh`，运行内容固定为 `40d43bb`。
- 工程输入：主仓库 `output/static-capacity-20261020/guarded-capacity-review-01/engineering-plan/engineering_evidence.json`，SHA-256 为 `18f01599d7876f269ec683639365b546ed264aa26223187fc7f004f6117ecd34`。

脚本只调用现有公共入口，生产模块保持原状。执行时检查卷 UUID、`rw`、
运行代码差异和输出路径未占用；任何命令失败即退出，保留已经生成的全部文件。
源数据库的前后 SHA-256、全部字段和 float64 精度由既有验证器核查。

## 已执行命令

主会话完成完整回归后，在 CPU 16–21 执行：

```bash
taskset -c 16-21 ionice -c 3 bash \
  /home/lsn/robot/robot_safecontrol/.scratch/worktrees/static-capacity/.trellis/tasks/10-20-static-formal-experiments/research/third-volume-validation.sh \
  --execute \
  /home/lsn/robot/robot_safecontrol \
  /media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/guarded-preflight-01/e-s5/run \
  /home/lsn/robot/robot_safecontrol/output/static-capacity-20261020/guarded-capacity-review-01/engineering-plan/engineering_evidence.json
```

将 `--execute` 改为 `--check-only` 可读取路径、挂载及源码状态，检查期间不写入文件。
现有输出完整保留，再次使用相同输出路径会被拒绝。

## 验证内容与输出

1. `prepare_static_storage_budget.py` 生成第三卷 UUID 与当前源码绑定的 E/S5 开发预算，
   保持 66 个几何 pair、原始运行配置、全部字段、5% 编码裕量和收尾空间。
2. `validate_static_storage_guard.py` 完整逐条重写 controller 和 plant，比较
   msgpack 字节摘要；随后每个 writer 使用 8 MiB WAL 额度，保留触发记录及
   `budget_exhausted` 终止事件，以 SIGKILL 终止进程并精确恢复已提交前缀。
3. `validate_static_filesystem.py` 完整重写两个数据库并逐值比较，保存 float64
   位模式摘要，检查每个 stream 的第 1、127、128、129、256、300 及最后记录，
   执行每个 writer 至多 1,200 条 SIGKILL 恢复（记录较少时使用全部记录），验证同目录 pending JSON rename
   后的内容。该报告可由现有工程容量生成器直接读取。

第三卷输出为 `storage-guard-01/` 和 `filesystem-01/`。
主仓库输出为 `output/static-capacity-20261020/third-volume-validation-01/`，
包含确切命令、脚本副本、输入及结果 SHA-256、Git 提交、CPU、时间、
卷身份、执行日志、退出码及前后空间。第二卷原始记录全部保留。
此次离线命令不启动 ROS，也不测量完整控制周期。

## 工程容量证据

主会话在第三卷执行启用容量守卫的完整 D/S5 sim，保存实际来源、记录额度、
WAL、RSS、退出状态和生命周期报告。三卷工程方案使用现有十九组审计、
第二卷五份 D 生命周期、ext4 生命周期、第三卷新生命周期、
两卷既有文件系统报告及第三卷 `filesystem-01/validation.json`。

`plan_static_engineering_capacity.py` 接收三个独立 `--volume`。第三卷须完整
计入自己的 WAL 工作空间及保留空间，派生处理和来源归档按现有规则计入；
1,080 项矩阵、零额外重试和 5% 编码裕量保持。生成时重新读取三卷现场空间，
检查各卷分配和总预算。结果继续明确 `formal_bound=false`，最终正式准入
依赖完整来源、共同周期及现场空间检查。

脚本 `bash -n`、实际准备检查和 `git diff --check` 均退出 0。
运行模块与 `40d43bb` 内容相同，实际执行退出 0。
第三卷完整 D/S5 由主会话既有 `.scratch/static-implementation/run_guarded_windows_s5.sh`
执行，本文脚本仅负责离线文件系统验证。

## 三卷工程容量重建入口

`rebuild-three-volume-capacity.py` 保存全部十一项 guarded 报告身份，使用其中
五份完整 D 运行、ext4 生命周期及第三卷完整 D/S5 作为共同控制参数证据。
全部 A/B/E 和强制 WAL 终止报告保存在输入身份清单；现有容量生成器的
生命周期参数要求与 D 的共同控制配置相同。第三卷 `storage-guard-01` 的
双 writer、SIGKILL、收尾余量和源码身份也必须通过执行前检查。

第三卷三份结果未齐备时，脚本输出 `pending_evidence`，退出 2，不创建输出。
默认模式只核查报告和挂载；`--execute` 才调用现有容量生成器。完整执行会
沿用十九组编码审计，并由生成器核对原始数据库身份，保持原始数据库内容。

```bash
source /home/lsn/robot/robot_safecontrol/.scratch/host-recovery/env.sh
taskset -c 16-21 ionice -c 3 python \
  /home/lsn/robot/robot_safecontrol/.scratch/worktrees/static-capacity/.trellis/tasks/10-20-static-formal-experiments/research/rebuild-three-volume-capacity.py \
  --resource-root /home/lsn/robot/robot_safecontrol --execute
```

结果写入主仓库 `output/static-capacity-20261020/three-volume-capacity-01/`；
三卷分配位置为原有 `formal-runs-ext4/`、第二卷原有 `formal-runs/`，以及
第三卷新建的 `formal-runs/`。输出路径存在时拒绝覆盖。
每卷完整计入 WAL 与 reserve，其他预算参数沿用现有生成器。
结果保持 `formal_bound=false`，正式采集继续依赖后续正式清单绑定与准入。

入口 AST 语法和差异检查通过，缺失证据路径曾实际验证为退出 2 且不创建输出。
三份报告齐备后，主会话实际执行工程生成器，十项检查全部通过、退出 0。
本次记录更新只读取小型报告及其 SHA-256，未重跑审计或 sim。
