# 记录生命周期与容量证据

工作目录为 `.scratch/worktrees/static-capacity`，分支
`codex/static-capacity`，本轮开始普通合入集成 `371e063`，
实际 sim 前普通合入 `659bf7c`。
全部运行限定 sim，正式矩阵保持停止，原始数据库和历史验证目录完整保留。

## 实现范围

`ScheduledMotion.next_control_step()` 保留已有未来执行步计算规则，
根据上次已经发起计算的执行步判断是否存在新执行步。
controller 仅为严格递增的 `start_step` 发起计算。
每个新执行步仍使用实际反馈对应的预测状态、参考进度和误差，
保留候选、拒绝、完整周期及恢复事件。

supervisor 按 controller、plant 的顺序启动 worker。
controller 完成资源加载和 JIT 预热后创建 ready 文件；
plant 启动后开始全部逐反馈记录。
初始化时限继续为每个 worker 180 秒，正常退出等待为 20 秒，
SIGINT 后等待为 5 秒，超出等待后保留数据库与 WAL 并终止进程。
配置中的 timeout、settle 继续执行。

`RecordingLimits` 将执行时使用的生命周期参数写入 manifest。
420 秒 timeout、2 秒 settle 加上两个 worker 的退出等待及
1 秒轮询留量，得到 473 秒记录预算。
500 Hz 反馈对应 236,500 条；100 ms 控制周期对应 4,732 条候选。
候选保留全部 trial，包括 `dual` 与 `qp_reused_from_trial`。
这些计数用于后续容量计算，不能单独证明压缩数据库总大小。

worker 在关闭 recorder 前记录 WAL 文件字节和分配字节。
`wal_autocheckpoint=0` 保持有效，记录的数值覆盖该 worker 的完整 WAL。
result 保存这些统计文件的 SHA-256。
验证脚本通过 psutil 每 100 ms 汇总 supervisor 及后代进程的 RSS。
监测进程使用 CPU 16–27，三个 sim 的 worker 独占 CPU 0–15、domain 200。

## 已完成的 RED 与定向验证

`tests/test_static_execution.py` 的新测试使用真实 `MotionPlanner`、
`ExecutionSchedule` 与 `ScheduledMotion`，覆盖拒绝后同一未来执行步
没有重复计算，以及新执行步继续计算。
RED 实际结果为 `1 failed, 1 passed in 7.61s`，
缺少 `next_control_step` 公共接口。
实现后执行与 controller 相关检查为 `3 passed in 21.02s`。
`RecordingLimits` 测试的 RED 为缺少该接口的 ImportError。

完整定向命令：

```bash
source /home/lsn/robot/robot_safecontrol/.scratch/host-recovery/env.sh
PYTHONPATH="$PWD/src:$PYTHONPATH" taskset -c 16-27 python -m pytest \
  tests/test_static_capacity.py tests/test_static_execution.py \
  tests/test_static_experiment_nodes.py tests/test_static_experiment.py -q \
  --basetemp="$PWD/.scratch/lifecycle-complete"
```

终端 session `51852` 实际输出 `26 passed in 58.52s`，退出码 0。
该次 stdout 没有单独保存文件；`.scratch/lifecycle-complete`
保存测试产物。`git diff --check` 和相关 Python compileall 均通过。

真实基线审计使用
`output/static-scene-validation-20261020/s5-current-03`，完整证据保存在
`output/static-capacity-20261020/lifecycle-red-s5/`：

- 949 条 candidate 对应 346 个不同的 `start_step`。
- 按实际反馈数量和 100 ms 周期计算的次数预算为 445。
- plant 的 ready 时间早于 controller。
- 原实验 verdict 保持 pass；本项生命周期验证为 fail。

## 独占 sim 的执行身份

主会话接手执行三项串行验证，脚本为主目录
`.scratch/static-implementation/run_lifecycle_checks.sh`，
日志为 `output/static-capacity-20261020/lifecycle-series-02.log`。
主会话 session 为 `82434`。
启动前场景和方法 agent 的密集进程全部自然结束。
本 worktree 在这三项运行期间保持源码固定。

计划输出目录：

- `lifecycle-s0-01/`：D/S0 完整路径与记录生命周期。
- `lifecycle-s5-01/`：D/S5 独立阻塞证据、规定终止与记录生命周期。
- `lifecycle-timeout-01/`：D/S0、`feed-factor=0.01`、
  `--expect-timeout`，要求真实反馈覆盖完整 420 秒 timeout。

420 秒压力验证使用单独的开发进给，验证最长写入、内存和 WAL。
正式矩阵的三档进给继续为 0.5/1.0/1.5，不使用压力运行代替正式组合。
三项串行脚本退出码为 0。S0 实验 verdict 为 pass，1,721 个候选的
执行步全部不同；最大完整周期 76.324 ms，分配 41,381,888 字节，
WAL 合计 965,410,824 字节，RSS 峰值 817,147,904 字节。

S5 生命周期检查为 pass，346 个候选执行步全部不同，
实验因最大完整周期 83.526 ms 超过 80 ms 预算而 verdict fail。
分配 5,103,616 字节，WAL 合计 175,977,624 字节，
RSS 峰值 824,414,208 字节。

420 秒压力运行按预期 timeout 结束，两个 worker 均退出 0。
保存 209,340 条反馈和 4,185 个不同执行步的候选，
分配 99,287,040 字节，WAL 合计 2,327,833,024 字节，
RSS 峰值 1,319,043,072 字节。
其 `lifecycle-validation.json` SHA-256：
`a83a83243d021d0c080c03677bc1b674ec469bfc70444e905f6f7c9ea0cb1800`。

三项运行的 120 项源码身份完全一致，完整源码已保存于
`output/static-capacity-20261020/lifecycle-source-01/source.zip`，
逐项核对原 manifest 的 SHA-256 均通过。
归档 SHA-256：
`be348ac6cbe9152f3598e277d75d2f4d756484126830e5aac501b4f49690870f`。
`archive.json` SHA-256：
`1d18e3b9310ec8b3396c37fe75cb6ca1cb58e5bf758095fa2a35aabd835b2f11`。
源码归档包含当时的验证脚本；三项数据库保持原件。

## worker 预算执行和生命周期验收

`RecordingBudget` 由每个 worker 持有，plant 在开始新反馈积分、
命令处理前检查，controller 在提交新计算前检查。
截止从实际记录任务开始，controller 的资源初始化等待不消耗记录额度。
473 秒是开始新任务的截止留量；已开始的任务可以在其后完成写入。
计数上限独立于 supervisor 的调度及时性。

关闭 controller 时，已经开始的计算保存完整 candidate、rejection 和 cycle；
尚未完成的 plant 准入检查完成证书记录。额外收尾事件预留固定数量。
worker 写出 `budget_exhausted` 时，supervisor 保存同名终止结果，
`run_formal` 停止后续组合，恢复入口也要求解决预算条件后才能继续。
其他实验失败继续按既有规则保存和统计。

ready 记录自身角色、monotonic 时间、manifest 与源码身份；
result 绑定 ready 及记录统计文件 SHA-256。
生命周期检查要求两个 worker 退出 0、预期终止原因与故障流一致，
并验证每个流的数量和最后任务开始时间。
压力模式明确接受 timeout，其他场景按 completed 或 S5 pause_timeout 验收。

预算接口 RED 为 ImportError；真实 ROS RED 为缺少 worker 预算接口，
结果 `2 failed, 1 deselected in 39.49s`。实现后完整定向命令为：

```bash
source /home/lsn/robot/robot_safecontrol/.scratch/host-recovery/env.sh
PYTHONPATH="$PWD/src:$PWD/portable_oscbf:$PYTHONPATH" taskset -c 16-27 \
  python -m pytest tests/test_static_capacity.py tests/test_static_execution.py \
  tests/test_static_experiment_nodes.py tests/test_static_experiment.py -q \
  --basetemp="$PWD/.scratch/lifecycle-budget-green"
```

实际结果 `30 passed in 95.35s`，退出码 0。
日志 `output/static-capacity-20261020/lifecycle-budget-green.log`，
SHA-256 `9f5a07ede85c58743d46665da59b927a5d54b1571ef67ebe7e25ba202c9404a4`。
本次测试使用 CPU 16–27，其他 agent 同时进行开发 sim，
测试耗时不用于共同周期结论。最终完整周期验证由集成后的独占运行提供。

controller 和 plant 的 pending future 在收尾期间发生真实异常时，
收尾入口仍保存失败 cycle 或 admission、FAULT transition、终止 event
及统计，关闭 ROS 后传播原异常。
测试分别使用真实 `StaticControl.propose(progress_m=-.1)` 和
`certify_motion(minimum_interval_s=-1.)` 的失败输入，全部依赖保持真实对象。
worker 结果文件通过同目录 pending 文件写入及 rename 发布。
命令行验证输出同时显示顶层 status 和 inspection_status。
收尾公共入口 RED 为缺少 `finish_worker_recording`，
日志 `output/static-capacity-20261020/cleanup-red.log`。
合入集成 `1186d0c` 后，同一四份测试文件再次完整执行，
测试目录为 `.scratch/lifecycle-cleanup-green`，
实际结果 `32 passed in 118.71s`，退出码 0。
完整日志 `output/static-capacity-20261020/lifecycle-cleanup-green.log`，
SHA-256 `e1771e88a6069a3adbb3b9c3ba911cc9a5420047a54f8ba7829983c286fe5f7e`。
`git diff --check` 与本次六个 Python 文件 compileall 均通过。

## 容量准入的后续证据

总预算需要分别包含固定 1,080 项、明确的额外重试数量、
当前完整运行及 WAL、串行派生处理、来源归档和系统保留空间。
每个正式运行开始前核查所在文件系统的实际空间。
记录额度变化必须保留已经完成的全部记录，再重新核算后续运行。
不能截断 payload、减少字段或降低 float64 精度。

当前主 ext4 的空间由实际 `df -B1` 复核；另一数据卷
`/dev/nvme0n1p4`、UUID `083CBB753CBB5BFE` 的只读调查可用空间为
133,752,336,384 字节。该卷目前保持卸载，尚未作为正式输出位置。
使用该卷前需要新建独立实验目录并核查真实 SQLite 事务、
SIGKILL 恢复和文件系统空间；已有资料继续保留。
