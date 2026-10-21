# 完整周期共同周期 study02

当前任务为 `.trellis/tasks/10-20-static-formal-experiments`。入口 `complete-cycle-period-study.py` 基于集成 commit `3771bc8` 准备，调用公共预算 producer 和 benchmark CLI。此次仅完成调度入口及小型准入检查，没有执行预算 producer、benchmark 或 sim。

## 运行入口

```bash
source /home/lsn/robot/robot_safecontrol/.scratch/host-recovery/env.sh
export PYTHONPATH=/home/lsn/robot/robot_safecontrol/src:/home/lsn/robot/robot_safecontrol/portable_oscbf${PYTHONPATH:+:$PYTHONPATH}
python /home/lsn/robot/robot_safecontrol/.trellis/tasks/10-20-static-formal-experiments/research/complete-cycle-period-study.py \
  --resource-root /home/lsn/robot/robot_safecontrol \
  --runtime-revision FULL_INTEGRATION_COMMIT
```

`FULL_INTEGRATION_COMMIT` 使用实际集成 revision，并在准备和执行时保持相同值。

- 默认模式：当前完整计时 preflight 和工程容量通过后，生成全部 20 项预算，检查身份及完整 study 空间，保存 `prepared.json`，随即结束。
- `--check-only`：读取准入材料并报告准备状态；不创建预算目录、不启动 sim。
- `--execute`：执行准备检查，必要时生成预算，然后串行启动完整 benchmark。已有 `prepared.json` 仅在计划、全部输入 SHA-256 和预算身份完全一致时使用。
- 新工程证据尚未生成时，返回 `pending_capacity`、缺失路径和完整 20 项计划，退出码为 2；不会开始预算或 sim。
- `study02` 已存在时，拒绝再次启动，保留已有全部数据。执行中失败则保存 `incomplete` 与原始日志；主会话根据实际失败决定后续处理。

准备和执行均需要主会话确认独占实验窗口。文件系统验证、其他 sim 与本次 benchmark 不能重叠。预算 producer 使用 CPU 16–21；benchmark 及其 worker 使用 CPU 0–15/domain 200；入口及等待进程使用 CPU 22–27。入口保持公共 benchmark 的顺序调度。

## 必需前置证据

固定来源为：

`/home/lsn/robot/robot_safecontrol/output/static-capacity-20261020/complete-cycle-preflight-01`

入口要求其中的 `plan.json`、`completed.json`、`engineering-plan/engineering_evidence.json` 和 `engineering-plan/capacity_plan.json` 完整存在，并检查：

1. 当前 `src`、`portable_oscbf` 与 `--runtime-revision` 无差异，无未登记源码；当前 `source_identity()` 包括 native 二进制身份。
2. 工程 evidence 与 capacity plan 的来源均为当前生产源码；两者均为 `pass`，全部 checks 通过，shortfall 为零，`formal_bound=false`；输入 SHA-256 一致。
3. preflight 为 `completed`，七项完整，正常项目功能为 `pass`；全部运行声明当前完整计时边界且具备 `eligible_for_period`。preflight runtime timing 的数值结论独立保留。
4. NTFS UUID 为 `083CBB753CBB5BFE`，以读写方式挂载。
5. 配置、工程证据、preflight 汇总、入口脚本、公共 producer/benchmark 脚本和环境文件均绑定 SHA-256；预算准备前后与 benchmark 前后重新检查。

入口使用 `development-scenes-07/s4.yaml`，maximum pairs 为 76。公共 `prepare_static_storage_budget.py --study-index` 根据新工程 evidence 生成全部周期与方法预算，完整矩阵为：

| period | methods |
| --- | --- |
| 20 ms | A、B、C、D、E |
| 40 ms | A、B、C、D、E |
| 50 ms | A、B、C、D、E |
| 100 ms | A、B、C、D、E |

每份预算检查实际配置 SHA、source SHA、工程 evidence SHA、卷 UUID、run_id 与 `development_only`。整体空间要求包含全部 20 项保留数据、最大单次 WAL、最大 reserve、派生与来源归档工作空间。空间不足时保留准备结果并终止。

## 新数据与结果

预算和控制记录目录：

`/home/lsn/robot/robot_safecontrol/output/static-capacity-20261020/guarded-period-budgets-02`

保存 `plan.json`、20 份 `.storage.json`、`index.json`、`prepared.json`、`preparation.log`、`benchmark.log` 与 `execution.json`。

全部 sim 原始数据目录：

`/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/guarded-period-study-02`

公共 benchmark 保存 `study.json`、`period_report.json`、20 个 run、逐 run 的 `.process.json` 与日志。完整运行后，入口逐个调用当前 `load_cycle_timing()`，核对新的 sidecar 身份，保存各项失败状态、功能判据及持久文件 SHA-256。SQLite、manifest、result、report、ready、recording、配置与日志均保留。

共同周期报告必须包含完整 20 项并采用 `controller_after_synchronous_records_and_recovery_v1`。公共 benchmark 按完整计时计算各方法 P99.9，并根据全部方法选择最短满足预算的周期。入口要求 benchmark 退出码与最终报告一致；`fail` 返回 1。异常导致测量不足 20 项时，入口保存失败数据和 `incomplete` 状态，正式采集继续停止。

原 `guarded-period-study-01`、`guarded-period-budgets-01`、既有 preflight 和全部原始数据库保持原样。历史计时结果不能进入 study02 的准入结论。

## 已完成验证及范围

`test_complete_cycle_period_study.py` 使用真实仓库与既有 JSON 证据，不使用 mock，不启动 producer 或数值任务。以下四项均已观察 RED/GREEN：

- 实际旧 revision 与当前源码不同，入口在准备前拒绝。
- 当前工程容量文件缺失时，入口保持等待，并输出规格规定的 20 项计划、CPU 和 domain。
- 实际旧工程容量来源无法准入当前生产源码。
- 实际 study01 的计时边界无法提供新版完整周期证据。

执行命令：加载 `.scratch/host-recovery/env.sh` 后运行同目录 `test_complete_cycle_period_study.py -v`。预算生成成功路径、当前完整证据的准备路径和全部 20 项 benchmark 执行路径，需要主会话在新 preflight 与容量通过后运行验证。
