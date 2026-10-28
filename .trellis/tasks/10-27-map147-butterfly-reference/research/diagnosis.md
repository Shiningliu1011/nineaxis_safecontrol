# 评价临时目录检查

采用 diagnosing-bugs 方法核验实际失败。main 的首次直接消费者检查得到 89 passed、1 failed，日志为 `.scratch/map147-152/integration/consumers.log`；唯一失败为 `test_actual_processed_motion_and_rejected_admission_have_reconstructable_progress`。数值、曲线、记录恢复及前面的执行评价断言已经通过，失败发生在真实 `evaluation_samples` 创建临时目录时。

`static_experiment_evaluation.py` 的公开入口使用 `resource_root/.scratch/static-experiments`，没有环境变量替代入口。当前路径是指向外部 `/media` 研究目录的既有符号链接；目标未挂载，`mkdir(parents=True, exist_ok=True)` 因此取得 `FileExistsError`。已直接读取链接身份和该函数，不需要增加生产诊断或数值探针；多假设与代码修订阶段在这个已确定的文件系统条件下省略。

验收采用可恢复的本地目录准备：保存原符号链接，使用当前仓库 `.scratch` 内目录执行同一公开入口及测试，完成后恢复原链接。所有目录和命令记录在 `integration/run_with_local_evaluation.sh`；不写入外部目标。首次失败继续保留，后续结果使用独立日志。

## 绘图 pytest 启动环境

完整主包逐文件运行时，四个绘图文件使用现有 `.scratch/pipe-assets-runtime/venv`。该进程继承了 ROS 的 `PYTHONPATH`，pytest 读取 ROS 插件 metadata 并导入 `launch_testing`，随后在该绘图环境内缺少 yaml，四个进程在收集测试前退出。日志为 `integration/full/logs/main-test_pipe_nurbs.log`、`main-test_pipe_painter.log`、`main-test_pipe_painter_events.log`、`main-test_pipe_pose.log`。

四个文件使用同一绘图 Python、Matplotlib 3.10.9 和 Axes3D，`PYTHONPATH` 仅包含 main `src` 与证据插件；该纯绘图环境不加载 ROS metadata。完整重跑得到 35 passed（8、3、11、13），命令、环境与真实阶段结果为 `integration/full/runs-plot-retry.json` 和 `results/plot-retry-*.json`。Matplotlib 与 Axes3D 的实际模块路径均位于该 venv，见 `plot-environment.json`。源码与测试断言保持原内容，首次失败日志继续保留。

## modular 相对计时

完整内核运行中，`test_modular_jit_matches_current_semantics_baseline` 的 3000 步数值误差、进度、QP 成功、单个 JIT 缓存及初始化时间断言全部通过，最后相对计时为 `13.186950948379993 ms > 6.4518506048943145 ms × 1.2`。首次日志与阶段结果为 `integration/full/logs/kernel-test_modular_jit_equivalence.log` 和 `results/kernel-test_modular_jit_equivalence.json`。

原基准是同轮公开 `test_baseline_tracking` 实际生成的 `integration/full/portable-output/baseline_tracking.npz`，SHA256 为 `40bdab1e26b1594a227e851e93cacac1856591969cec704076456ccd415a916f`。复核保持其字节与相对门槛，通过新输出目录中的本地链接读取原基准，不重新生成。

采用 diagnosing-bugs 的原用例作为反馈入口，原始命令和断言已经产生一次明确失败。该入口的 3000 步是既有比较要求，保持原步骤与预算；生产数值路径不增加日志。依次核验以下原因：

1. CPU 调度或频率变化：若它影响此计时，限定既有 P-core 集合后，同一基准的耗时与原断言结果应变化。
2. 同时运行的数值负载：若它影响此计时，其他任务自然完成并暂停后，原全部 CPU 条件应取得变化；记录测试同期进程、负载和 CPU 频率。
3. 调用本身的性能差异：若相同源码、基准和受控 CPU 条件下仍超过 120%，保留失败并交由实现 owner 修订，继续保持原门槛。

完整功能批次结束后，顺序执行 controller 与 modular 两类隔离计时，再交接 #101 的独立查询计时，避免测试间相互争用。每次 pytest 记录实际进程 CPU affinity、环境、源码 SHA256、原始结果和测试期间采样；复核结果以本次实际日志为准。

## 四个物理 P-core 对照

main 的 controller 全 CPU 为 300 步、p95 `28.08213394600898 ms`、229 步超期；八个 P-core 为 300 步、p95 `18.511266743735177 ms`、4 步超期（1.33%）。两项原测试均未通过，完整原结果保持。

在这两项和 modular 对照完成后，预先声明下一假设：小维度控制计算的线程调度与同步成本随允许的 CPU 集合变化。如果此项影响结果，将 CPU 集合从八个 P-core 改为四个物理 P-core（`0,2,4,6`），在相同源码、ROS executor 四个线程、控制参数和原 20 ms/1%/300 步门槛下，耗时和超期数量应相应变化。仅执行一次该条件的对照；同期记录进程线程数量、频率、负载、实际 affinity，结束后保存进程状态。没有更改系统调频或其他用户进程，也没有重复相同条件等待通过。

该一次对照为 1 passed：303 步，p95 `14.342699509870727 ms`，1 步超期（0.330033%），最大单步 `23.541554997791536 ms`。进程实际 affinity 为 `[0,2,4,6]`；同期 `ps` 记录线程数量，结束状态保存 25 个仍存在的线程。证据为 `integration/full/perf-p-four/` 的实际报告、`process.json`、`process-after.json` 和 `environment.json`。此结果表明 CPU 集合会改变当前门槛结论，适用条件完整保留。

modular 的全 CPU 独立复核仍失败（14.938 ms）；八个 P-core 原测试通过（6.379 ms、初始化 27.36 s、3000 步全部数值差异为零、JIT 缓存 1）。两个目录均验证原 baseline SHA256 未变。所有计时结束后，数值窗口交给 #101 的独立查询对照，本任务仅整理轻量记录与最终文件身份。
