# 静态实验紧凑存储

本文说明现有记录格式和历史容量测量，用于读取、验证和复现已有数据。项目当前清单与容量要求由[完整项目验收任务](planning/nineaxis-full-project/dependencies.md)管理。

新运行使用 SQLite 格式 3；旧 JSON 和格式 2 数据库保持可读。安装记录依赖：

```bash
python -m pip install -r requirements-static-recording.txt
```

## 保存与恢复

`static_experiment_recording.py` 统一管理写入、流读取和按 sequence 读取。
`samples` 保留 sequence、stream、monotonic_s、simulation_s 和 ros_time_ns。
payload 使用 MessagePack 双精度数值，每条分别执行 zlib level 6 和
zstandard level 3 无损压缩，保存较短结果，codec 列记录对应编码。
两种编码均使用同一 stream 的前一条完整内容：zlib 字典最多 32 KiB，
zstandard RAWCONTENT 字典最多 128 KiB，每 128 条独立开始。
全部字段和 float64 位模式保持一致。
keyframe 列指定该段起点，随机读取只需恢复对应 stream 的当前段。
重复状态参与字典压缩，原有字段和事件均可完整还原。

每次 append 仍以独立 SQLite 事务提交；提交成功后更新内存字典。
WAL 自动 checkpoint 继续关闭，采样结束后执行 TRUNCATE checkpoint。
中断恢复必须同时保留数据库和 WAL，使用 `finalize_recording()` 完成完整性检查。
已经提交的记录可在写入进程运行期间读取。`synchronous=NORMAL` 保留原有
进程异常恢复语义；本次 SIGKILL 验证不代表主机断电时最后事务仍能保存。

格式通过 `PRAGMA user_version` 判断：0 为旧 JSON，
2 为 MessagePack/zlib level 1，3 为当前编码。
未知版本、非有限写入、缺少字典、压缩内容损坏和不存在的指定记录均报告错误。
新写入目标必须不存在；异常运行保留记录，批量实验沿用原有未完成状态管理。

## 原始运行转换

转换只向新目录写入，原始目录继续保留：

```bash
source .scratch/host-recovery/env.sh
python scripts/static_obstacle_experiment.py compact \
  --source output/static-experiments/s0-joint-full-geometry-01 \
  --output output/compact-s0 --resource-root "$PWD"
python scripts/static_obstacle_experiment.py replay \
  --output output/compact-s0 --resource-root "$PWD"
```

转换入口接受格式 0 或 2，输出格式 3。
`storage_migration.json` 保存源目录、全部源文件 SHA-256、原始 manifest、
各流数量、逐条内容身份以及转换数据库 SHA-256。原有报告位于 `source_reports/`。
升级格式 2 时保留它的历史来源报告，并将直接来源的报告与迁移说明保存到
`source_reports/format-2/`。
旧报告已经保存的身份字段必须匹配；缺少的历史身份字段单独列出，转换后
重新评价并生成绑定 manifest、result、数据库和迁移说明的完整报告。
重放和报告读取均核对转换说明中的数据库、manifest、result、path 身份；
任一内容变化时立即拒绝，不重写已有报告。
原始 manifest 中的绝对路径和来源身份保留，读取资源使用既有重定位接口。
转换命令成功仅表示转换与重算完成；具体实验结论仍读取报告的 verdict。

## 派生数组与报告

评价默认完整计算全部指标，只保存汇总报告，`derived_data_policy=on_demand`。
三组逐反馈数组在请求保存或绘图时生成。显式保存：

```bash
python scripts/static_obstacle_experiment.py replay \
  --output output/compact-s0 --resource-root "$PWD" --save-samples
```

绘图通过项目 `.scratch/static-experiments/derived-*` 中的独立目录重算数组，
逐项核对汇总指标与已保存报告相同后绘图，并清理本次中间文件。
原始运行报告身份保持不变。`figures.json` 保存使用的数组、报告、字体和图片身份。
`evaluation_provenance` 保存实际生成数组的评价程序及源码身份；
使用历史已保存数组时保留原报告提供的身份，未提供的历史字段记为 null。
如果当前评价程序重算的指标发生变化，绘图会拒绝继续，需要核查后执行 replay。
已有运行中的派生文件不会被自动删除。

独立绘图 Python 环境须安装 `requirements-static-report.txt`、
`requirements-static-recording.txt`、`portable_oscbf/requirements.txt`，并构建安装
本项目的原生 FCL 模块；按需重算需要同一组评价依赖。
调用前加载 ROS 2 Humble 和项目 `install/setup.bash`。
`--report-python` 保留虚拟环境中的 Python 路径，包括符号链接。

## 验证入口

```bash
python -m pytest tests/test_static_recording.py tests/test_static_campaign.py \
  tests/test_static_execution_evaluation.py -q
python scripts/validate_static_storage.py \
  --output output/static-storage-validation --resource-root "$PWD"
python scripts/validate_static_recording_upgrade.py \
  --source-root output/static-storage-validation-05 \
  --output output/static-recording-upgrade --resource-root "$PWD"
```

真实验证默认处理接手包中的完整 S0、S2、S4、S5 记录，输出目录必须不存在。
验证包含逐字段与 float64 位模式比较、跨段随机读取、全部流数量和时间字段、
真实 SIGKILL 已提交记录恢复、旧格式和新格式的独立指标与派生数组重算，
以及读写耗时、目录容量、WAL 处理峰值和内存峰值。
历史机器生成的派生数组与当前重算的最大差值另外保存。
升级验证针对四组完整格式 2 来源，逐条检查所有字段及 float64 位模式，
比较同机独立重算的全部指标与派生数组，并对每个数据库执行 1,200 条
已提交记录的真实 SIGKILL 恢复。升级验证全程使用精确比较。
编码研究工具 `scripts/benchmark_static_codecs.py` 将对照数据库标记为格式
100，生产读取入口拒绝读取这些实验文件。

容量和耗时结论以[接手记录](static_obstacle_experiments_handoff.md)所指向的
历史测量为准。后续复用时核对对应源码、配置和记录身份。

## 正式矩阵容量准入

controller 完成初始化和 JIT 预热后启动 plant。首次 plant 反馈开始后，
supervisor 使用配置中的完整 timeout，退出继续使用每个 worker 的
20 秒正常等待和 5 秒 SIGINT 等待。manifest 中的 `recording_limits`
统一保存这些生命周期参数，以及记录时长、反馈数量和候选数量预算。
420 秒 timeout、2 秒 settle 对应 473 秒记录留量，其中包含两组退出等待
和 1 秒轮询留量。

controller 对同一未来 `start_step` 只发起一次计算，新执行步继续计算
实际误差和恢复条件。全部实际反馈仍逐条保存。
在 100 ms 控制周期下，473 秒记录留量对应最多 236,500 条反馈和
4,732 条候选；每条候选保留至多九次 trial 的全部字段。
plant 在新反馈积分与命令处理前检查本地 `RecordingBudget`，
controller 在新计算前执行相同检查。各 worker 自身执行次数和 monotonic
截止检查。截止时停止开始新任务，已经开始的计算继续保存完整候选、
拒绝、周期和收尾事件，因此最后写入的完成时间可以晚于截止时间。
manifest 的 `stream_record_limits` 逐流记录上限包含固定收尾额度。
预算耗尽时保存 `budget_exhausted` 结果，正式批次停止后续运行，
已有原始记录继续保留。
每个 worker 退出时保存 `*.recording.json`，记录 checkpoint 前的 WAL
字节、分配字节、任务计数和开始时间。ready 文件包含 monotonic 时间、
manifest 与源码身份，result 绑定 ready 和统计文件的 SHA-256。
controller 统计文件另存版本化的完整周期时间表，以 SQLite cycle sequence
逐项关联；计算失败和终止阶段也需要对应记录。每项固定包含 sequence、
起点、终点和 duration，按 256 字节预留，其他固定统计字段预留 65,536 字节，
总额向上取整至 4,096 字节。100 ms 周期最多 4,732 项，controller 预留
1,277,952 字节；20 ms 周期最多 23,652 项，预留 6,123,520 字节。
plant 统计文件预留 65,536 字节。工程预算明确将这两个额度加入每个运行的
保留文件和 writer 收尾额度，内存表使用相同的记录数量上限。
统计发布前验证 UTF-8 JSON 尺寸、文件系统分配单位和实际剩余空间；同目录
pending 文件写完后验证实际分配，再原子重命名并检查已发布文件的收尾增加量。
统计文件缺失或超出已声明空间时保留数据库并报告失败。
收尾中的计算或准入 future 发生异常时，继续保存已有周期、故障事件
和统计，关闭 ROS 后传播原异常。worker 结果使用同目录临时文件
写完后重命名发布，supervisor 只读取完整 JSON。

`static_experiment_capacity.py` 分别计算全部计划运行、保留的失败重试、
单次运行的 WAL 工作空间、串行派生处理、来源归档和系统保留空间。
各项使用字节数；固定矩阵已经包含每个组合的三次重复。
额外重试预算通过 `--retry-runs` 显式提供，增加重试需要重新核算。
恢复批量运行时，已有目录继续保留，只为尚未运行的组合预留新记录空间。
每个新运行启动前检查当前文件系统可用空间，已有运行超过记录容量上限时停止批量执行。

正式目录需要 `capacity_plan.json`，其身份绑定 `formal_manifest.json`、
代码来源和原始验证证据。准入接受经过验证的完整容量上界，或者具有完整验证
范围、逐流次数限制和逐次空间检查的工程预算。
单纯样本外推即使小于可用空间，也会保持证据不足。
容量文件保存 `CapacityBudget` 的各项数值、
`is_validated_capacity_upper_bound`、`is_validated_engineering_budget`、`evidence_sha256`、
`formal_manifest_sha256` 和 `source_sha256`。
证据文件或清单身份变化时，需要重新验证容量。

`scripts/audit_static_record_envelopes.py` 审计全部实际字段、trial 数量、几何组合、
矩阵 shape、keyframe/delta 编码大小及 SQLite 页空间。旧格式在独立输出目录
转换并核查完整记录身份，原件继续保留。`scripts/validate_static_filesystem.py`
在实际目标文件系统逐条重写真实记录，验证 float64 位模式、随机读取、
SIGKILL 恢复和 JSON 原子发布。

`scripts/plan_static_engineering_capacity.py` 接收多份 `--audit`、`--lifecycle`、
`--filesystem`，以及 `--runtime-archive`、`--config`、`--resource-root`、
多个已存在的 `--volume` 和新的 `--output`。工程模型按实际几何组合数量和
SNS 记录结构分组，每个固定运行编号保存自己的完整额度。编码包络保留 5%
空间裕量，SQLite 根据实测页空间与每条记录开销计算，最终按已验证文件系统的
4096 字节分配单位取整。两个卷各自保留完整单次 WAL 和系统空间；派生处理
与来源归档在主项目文件系统预留。

工程证据要求 A–E、全部场景及三档 S3 记录覆盖，当前执行源码和公共参数下
的 S4/S5、完整 420 秒写入、各目标文件系统完整 sim 与事务恢复验证。
批量准入文件的变化单独测试；运行源码全部核对 SHA-256，
`RecordingLimits`、`RecordingBudget` 及其前置依赖核对归档 AST。
运行代码或公共参数变化需要更新对应运行证据。
生命周期证据和正式容量准入使用相同的公共 control 比较：先核对 A–E
方法组合，再排除 `solver_mode`、`recovery.mode` 和进给档位的
`nominal_feed_multiplier`。控制周期、求解权重、构型辅助、全部恢复阈值
及其他公共字段保持逐值比较；正式 A–E override 和 1,080 项矩阵保持原定义。

在正式清单冻结后提供 `--formal <正式目录>`，生成器复核配置、来源和逐卷
预算。将生成的 `capacity_plan.json` 放入该正式目录；未提供 `--formal` 的
结果标记 `formal_bound=false`，只用于规划。每个新运行通过全部准入检查后，
写入被分配的卷，正式目录的 `runs/<id>` 链接到该运行。完成后再次检查额度，
超过工程包络时保留整个运行并停止后续批次。

`run_formal` 将逐 writer 预算写入运行清单；数据库页数、WAL 实际分配、
其他保留文件及文件系统可用空间由 worker 在开始新任务前检查。
容量检查包含在 controller 的完整周期计时中。已经开始的候选计算及待完成
周期继续完整保存；触发记录、终止事件和统计保持可读，结果使用
`budget_exhausted` 停止后续批次。

两个 writer 的收尾额度来自每卷保留空间，完整 checkpoint 所需数据库空间
和其他保留文件单独扣除。每条记录的未压缩尺寸、WAL 分配增长及实际收尾
增加量随运行保存，待完成周期按实际数量预留空间。编码包络和单条记录尺寸
属于声明的 schema 验证范围；字段超过范围时保留该记录并停止新任务。
最终容量准入需要启用这些守卫的完整 sim 和最长写入证据。

共同周期测量使用 `benchmark --storage-budget-index <索引文件>`。
索引必须包含 `period-20-A` 至 `period-100-E` 的全部二十项，每项使用
`{"path": "<预算 JSON>", "sha256": "<SHA-256>"}`。预算绑定该方法、
周期、配置、源码和输出卷，并由实际 worker 执行；报告保存预算和退出
统计的身份。`freeze` 同时核查守卫统计，预算终止的运行不能用于共同周期准入。

已有真实记录的审计入口：

```bash
source .scratch/host-recovery/env.sh
python scripts/validate_static_capacity.py \
  --catalog output/static-experiments/development-scenes-05/catalog.json \
  --validation output/static-storage-validation-05/validation.json \
  --output output/static-capacity-check --resource-root "$PWD" \
  --retry-runs 0 --reserve-bytes 10000000000
```

该命令逐项核查原始记录身份、各流数量、最大 MessagePack payload、
实际压缩大小及完整流 zlib6 无损往返结果，生成固定 1,080 项的时长和容量明细。
该审计输出保留样本预算及其假设；完整准入证据还包含实际生命周期、
逐流记录上限、全部方法与场景的字段结构、完整最长写入和退出过程。
结果清楚标注样本覆盖范围、未证明项和当前挂载文件系统。
退出码 2 表示容量尚未通过；报告和原始数据仍会保留。
完整流压缩测量只提供编码潜力证据，当前逐条事务语义和记录格式保持有效。

`scripts/validate_static_lifecycle.py` 在指定 CPU 和 ROS domain 中执行完整
sim，监测进程在自己的 CPU affinity 中每 100 ms 汇总 RSS，
并在运行结束后核查启动顺序、严格递增的候选执行步、任务截止及记录数量。
检查同时要求两个 worker 正常退出、符合场景的预期终止原因，
以及完整的源码、ready 和统计身份；fault、interrupted、
budget_exhausted 均无法通过生命周期验收。
原始日志、数据库、报告、WAL 统计和命令均保留。
`--expect-timeout` 用于最长写入压力验证，要求实际触发配置 timeout
且反馈记录覆盖整个时限。压力配置与正式三档进给分别记录。
