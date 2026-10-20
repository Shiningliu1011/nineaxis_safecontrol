# 静态实验紧凑存储

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
本机测量为准。S4/S5、共同周期和其他正式实验前置条件仍需分别验收。

## 正式矩阵容量准入

`static_experiment_capacity.py` 分别计算全部计划运行、保留的失败重试、
单次运行的 WAL 工作空间、串行派生处理、来源归档和系统保留空间。
各项使用字节数；固定矩阵已经包含每个组合的三次重复。
额外重试预算通过 `--retry-runs` 显式提供，增加重试需要重新核算。
恢复批量运行时，已有目录继续保留，只为尚未运行的组合预留新记录空间。
每个新运行启动前检查当前文件系统可用空间，已有运行超过记录容量上限时停止批量执行。

正式目录需要 `capacity_plan.json`，其身份绑定 `formal_manifest.json`、
代码来源和原始验证证据。只有经过验证的完整容量上界才能用于准入；
样本外推即使小于可用空间，也会保持证据不足。
容量文件保存 `CapacityBudget` 的各项数值、
`is_validated_capacity_upper_bound`、`evidence_sha256`、
`formal_manifest_sha256` 和 `source_sha256`。
证据文件或清单身份变化时，需要重新验证容量。

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
时长留量包含 controller 启动期间 plant 的记录、运行 timeout、
settle 和进程退出等待。候选被拒绝后的重新计算频率也需要单独约束。
结果清楚标注样本覆盖范围、未证明项和当前挂载文件系统。
退出码 2 表示容量尚未通过；报告和原始数据仍会保留。
完整流压缩测量只提供编码潜力证据，当前逐条事务语义和记录格式保持有效。
