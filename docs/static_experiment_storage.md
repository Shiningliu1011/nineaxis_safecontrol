# 静态实验紧凑存储

新运行使用 SQLite 格式 2；旧 JSON 数据库保持可读。安装记录依赖：

```bash
python -m pip install -r requirements-static-recording.txt
```

## 保存与恢复

`static_experiment_recording.py` 统一管理写入、流读取和按 sequence 读取。
`samples` 保留 sequence、stream、monotonic_s、simulation_s 和 ros_time_ns。
payload 使用 MessagePack 双精度数值和 zlib 无损压缩；没有数值取整或采样删除。
同一 stream 的前一条完整内容提供最多 32 KiB 压缩字典，每 128 条独立开始。
keyframe 列指定该段起点，随机读取只需恢复对应 stream 的当前段。
重复状态参与字典压缩，原有字段和事件均可完整还原。

每次 append 仍以独立 SQLite 事务提交；提交成功后更新内存字典。
WAL 自动 checkpoint 继续关闭，采样结束后执行 TRUNCATE checkpoint。
中断恢复必须同时保留数据库和 WAL，使用 `finalize_recording()` 完成完整性检查。
已经提交的记录可在写入进程运行期间读取。`synchronous=NORMAL` 保留原有
进程异常恢复语义；本次 SIGKILL 验证不代表主机断电时最后事务仍能保存。

格式通过 `PRAGMA user_version` 判断：0 为旧 JSON，2 为当前格式。
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

`storage_migration.json` 保存源目录、全部源文件 SHA-256、原始 manifest、
各流数量、逐条内容身份以及转换数据库 SHA-256。原有报告位于 `source_reports/`。
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
```

真实验证默认处理接手包中的完整 S0、S2、S4、S5 记录，输出目录必须不存在。
验证包含逐字段与 float64 位模式比较、跨段随机读取、全部流数量和时间字段、
真实 SIGKILL 已提交记录恢复、旧格式和新格式的独立指标与派生数组重算，
以及读写耗时、目录容量、WAL 处理峰值和内存峰值。
历史机器生成的派生数组与当前重算的最大差值另外保存。

容量和耗时结论以[接手记录](static_obstacle_experiments_handoff.md)所指向的
本机测量为准。S4/S5、共同周期和其他正式实验前置条件仍需分别验收。
