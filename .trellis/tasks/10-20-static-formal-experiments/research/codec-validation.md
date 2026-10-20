# 逐条紧凑编码验证

本次使用独立 worktree `.scratch/worktrees/static-capacity`，
CPU affinity 为 16–27，输入为主目录
`output/static-storage-validation-05/` 中四组完整记录。
原始目录 `output/static-experiments/` 和格式 2 来源均保留。
正式采集停止，容量准入继续关闭。

## 版本和编码语义

环境使用 `zstandard==0.25.0`，依赖已加入
`requirements-static-recording.txt`，主环境和报告环境均已安装。
Python API 依据本机 `zstandard/__init__.pyi` 和
[官方 dictionary 文档](https://python-zstandard.readthedocs.io/en/latest/dictionaries.html)
核对。字典使用 `DICT_TYPE_RAWCONTENT`，每条记录生成独立压缩 frame，
zstandard 开启 checksum，解码拒绝 frame 后的额外数据。

格式 3 仍先使用 MessagePack 编码全部字段，保留 IEEE float64。
对同一 payload 分别执行 zlib level 6 与 zstandard level 3，
保存较短结果，在 `codec` 列中记录编码。
每个 stream 保留前一条原始内容，zlib 使用最后 32 KiB，
zstandard 使用最多 128 KiB；每 128 条创建独立起点。
每条 SQLite INSERT 单独提交，提交成功后更新内存字典。
WAL 和恢复语义保持一致，格式 0/2 可继续读取。

## 编码对照

实际执行：

```bash
source /home/lsn/robot/robot_safecontrol/.scratch/host-recovery/env.sh
PYTHONPATH="$PWD/src:$PYTHONPATH" taskset -c 16-27 \
  python scripts/benchmark_static_codecs.py \
  --source-root /home/lsn/robot/robot_safecontrol/output/static-storage-validation-05 \
  --output /home/lsn/robot/robot_safecontrol/output/static-capacity-20261020/codec-benchmark-01
PYTHONPATH="$PWD/src:$PYTHONPATH" taskset -c 16-27 \
  python scripts/benchmark_static_codecs.py \
  --source-root /home/lsn/robot/robot_safecontrol/output/static-storage-validation-05 \
  --output /home/lsn/robot/robot_safecontrol/output/static-capacity-20261020/codec-benchmark-02 \
  --codecs zlib6-32 zlib9-32 zstd6-128 zstd9-128
```

两次执行均返回 0，`benchmark.json` 为 `status=pass`。
每个编码完整处理四组八个数据库的 331,573 条真实记录，
逐条比较重新编码的 MessagePack 字节，包含全部字段、float64
和三个时间字段。跨 keyframe 随机读取通过；每个数据库、每种编码
分别执行 300 条已提交记录的真实 SIGKILL 恢复。
每个文件的来源、结果 SHA-256 和分流耗时均保存于 JSON。
测量数据库使用实验版本 100，生产读取入口拒绝该版本。

| 编码 | 八个数据库总字节 |
|---|---:|
| zlib level 1 / 32 KiB | 143,704,064 |
| zstandard level 3 / 128 KiB | 131,612,672 |
| zstandard level 3 / 256 KiB | 131,612,672 |
| zlib level 6 / 32 KiB | 138,846,208 |
| zlib level 9 / 32 KiB | 137,568,256 |
| zstandard level 6 / 128 KiB | 129,572,864 |
| zstandard level 9 / 128 KiB | 130,506,752 |

128/256 KiB 的结果一致；真实最大 candidate 为 126,147 字节。
S5 controller 的 zlib level 1 数据库为 7,450,624 字节，
zstandard level 3 为 696,320 字节。其余场景的变化分别保存在 JSON。
SQLite 页分配会影响最终文件大小，单条压缩 payload 更短不能独立
证明完整数据库占用同比例减少。

## 格式 3 完整升级

实际执行：

```bash
PYTHONPATH="$PWD/src:$PYTHONPATH" taskset -c 16-27 \
  python scripts/validate_static_recording_upgrade.py \
  --source-root /home/lsn/robot/robot_safecontrol/output/static-storage-validation-05 \
  --resource-root /home/lsn/robot/robot_safecontrol \
  --output /home/lsn/robot/robot_safecontrol/output/static-capacity-20261020/format3-validation-01
```

退出码为 0，`validation.json` 为 `status=pass`。
331,573 条记录逐字段与 float64 位模式一致；跨段随机读取通过。
八个数据库分别执行 1,200 条已提交记录的 SIGKILL 恢复，
全部恢复进程返回 `-9`，记录数量和 MessagePack SHA-256 一致。
controller 恢复覆盖多个 stream 的 128 条边界，S5 包含 rejection。
源文件全部身份复核通过。

版本 2 与版本 3 分别完整重算报告；全部测量指标精确相同。
三类派生文件的每个数组都按 dtype、shape、tobytes 比较。
S0/S2 的实验 verdict 为 pass，历史 S4/S5 的 fail 结论保持一致。
升级验证通过仅表示存储与重算一致。

| 样本 | 来源目录字节 | 格式 3 目录字节 | 来源重放秒 | 格式 3 重放秒 |
|---|---:|---:|---:|---:|
| S0 | 43,264,811 | 43,600,340 | 33.458 | 30.545 |
| S2 | 49,317,246 | 46,520,484 | 36.419 | 33.788 |
| S4 | 46,435,581 | 42,830,884 | 35.285 | 32.873 |
| S5 | 12,011,672 | 5,361,076 | 9.771 | 9.372 |

四组新目录共 138,312,784 字节，包含新增迁移来源说明。
验证期间工作目录峰值为 1,116,987,186 字节，分配峰值为
1,133,207,552 字节，进程 RSS 峰值为 1,008,439,296 字节。
这些峰值包含转换、恢复数据库、独立重放和派生文件。
离线 append 的 controller p99.9 为 1.381/1.957/1.707/4.341 ms；
plant p99.9 为 0.356/0.587/0.709/0.839 ms。
S2/S4 plant 的单次最大值为 25.261/23.767 ms，真实控制周期、
plant 记录延迟、RSS 和 WAL 必须在完整 sim 中重新测量。

TDD 使用公共边界 `compact_database`、`RunRecorder`、
`iter_records` 和 `read_record`。格式 2 升级测试先得到
`1 failed, 5 passed`，实现后相关检查通过：

```bash
PYTHONPATH="$PWD/src:$PYTHONPATH" taskset -c 16-27 python -m pytest \
  tests/test_static_recording.py tests/test_static_capacity.py \
  tests/test_static_campaign.py tests/test_static_execution_evaluation.py -q \
  --basetemp="$PWD/.scratch/codec-tests-complete"
git diff --check
```

测试结果为 `21 passed in 27.79s`；差异检查通过。
本次提交前普通 merge 集成分支 `332eb9a`，基础实现来自 `f3b3658`。

## 文件身份

下列文件均位于主仓库 `output/static-capacity-20261020/`；
每次验证的完整脚本保存在对应子目录。

| 文件 | SHA-256 |
|---|---|
| `codec-benchmark-01/benchmark.json` | `454e86715a8485fe881badaaca50754cc770da7a99d219194006d98be432ecee` |
| `codec-benchmark-01/benchmark_source.py` | `0b3a50765364f7be2da8923818ca49a4f2f06bf92f067db9cab7824f4fca986e` |
| `codec-benchmark-02/benchmark.json` | `9166d30152a7428f9b059645ebd10d1cf670dabec776ca6309fadffb76282f68` |
| `codec-benchmark-02/benchmark_source.py` | `3e00d4ae43b584b277f32f0fe7b92ace933554e06c566ae8b25a08e48e50a775` |
| `format3-validation-01/validation.json` | `5a9dae049d72c8afdcdbc3a93482d83e9b32b04721e4e41e313f9412abb3906c` |
| `format3-validation-01/validation_source.py` | `7d0bb44fcdde65a5dfb157ff18e339ca57ca13bce69811331e71177572b4294d` |

## 容量证据的适用范围

编码结果证明样本无损、随机读取和恢复能力；完整工程预算仍需要
最新控制记录的字段、次数和最长运行条件。
scene 工作增加的 `dual` 和 `qp_reused_from_trial` 字段须纳入后续预算。

当前源码确定的计数关系：

- plant 每次 0.002 秒 timer 生成一条 `feedback`，至多一条 `recording_delay`。
- 每个 controller command 对应 plant 的 `received_command` 与 `admission`。
- 每次 controller 计算至多九个 trial，生成一条 `candidate`、
  一条 `cycle`，以及对应 command 或 rejection；异常计算单独保存失败。
- controller 初始化最长等待 180 秒，当前启动次序允许 plant 在此期间记录。
- 正常运行 timeout 为 420 秒，退出等待分别为每个 worker 20+5 秒。

共同周期的新计算调度、controller 预热与 plant 启动顺序、
完整生命周期及 WAL 需要集成后的真实 ROS 验证。
容量报告必须绑定这些行为和代码身份，并包括失败、来源与串行派生处理。
当前样本的 WAL 在 zlib 对照中为 S0 plant 913,004,392 字节、
controller 88,654,192 字节；S2 分别为 920,156,712 和 94,722,952 字节。
这些数据可以参与最长运行验证的工作空间预算，尚未单独构成完整上界。
