# 正式矩阵容量验证

本次在 `/home/lsn/robot/robot_safecontrol` 的恢复环境完成离线审计。
运行日期为 2026-10-20，CPU affinity 为 26–27，正式采集保持停止。
输出目录为 `output/static-capacity-20261020/`，原始数据保留于
`output/static-experiments/`；输入紧凑副本为
`output/static-storage-validation-05/`。

## 实际命令和数据身份

在独立 worktree `.scratch/worktrees/static-capacity` 执行：

```bash
source /home/lsn/robot/robot_safecontrol/.scratch/host-recovery/env.sh
PYTHONPATH="$PWD/src:$PYTHONPATH" taskset -c 26-27 \
  python scripts/validate_static_capacity.py \
  --catalog /home/lsn/robot/robot_safecontrol/output/static-experiments/development-scenes-05/catalog.json \
  --validation /home/lsn/robot/robot_safecontrol/output/static-storage-validation-05/validation.json \
  --output /home/lsn/robot/robot_safecontrol/output/static-capacity-20261020/retry-budget-0 \
  --resource-root /home/lsn/robot/robot_safecontrol --retry-runs 0
```

退出码为 2，容量状态为 `insufficient_evidence`，
`is_validated_capacity_upper_bound=false`。真实数据核查完成：
四组全部原始文件与各自 `storage_migration.json` 的 SHA-256 一致，
紧凑数据库身份一致，331,573 条记录逐流统计完成。
完整流 zlib6 编码与解码的 MessagePack 字节 SHA-256 一致，
所有 float64、字段和三个时间字段参与核查。

| 文件 | SHA-256 |
|---|---|
| `retry-budget-0/capacity_plan.json` | `54802e61b8f7781863044f499e4f19170b522c6614e459adc0d3d623736423a0` |
| `capacity_plan.json` | `63d67ee4d94cc7f9f65f96b6da9f6a48f5b15583e01feb6cd3a22400e9906530` |
| `s0-joint-full-geometry-01.json` | `52be43b8d8c7c383ae22fdbca6dcb3edd5c84d4acfec3714aa8933239b858aca` |
| `s2-joint-development-01.json` | `4e79d9ebd0140a4037b4260eae0ce4bce6bd96bb007c5a569a105ebce606893a` |
| `s4-joint-development-10.json` | `f732ad0edb9f92167fce5ef2ffd0f855f645b1468dae486caeadfadb501168f6` |
| `s5-joint-development-01.json` | `6ac90f3b701ec9f199e82b00a95ea076b5e2839e049625033c913e94ce9406e6` |

报告保存全部原始文件身份、数据库身份、场景配置身份、执行程序身份、
完整命令、Python/SQLite 版本、CPU affinity、挂载文件系统与空间测量。
额外重试压力调查使用的审计脚本完整保存为同目录 `validator_source.py`，SHA-256 为
`23a7536c6af33ae98a175f27a9f01358bced4323c3ba8ef774ca32a23fc0f4ea`。
输入 `validation.json` SHA-256 为
`740eafa49b709b5b70d2d744b854f1cbe1af45d1fc4612181011af78a9005209`。

## 固定矩阵和记录时长

全部 1,080 项分别写入报告：S0、S1、S2、S3 的三档行程、S4、S5，
每项包含三个初态、0.5/1.0/1.5 倍进给、A/B/C/D/E、三次重复。
当前八个场景的运行 timeout 均为 420 秒。

`static_experiment.py` 在 plant ready 后启动 controller；
controller 启动最多等待 180 秒，期间 plant 已经记录 500 Hz 反馈。
正式 timeout 计时从两个 worker ready 后开始。
退出过程中两个 worker 各有 20 秒正常等待和 5 秒 SIGINT 等待。
本次样本预算使用 `420 + 180 + 2 + 50 + 1 = 653` 秒的记录时长留量，
其中 2 秒来自 `settle_s`，1 秒为轮询留量。
调度暂停没有独立执行界限，因此该留量仍须验证。

`StaticExperimentController._finish_calculation()` 在候选拒绝时保留
candidate、rejection 和 cycle，随后 timer 可以重新发起计算。
这条分支的候选数量不能直接使用共同周期倒数计算。
`StaticControl.propose()` 最多尝试八个正向进给候选，再尝试一个零进给候选。
原始 S5 实际出现 9 次 trial，最大 candidate MessagePack payload
为 126,147 字节；S2 的实际记录包含最多 76 个几何 pair。

## 样本预算

全部数字使用十进制字节；下列数字均保持样本估算身份。

| 项目 | 字节 |
|---|---:|
| 最大完整样本直接乘以 1,080 | 53,262,625,680 |
| 三档进给按时长反比外推 | 65,098,764,720 |
| 全部运行按 653 秒与最大已测存储速率外推 | 189,059,362,200 |
| 每个组合额外保留一次完整重试 | 189,059,362,200 |
| 独立临时工作空间留量 | 15,644,129,809 |
| 串行派生处理留量 | 177,256,876 |
| 来源归档留量 | 100,000,000 |
| 系统保留空间 | 10,000,000,000 |
| 合计 | 404,040,111,085 |
| 本次测得主文件系统可用空间 | 80,241,741,824 |

临时工作空间留量采用历史完整存储验证分配峰值按最长时段缩放；
该历史峰值包含 WAL、旧格式对照和转换验证，并非单次 WAL 上界。
来源归档与系统空间为显式预算额度；追加重试需要重新核算。
当前预算无法通过，尚不能据此推断完整正式实验的实际最终大小。
表中的 1,080 次额外重试用于容量压力调查。正式固定矩阵已经包含
每个组合三次重复；正式额外重试数量应显式提供，并在预算用完时停止。
无额外重试时，同样留量下的样本预算为 214,980,748,885 字节。
该预算已重新执行全部数据审计，实际可用空间为 80,204,554,240 字节，
估算差额为 134,776,194,645 字节，完整结果位于 `retry-budget-0/`。
容量估算不足不改变算法功能评价。

## 紧凑编码测量

按每个数据库的 stream 顺序将完整 MessagePack 记录送入 zlib6，
持续解码并核对 SHA-256。结果包括全部字段与时间列，
不包含 SQLite 索引和事务开销。

| 场景 | 当前完整目录字节 | 完整流 zlib6 字节 |
|---|---:|---:|
| S0 | 43,264,811 | 30,706,980 |
| S2 | 49,317,246 | 36,796,777 |
| S4 | 46,435,581 | 32,897,230 |
| S5 | 12,011,672 | 5,972,754 |

该测量证明完整流编码能够保持内容身份。随机读取、逐条事务提交、
SIGKILL 恢复和正式周期性能尚未由该编码实现证明。
本次没有更换记录格式或改变原始数据。

## 容量准入与检查

公共边界为 `CapacityBudget`、`assess_capacity()`、
`require_formal_capacity()` 和 `run_formal()`。
预算分别计入计划记录、失败重试、临时工作空间、派生处理、
来源归档与系统空间。正式执行要求容量证据绑定清单、代码和来源 SHA-256；
每次启动新运行前重新读取实际可用空间。
恢复执行保留已有记录，只为剩余组合计入新容量。
已有目录超过声明容量上限时拒绝继续。

定向检查命令：

```bash
PYTHONPATH="$PWD/src:$PYTHONPATH" python -m pytest \
  tests/test_static_capacity.py tests/test_static_recording.py \
  tests/test_static_campaign.py -q \
  --basetemp="$PWD/.scratch/capacity-tests-complete"
git diff --check
```

定向测试实际通过 17 项；预算缺口、样本证据拒绝、身份校验、
恢复运行和当前真实可用空间检查均在测试范围内。
样本预算的公共 API 保持 `insufficient_evidence`，空间差额单独报告。

需要继续完成的证据包括全部方法/场景的编码上界、
候选重算调度、最长运行/WAL、派生处理和失败重试的实际验证。
正式容量准入继续关闭。
