# 2026-10-20 紧凑存储验证

工作基准 `6afe1d34`，主机 i7-14700、Ubuntu 22.04.5、Python 3.10.12，
详细环境见 `docs/host_recovery_20261020.md`。所有验证限定 sim 或离线副本。

完整命令：

```bash
source .scratch/host-recovery/env.sh
python scripts/validate_static_storage.py \
  --output output/static-storage-validation-05 --resource-root "$PWD"
```

退出状态 0，结果 `status=pass`。原始目录 `output/static-experiments/` 保留，
转换前后逐文件核对 SHA-256。四组数据库共 331,573 条记录逐项一致，
包括 stream、sequence、三个时间字段、反馈、候选、控制决策、事件和失败原因；
float64 使用位模式比较。

## 容量和重放

容量采用完整运行目录，包含数据库、来源说明、原始报告、重算汇总与其他文件。
MB、GB 使用十进制；GiB 使用 2 的幂。

| 来源运行 | 原始字节 | 紧凑字节 | 节省 | 原始格式重算 / s | 紧凑格式重算 / s | 结论 |
|---|---:|---:|---:|---:|---:|---|
| `s0-joint-full-geometry-01` | 187030857 | 43264811 | 76.87% | 29.981 | 26.917 | pass |
| `s2-joint-development-01` | 218380214 | 49317246 | 77.42% | 33.308 | 29.972 | pass |
| `s4-joint-development-10` | 206865974 | 46435581 | 77.55% | 32.416 | 28.555 | fail |
| `s5-joint-development-01` | 84769396 | 12011672 | 85.83% | 8.943 | 8.009 | fail |

表中结论为实验自身结论；四组存储一致性验证均通过。
原格式和紧凑格式分别在本机独立重算，全部评价字段及派生数组逐项一致。
归档机器与本机的历史派生距离最大差值为 6.52434450909567e-9 m；
该差值单独记录，未修改几何验收精度。历史反馈、时间、参考进度保持一致。
三个派生数组按需重算耗时分别为 31.156、35.163、33.303、8.432 s，
生成体积分别为 38,619,134、49,937,480、44,883,794、1,490,482 字节。
本次一致性复核与控制内核回归同时执行，耗时包含该负载的影响。

单个已验证完整运行样本为 43.26–49.32 MB。以最大样本直接乘以 1,080，
预计 53.26263 GB；按三档进给 0.5、1.0、1.5 的时长反比作相同数量外推，
预计 65.09876 GB。后一个估算采用平均系数 11/9，尚未包含场景、方法、
暂停时长和失败模式的全部差异，不能作为正式容量上界。

验证进程每 25 ms 采样目录容量，包含新数据库、WAL、旧格式写入对照、
中断副本和派生数组。观测逻辑峰值 1,112,808,684 字节，
文件系统分配峰值 1,130,258,432 字节，进程 RSS 峰值 987,701,248 字节。
采样峰值不保证捕获短于 25 ms 的瞬时变化。

清理重复迁移资料后，2026-10-20 20:06（Asia/Shanghai）测得
可用空间为 80,986,005,504 字节；容量记录在 `output/host-recovery-capacity.json`，
该文件在结束工作前更新最终测量时间和空间。
直接外推后剩余约 27.72 GB，按进给外推后剩余约 15.89 GB；
再预留已测验证分配峰值，剩余约 14.76 GB。
该样本外推已经能够容纳；系统、失败重试、派生处理及全部场景的容量上界
仍需纳入正式计划。正式采集保持停止。

## 读写耗时

独立测量使用 `taskset -c 0-15` 和上述环境，无其他项目测试同时运行。
结果保存在 `output/static-storage-performance-01/measurement.json`，
SHA-256 为 `eb8f8f353b62960a04a82f2d2b26a399348e7342e8cf5eafaabf9ce69a5459d3`。
测量入口 `.scratch/host-recovery/measure_storage.py` 的 SHA-256、
CPU affinity 和完整环境身份保存在该结果文件。

顺序读取包含解码和逐条内容 SHA-256；单位秒。

| 场景 | plant 旧 / 新 | controller 旧 / 新 |
|---|---:|---:|
| S0 | 0.939 / 0.564 | 0.778 / 0.458 |
| S2 | 0.936 / 0.569 | 0.998 / 0.598 |
| S4 | 0.941 / 0.575 | 0.893 / 0.526 |
| S5 | 0.227 / 0.126 | 0.766 / 0.369 |

以下为单条写入平均 / P99.9，单位毫秒。旧格式对照包含 JSON 序列化与
SQLite 事务，新格式包含完整 append 校验、压缩、事务及内存状态更新。
转换读取成本不计入单条 append，但计入完整转换耗时。

| 场景 | plant 旧 | plant 新 | controller 旧 | controller 新 |
|---|---:|---:|---:|---:|
| S0 | 0.0202 / 0.0586 | 0.0298 / 0.1230 | 0.1747 / 0.6914 | 0.1763 / 0.7389 |
| S2 | 0.0202 / 0.0514 | 0.0305 / 0.1373 | 0.2181 / 1.0553 | 0.2498 / 1.0761 |
| S4 | 0.0205 / 0.0623 | 0.0316 / 0.5077 | 0.1968 / 0.9469 | 0.1979 / 0.9449 |
| S5 | 0.0201 / 0.0390 | 0.0201 / 0.0528 | 0.7913 / 3.6072 | 0.6196 / 2.1761 |

新格式的 plant 写入最大值在 S2 为 34.051 ms、S4 为 23.601 ms；
旧格式对应最大值为 0.733 ms、0.277 ms。全部逐组测量结果保留。
这些离线连续写入数据不能代替实际 ROS 闭环周期测量。

## 紧凑格式的完整 S0 sim

运行目录 `output/static-storage-control-01`，使用
`config/static_experiment_s0.yaml`、`joint`、`improved`、ROS domain 197，
CPU affinity 0–15。完整命令和环境身份保存在
`output/static-storage-control-01-measurement.json`。

命令退出 0，控制器完成 1,719 步，反馈评价、综合证据均为 pass。
有效反馈 87,116 条，路径距离最大 0.172592 mm、工具轴误差最大
0.062754°；外部间距、自碰撞间距、关节限制、进度结算及 QP 均通过。
完整周期 P99.9 为 68.333 ms、最大 70.513 ms，周期 100 ms，
deadline miss、admission rejection 和 rejected cycle 均为 0。

启动、运行和评价合计 221.961 s，最终目录 43,357,255 字节。
每 25 ms 采样观测到包含数据库 WAL 的逻辑容量峰值 998,497,488 字节，
文件系统分配峰值 1,012,109,312 字节，子进程 RSS 峰值 731,086,848 字节。
WAL 临时容量需要计入实验工作空间。该运行仅验证 S0 的紧凑记录闭环，
共同周期仍需完成规定的 20 组方法与候选周期测量。

| S0 运行文件 | SHA-256 |
|---|---|
| `manifest.json` | `77c8cedcbb000efa22c7533218eaa19885347690b4cd5dddc8ce1e50d6c04fa3` |
| `plant.sqlite3` | `ec9bf39b5cae023761752ecc7172c65e690cde61dd0f367141477a80a7a516f6` |
| `controller.sqlite3` | `4ec8e329dcc7a819343d3dd981f902f79108ebd35b39405fa822d2f94eafae3c` |
| `feedback_report.json` | `276023387764baf602f12f89211b6bdd0129d86874b35193a8f573aefa888669` |
| `../static-storage-control-01-measurement.json` | `9b0e38fefa9c0c484ccf1ab428fd1440eaded9d0575b60457aa438d526cc910a` |

## 恢复、CLI 和图表

每组 plant/controller 均在写入来源记录前 256 条并确认提交后，
由父进程发送 SIGKILL，退出码为 -9。8 次检查均恢复全部已确认记录，
跨过 128 条字典重置边界；TRUNCATE checkpoint 后内容保持一致。
单独单元测试还检查 300 条记录恢复、实时读取、跨段随机访问、
损坏数据及未知版本拒绝。

`compact` CLI 实际处理完整 S5：

```bash
python scripts/static_obstacle_experiment.py compact \
  --source output/static-experiments/s5-joint-development-01 \
  --output output/static-storage-cli-s5-01 --resource-root "$PWD"
```

命令退出 0，重算报告仍为 fail。报告生成命令：

```bash
python scripts/static_obstacle_experiment.py report \
  --output output/static-storage-validation-05/s0-joint-full-geometry-01 \
  --resource-root "$PWD" \
  --report-python "$PWD/.scratch/host-recovery/report-venv/bin/python" \
  --figures-output "$PWD/picture/static-storage-validation"
```

命令退出 0，生成 6 张图表；数据摘要、字体身份和字符覆盖、
图片文件完整性与尺寸均通过实际检查，未执行视觉检查。
`figures.json` SHA-256：
`bb0ef9adf25e80ed1571ee4571b74bc751326451580581720fb843695b552679`。
`evaluation_provenance` 与本次生成数组所用的 evaluator 及全部评价源码身份一致，
原始汇总报告 SHA-256 保持不变。完整检查结果为
`output/static-storage-final-figures-check.json`，SHA-256 为
`f96ef644779043398ffb726e09a50f0cdca7f40113423fec641a613974aff00e`。

## 原始数据库身份

| 场景 | plant SHA-256 | controller SHA-256 |
|---|---|---|
| S0 | `dadbf75e12daaddd7ba6ce68210066d89558fe4d66be5d7e5e540292366ab31a` | `88f02789a8f1255141c61bc16f8411dadcf3a8f6f613abf37c882d6ee03ca180` |
| S2 | `74e315ffbb614132c5fa8002a6a89702a4efd60ecdcd50fb96b486adde9ab389` | `fae6414ac1435f4cd5341f738c3f49d4f2ee2ed7fa4a626fa3a9f45c179c981b` |
| S4 | `feb3df8f99cd18d2cd0ffb8491b04e88c1b749e5b3af14148e66fa3945a0f9c2` | `046d5164019fa8d8783de49dea3acbb9b5755ad5b7c467df1b59107501ffe9d8` |
| S5 | `df962dd99d5d72efb8ba725002c0becfa1786ec536e0c11e3409b34865a59077` | `7857ec84bbca1591902877a8e86e190b2c75898ae314acb69b2329b6c61badbc` |

完整计数、来源、新数据库身份和测量在
`output/static-storage-validation-05/validation.json`，SHA-256 为
`740eafa49b709b5b70d2d744b854f1cbe1af45d1fc4612181011af78a9005209`。
