# 三卷工程容量验收

2026-10-21，三卷工程方案 `status=pass`，十项证据检查全部通过，
`is_validated_engineering_budget=true`。方案保持 `formal_bound=false`、
`is_validated_capacity_upper_bound=false`；正式清单绑定、共同周期与现场空间
检查按各自证据完成。固定矩阵为 1,080 项、零额外重试，编码裕量保持 5%。

## 完整预算与分卷

| 预算项目 | 字节 |
| --- | ---: |
| 全部运行保留数据 | 198,629,535,744 |
| 三卷 WAL 工作空间 | 11,667,505,152 |
| 派生处理空间 | 536,870,912 |
| 来源归档空间 | 134,217,728 |
| 三卷保留空间 | 3,221,225,472 |
| 总需求 | 214,189,355,008 |
| 生成时三卷可用 | 284,842,614,784 |
| 总剩余空间 | 70,653,259,776 |

| 输出卷 | 分配运行数 | 所需字节 | 当时可用字节 |
| --- | ---: | ---: | ---: |
| ext4 | 381 | 74,140,913,664 | 74,175,369,216 |
| 第二卷 NTFS | 686 | 132,703,035,392 | 132,869,025,792 |
| 第三卷 NTFS | 13 | 7,345,405,952 | 77,798,219,776 |

每卷完整计入 3,889,168,384 字节 WAL 和 1,073,741,824 字节保留空间。
派生处理与来源归档空间计入主 ext4。逐卷预算、逐次记录额度与总预算一致。
分配依据生成时各卷空间，最终正式准入需要重新检查各卷当前空间及运行清单。
运行期间的字节、空间和记录数量守卫继续生效；预算耗尽时保存原始记录和
终止事件，并停止后续批次。

输出根目录：

- ext4：`/home/lsn/robot/robot_safecontrol/output/static-capacity-20261020/formal-runs-ext4`，UUID `03b3e264-cb91-440b-b594-0bdc7c9d190c`。
- 第二卷：`/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/formal-runs`，UUID `083CBB753CBB5BFE`。
- 第三卷：`/media/lsn/Windows/robot_safecontrol_static_experiments_20261021/formal-runs`，UUID `6C001EAA001E7B72`。

## 第三卷实际文件系统验证

第三卷 `/dev/nvme0n1p3` 以 `ntfs3`、`rw` 正常挂载；全部新增数据位于
`/media/lsn/Windows/robot_safecontrol_static_experiments_20261021/`。
已有 Windows 内容和全部原始实验记录保持原状。
离线入口在 11:04:25–11:04:45（UTC+08:00）、CPU 16–21 执行，退出 0。

来源是第二卷 `guarded-preflight-01/e-s5/run/`，包含 plant 18,492 条、
controller 1,099 条，共 19,591 条记录。完整重写、逐值比较和 float64
位模式摘要全部通过，重写数据库的完整 SHA-256 与原始数据库相同。

| 原始数据库 | SHA-256 |
| --- | --- |
| `plant.sqlite3` | `2d7a4c1832745781c1071682c47426cf5cb9f2e361b42c46efa9f97a6baae082` |
| `controller.sqlite3` | `4353c912034ab4ce8b9c78b25a525578dbbe21877939f79d73f19fdfb898cbe3` |

`filesystem-01/` 完成 plant 14 次、controller 28 次随机读取。
SIGKILL 恢复精确保留 plant 1,200 条、controller 全部 1,099 条已提交记录，
两个进程退出码均为 -9；同目录 pending JSON rename 后内容检查通过。

`storage-guard-01/` 使用真实记录触发 8 MiB WAL 额度，plant 保存 935 条、
controller 保存 872 条原始记录及各自终止事件后执行 SIGKILL。
恢复结果精确一致，原因均为 `budget_exhausted: storage/wal_bytes`，
两个 writer 的收尾增长各为 16,384 字节，均在保留额度内。
全部重写及恢复数据库保留在对应目录。

## 第三卷完整 sim 与计时边界

完整 D/S5 保存于第三卷 `guarded-s5-01/run/`，使用启用容量守卫的真实
controller、plant，CPU 0–15、ROS domain 200；监测间隔 0.1 秒。
终止为 `pause_timeout`，两个 worker 退出码均为 0。
功能及记录生命周期通过，记录数量与字节额度通过。

346 个周期，P99.9 为 91.120665 ms、最大值 95.185244 ms，
deadline miss 和 admission rejection 均为零。100 ms 周期下的 80% 计时
准入要求未满足，因此 timing 和总体 verdict 保留 `fail`，run 进程退出 1。
该运行提供第三卷完整写入生命周期证据，不提供共同周期通过结论。

WAL 各 writer 观测最大值之和为 175,779,864 字节，RSS 观测峰值
814,178,304 字节，完整运行占用 5,222,400 字节。

| 第三卷 D/S5 原件 | SHA-256 |
| --- | --- |
| `run/manifest.json` | `c957e402404c08bdb0ce0c649e2b91da753b64e6360f596093d49ba2708e0cd4` |
| `run/controller.sqlite3` | `62872ebb40ef995ec7e33b150f963bff7e6a90f420f1865a22667fc3a7b9a0d6` |
| `run/plant.sqlite3` | `87f6b2d4b037407970a5497c6b5326a281ea3814bce9bb3f78ba029b31064ec1` |

## 工程来源与可复查入口

运行内容绑定 `40d43bb3712e3f99c11a59750a59d1ee65ecb525`，123 份运行来源
沿用主仓库 `output/static-capacity-20261020/guarded-preflight-01/source/sources.zip`，
SHA-256 为 `c381f34a51223f99f575c26eaed9b6045601e8ea200efaadb1576e20ca8560a9`。
文件系统验证使用的 `static_experiment_recording.py` SHA-256 为
`5106598acb41d34ad299509d530b5a7118f4bfce552397e8c5f08f1c245dc8a3`。
完整回归结果见 [当前源码完整回归](full-regression-20261021.md)。

工程生成在 11:06:22–11:06:30（UTC+08:00）、CPU 16–21 执行，退出 0。
它复用十九组编码审计、第二卷五份完整 D 生命周期、ext4 和第三卷生命周期、
三卷文件系统报告，并保存全部十一项 guarded 报告身份。
十项检查覆盖方法、全部场景及 S3 行程、当前 S4/S5、完整超时记录、
各输出卷完整 sim、精确记录及随机读取和 SIGKILL、来源、分配单位与名称长度。

主仓库 `output/static-capacity-20261020/three-volume-capacity-01/` 保存确切
命令、脚本副本、CPU、时间、全部输入身份、逐次额度和分卷预算。
`third-volume-validation-01/` 保存离线执行日志、预算及输入和输出校验。
执行入口见 [第三卷验证入口](third-volume-validation.md)。

| 证据文件 | SHA-256 |
| --- | --- |
| 第三卷 `storage-guard-01/validation.json` | `948505c72f893f1e4b1cc05a7d451de63d067a6a047facacd3d0ce54ea9e564f` |
| 第三卷 `filesystem-01/validation.json` | `4504c241a89e3176bd18e6d3ff21e98c54a6c002329ca09a9d0022cf888b3182` |
| 第三卷 `filesystem-01/atomic.json` | `fbd716a36fe1f28b9c533a21b791310fe869e442282e6d9501aeba9164ca12a2` |
| 第三卷 `guarded-s5-01/lifecycle-validation.json` | `e2e6b61981ac19c1476fe780f829c7caf5ef5db04d86d51c78cdc75a7f31bd35` |
| `three-volume-capacity-01/execution.json` | `e195965e23a4f68810715e49c9fe827e85e819f81779e4d74586ca56c523e824` |
| `engineering-plan/capacity_plan.json` | `a958b7eff4882a8adf93ae8a19d2e479523b6a556247ad47d9e2959b5a5e696b` |
| `engineering-plan/engineering_evidence.json` | `2119aa8150c5335858649f0066a08f6bd1a89b56426530b6c68b48cfae020422` |

本次文档核查只读取这些小型报告及其校验值，没有重跑审计、写入验证或 sim。
