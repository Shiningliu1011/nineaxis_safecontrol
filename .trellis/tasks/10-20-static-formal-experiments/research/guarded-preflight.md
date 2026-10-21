# 启用容量保护的完整仿真

2026-10-20 至 2026-10-21 本机连续执行，固定运行源码为
`40d43bb3712e3f99c11a59750a59d1ee65ecb525`。后续文档提交保持
运行源码相同。全部运行限定 sim，worker 使用 CPU 0–15、
ROS domain 200，监测进程使用 CPU 22–27；其他密集验证已经结束。

环境入口为 `.scratch/host-recovery/env.sh`。每项预算绑定配置、
源码和目标卷 UUID，容量检查实际参与完整周期计时。
代码、模型与环境归档位于
`output/static-capacity-20261020/guarded-preflight-01/source/`，
`sources.zip` SHA-256：
`c381f34a51223f99f575c26eaed9b6045601e8ea200efaadb1576e20ca8560a9`。

原始记录统一保存于
`/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/guarded-preflight-01/`。
各目录下 `run/` 包含完整数据库、配置、manifest、退出结果和反馈报告；
`lifecycle-validation.json` 保存实际启动、记录数量、收尾及原件身份。
该 NTFS 卷 UUID 为 `083CBB753CBB5BFE`。既有原件保持原内容。

## 已确认的运行结果

| 目录 | 功能及生命周期 | 终止原因 | P99.9 |
| --- | --- | --- | --- |
| `d-s5` | 通过 | `pause_timeout` | 76.788019 ms |
| `e-s5` | 通过 | `pause_timeout` | 57.935799 ms |
| `b-s5` | 通过 | `pause_timeout` | 77.717761 ms |
| `d-s4-initial1-feed1.5` | 完整路径及终点通过 | `completed` | 72.945313 ms |
| `d-s4-initial3-feed0.5` | 完整路径及终点通过，6次恢复 | `completed` | 64.287547 ms |
| `d-s2` | 完整路径及终点通过 | `completed` | 61.271307 ms |
| `a-s2` | 完整路径及终点通过，33次恢复 | `completed` | 65.482802 ms |
| `b-s2` | 完整路径及终点通过 | `completed` | 65.788009 ms |
| `e-s4` | 完整路径及终点通过，26次恢复 | `completed` | 76.915183 ms |
| `d-timeout` | 420秒生命周期检查通过，路径未完成 | `timeout` | 55.894978 ms |
| `d-wal-limit` | 预定预算耗尽，两个 worker 正常退出 | `budget_exhausted` | 61.826941 ms |

全部11项均为 workers `[0,0]`。前九项总体报告通过；后两项的完整路径
验收保持失败，实际终止与各自验证目的相符。全部 writer 的收尾增长
均在预留空间以内，实际记录尺寸及单次 WAL 增长均未超过声明界限。
每项预算、manifest、两份数据库、结果及报告的 SHA-256 保存在
`output/static-capacity-20261020/guarded-preflight-01/status.json`；
下表固定生命周期核查文件身份，其内部包含全部原件身份。

| 目录 | `lifecycle-validation.json` SHA-256 |
| --- | --- |
| `d-s5` | `4435f5526317724ba3cfb56946f806b738f2549c851bbda4a48803ff5b082c55` |
| `e-s5` | `aca09b27c4f1563913334a72b490e785302790bb3c3a2b297733ca9def862037` |
| `b-s5` | `4ed0c01ad59744671af948796a07b9d8a9e900a170a994fc384d14375b58f1d3` |
| `d-s4-initial1-feed1.5` | `c74790d50aa4dca1b8df47e2721079f98f7b7e87b8db71ff0fb85b2533228eef` |
| `d-s4-initial3-feed0.5` | `1b93d504b380d23aedf3c846c9cd3ec30f0b11d81a3858e8d77679e1bef1e4a5` |
| `d-s2` | `1d9e59fb6e567f2f3107527692e8d83a6a6f47b215ae498611dad66860479c7a` |
| `a-s2` | `b51a704e27a13bf1b3955e17b44c20f9d29be97af50c4371d720a9a809266bba` |
| `b-s2` | `18704eebd6311ec220c87179cade6c0d4857f97d0d1563f5df0217edaed6fe7e` |
| `e-s4` | `65e9ef851fb24a536ec4a31f675247f5c14e64a7a630315244d71b3d733ff55f` |
| `d-timeout` | `c76f9221b86dcff0b6520f650ebb2a06e2791d5e6d5db117113a1921b7fc3753` |
| `d-wal-limit` | `e757ae3bfa5bbfae5de5867acecf4cee2ee8db5c41aaf40fd2df765be206e435` |

完整核查汇总为同目录 `validation-summary.json`，SHA-256：
`1753d5d603d8997a637cbeb25ec757720bfb8fbe2e24bb23b104082d1842f836`。
执行脚本归档为 `execution-scripts.zip`，SHA-256：
`dc413276814b59c72554f0561428e76f9bc6f16638a158e3e932836a23d90192`。

## 独立求解核查

当前来源的新 D/S2、B/S2 的全部1771、1767个 QP trial 均满足原始限制，
最大超差分别为 `1.692e-17`、`1.346e-17`；各5个代表输入的独立
SciPy SLSQP 核查通过。结果分别位于 NTFS 原件目录的
`d-s2/commands/run-solver.json`、`b-s2/commands/run-solver.json`，SHA-256为
`68001667b5115caa5d1368fd255d9abf7440a6ccf6d6dc679957ad601e11c85f`、
`799ed02668e54d1462eb83a6c6974f3487c6373e04ce780f7837780e08a3116a`。

当前 E/S4、E/S5 分别核查2179、2826个实际 SNS trial，
负上界数量186、2745，饱和数量940、2787；最大广义限制超差
分别为 `4.447e-15`、`6.107e-16`，最小 task_scale 均为1。
代表输入的独立 HiGHS 最大缩放与 null-space/DLS 核查通过。
结果位于主目录同级 `solver-audits/e-s4.json`、`e-s5.json`，SHA-256为
`455ec8b3e82e16334773370ad0adda8a14f5a05fd46d70680f9ae79cb324f4f5`、
`a7d4c1e09ce4e5857364abac94d867f37f6638f5b35ae8a6c5077bde73e84271`。

## ext4 完整运行

`output/static-capacity-20261020/guarded-ext4-s5-01/` 保存同一源码的
ext4 D/S5。功能、完整生命周期及记录收尾通过，workers `[0,0]`；
P99.9为85.834063 ms，最大95.345604 ms，没有超期或准入拒绝。
该项时间验收失败，总体报告保持失败，共同周期仍需独立测量。
生命周期与反馈报告 SHA-256 分别为
`e3729ab37a075b5554eb592f67e1e93f07ad91cfedb30100f422c4a28933d572`、
`74f35621876f726d480bcd4a34328ee44f9ecf270af795144ea8070dfb1435c1`。

## 接续入口

已结束的用户 service 为 `static-guarded-preflight-01.service`，
执行脚本为 `.scratch/static-implementation/run_guarded_validation.py`，
启动脚本为同目录 `run_guarded_validation.sh`。检查实际 service 和
进程状态后，再使用 JSON 中的进度；进度文件本身不证明进程仍存活。

固定计划为
`output/static-capacity-20261020/guarded-preflight-01/plan.json`，
SHA-256：
`aa0ee3a7a99568f5c954e2260ed276f980b08e2df10748ebb81d17e0709f5671`。
计划记录执行与预算生成脚本身份，11项均已结束，service 为
`inactive/MainPID=0/ExecMainStatus=0`。ext4 补充 service
`static-guarded-ext4-s5-01.service` 同样正常结束。所有原始目录保留。

这些运行属于正式采集前的验证。完整矩阵仍为 `0/1080`；
共同周期、最终现场容量、完整回归与最终审查保持待办。
