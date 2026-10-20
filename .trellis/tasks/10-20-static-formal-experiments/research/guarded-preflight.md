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

上述四项均为 workers `[0,0]`，总体报告通过。
每项预算、manifest、两份数据库、结果及报告的 SHA-256 保存在
`output/static-capacity-20261020/guarded-preflight-01/status.json`；
下表固定生命周期核查文件身份，其内部包含全部原件身份。

| 目录 | `lifecycle-validation.json` SHA-256 |
| --- | --- |
| `d-s5` | `4435f5526317724ba3cfb56946f806b738f2549c851bbda4a48803ff5b082c55` |
| `e-s5` | `aca09b27c4f1563913334a72b490e785302790bb3c3a2b297733ca9def862037` |
| `b-s5` | `4ed0c01ad59744671af948796a07b9d8a9e900a170a994fc384d14375b58f1d3` |
| `d-s4-initial1-feed1.5` | `c74790d50aa4dca1b8df47e2721079f98f7b7e87b8db71ff0fb85b2533228eef` |

## 接续入口

用户 service 为 `static-guarded-preflight-01.service`，
执行脚本为 `.scratch/static-implementation/run_guarded_validation.py`，
启动脚本为同目录 `run_guarded_validation.sh`。检查实际 service 和
进程状态后，再使用 JSON 中的进度；进度文件本身不证明进程仍存活。

固定计划为
`output/static-capacity-20261020/guarded-preflight-01/plan.json`，
SHA-256：
`aa0ee3a7a99568f5c954e2260ed276f980b08e2df10748ebb81d17e0709f5671`。
计划记录执行与预算生成脚本身份，按顺序保存11项开发验证。
当前后续项目为 D/S4 initial3/feed0.5、新 S2 的 D/A/B、
E/S4、420秒完整超时和 WAL 预算耗尽。worker 异常退出时队列停止，
所有已经生成的目录保留。

这些运行属于正式采集前的验证。完整矩阵仍为 `0/1080`；
共同周期、最终现场容量、完整回归与最终审查保持待办。
