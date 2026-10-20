# S4 开发场景 A/C 配对

2026-10-20 完成相同 S4、initial 1、名义进给、100 ms 周期、
构型辅助系数 0.02 的 A/C 完整 sim。生产源码固定为 `7b643c4`，
使用 `.scratch/worktrees/static-scenes`，CPU 8–15、ROS domain 200。
独立见证采用 `s4-paired-independent-witness-01/witness`。
另一组开发验证同期运行，本次耗时仅作为开发观测。

## 真实结果

| 指标 | A | C |
| --- | ---: | ---: |
| 最终状态 | `pause_timeout` | `pause_timeout` |
| 完整跟随验收 | fail | fail |
| 实际最终进度 | 1.201319834 m | 1.201349559 m |
| 未到达的路径长度 | 0.615439910 m | 0.615410186 m |
| 实际恢复次数 | 9 | 6 |
| 有效正向候选至恢复的延迟 | 全部 0 s | 0.2–0.3 s |
| 路径 RMS / P95 / 最大误差 | 0.035104 / 0.085340 / 0.149573 mm | 0.035175 / 0.086154 / 0.149581 mm |
| 完整周期数量 | 2,280 | 2,281 |
| 周期 P99.9 / 最大耗时 | 71.662537 / 86.265383 ms | 76.600506 / 82.027355 ms |
| 超期 / 准入拒绝次数 | 0 / 0 | 0 / 0 |
| controller / plant 退出码 | 0 / 0 | 0 / 0 |

暂停期间参考进度保持，实际关节发生调整；两组均出现暂停后的实际恢复，
随后在持续受限时达到暂停超时条件。C 的六次确认延迟为
0.3、0.2、0.3、0.3、0.2、0.2 秒。两组均未完成全路径，原始失败完整保留。
当前单次配对用于开发行为验证，正式重复实验及其统计结论仍待执行。

## 配对身份与文件

项目目录：`/home/lsn/robot/robot_safecontrol`。
原始数据：`output/static-scene-validation-20261020/s4-ac-pair-01/{A,C}/run/`。
`comparison.json` 实际检查两份配置仅在 `control.recovery.mode` 不同，
全部生产源码 hash 和路径 hash 相同；同时核对报告引用的数据库及 manifest。
原始记录、报告、终止状态和逐事件恢复数据均由该文件列出。

生成入口：`.scratch/static-implementation/summarize_s4_ac.py`。
`comparison.json` SHA-256：
`00a0c4e8b14fbddbe7a4c063c5aa43df8ec313d7af4cfdcd602e09b891dcfaff`。

| 文件 | SHA-256 |
| --- | --- |
| `A/run/controller.sqlite3` | `2db3d6d93d1a9c178868545338bb7696b6e31d35533811d7cc4903c16e7b0bfb` |
| `A/run/plant.sqlite3` | `a603c820c499fe0ef4dc99f890025da64fc8eea81bb758bc99782c19aad8b402` |
| `A/run/feedback_report.json` | `79c3a37852480e4b9d2626a8f47395d2abb489ca3013af210bd2a5fc465f6d9c` |
| `C/run/controller.sqlite3` | `4d17176f2d4117e2d5e73e7aa2b1e5aa8a7a95894936edee89ccaf0e8f9f5b93` |
| `C/run/plant.sqlite3` | `87e5cf550e85658a5cc297458fa6396f6cf896e5b608c4d2e3812f8322528213` |
| `C/run/feedback_report.json` | `47b15d73706fdf03356efd3a3045803fa352ab421ebea7ef4233ba395ad20e08` |
