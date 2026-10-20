# E 方法 S1/S4 开发运行

本机原件位于
`/home/lsn/robot/robot_safecontrol/output/static-method-evidence-20261020/`。
来源为 `codex/static-method-evidence` 的
`e6fe012ea3eb77ad1adaae5865a4d0d365e522af`，环境入口为
`.scratch/host-recovery/env.sh`，固定 CPU 8–15、ROS domain 200。
配置来自 `development-scenes-06`，周期100 ms，
共同 `posture_speed_limit=0.02`。同期存在其他开发验证，
耗时记录用于开发观察。

## 实际结果

| 运行目录 | 完整运行结果 | 周期数 | P99.9 / 最大耗时 |
| --- | --- | --- | --- |
| `e-s1-current-01` | 完整跟随，功能与运行时间均通过，workers `[0,0]` | 1707 | 55.459798 / 72.635040 ms |
| `e-s4-current-01` | `fault`，反馈 step22027 的 age=201.302602 ms，workers `[0,1]` | 440 | 93.307392 / 120.817497 ms |

S1 全部外部对象间距及36对自碰撞间距通过；有序路径和终点通过。
S4 原件完整保留，尚待最终来源、容量保护启用后的完整仿真。
`static-method-evidence-20261020-remaining-02.service` 已经结束；
B/S5 尚未启动。

两个运行的 manifest、源文件和数据库在审计前后保持相同；
格式3数据库的 `integrity_check` 均通过。
来源归档复用
`output/static-capacity-20261020/ntfs-s5-E-02-source/sources.zip`，
SHA-256 为
`d9965035536bb13a8561f1fa7e9ad6fdbe0ab5d178cc9250df702e175fda9c28`。

## 独立数值核查

使用 `commands/verify_current_sns.py` 读取全部实际 trial，
核对广义上下界和完整任务残差，并针对代表输入运行
SciPy HiGHS 最大缩放与独立 null-space/DLS 计算。

| 运行 | trial 数量 | 负上界数量 | 饱和数量 | 最大限制超差 | 最大任务残差 |
| --- | --- | --- | --- | --- | --- |
| E/S1 | 1710 | 61 | 164 | 4.376e-17 | 6.266e-16 |
| E/S4 | 447 | 0 | 76 | 0 | 4.569e-16 |

实际最小 task_scale 均为1。S1 的6个、S4 的5个代表问题，
HiGHS 最大缩放均为1；独立 DLS 的任务差分别不超过
`2.163e-16`、`2.435e-16`。这些数值检查通过，
S4 的完整运行结果仍为失败。

## 校验值

下表路径相对于上述原件目录，均为 SHA-256。

| 文件 | 校验值 |
| --- | --- |
| `e-s1-current-01/controller.sqlite3` | `d734aea1514b4b724ed23884b0aff9d7bfb88aeef1a073de4d8a33a601f980d3` |
| `e-s1-current-01/plant.sqlite3` | `833cef3543c1bf8239315f400773877255f75c1f36356a121ceb9c61504b9642` |
| `e-s1-current-01/feedback_report.json` | `5f3ccdbb504f84cadafaf49dfc50db67373f47b59d34e633d29fdaab4646d08b` |
| `e-s4-current-01/controller.sqlite3` | `c399638cd4dafa1b899a61e1dc6fd0f125304d30169759ee1939f02ef0799b58` |
| `e-s4-current-01/plant.sqlite3` | `ae9cb0dd2b7abe5acf4c50309f7dc1df5f6f2601172a07bedab3d9b4bc3cc14a` |
| `e-s4-current-01/feedback_report.json` | `8bcf5dd94057bac697b3906cd30a8d185c3da2e139a2de703083031749a37d9e` |
| `commands/e-s1-current-01-audit.json` | `0f9a158553e007c357eda8b2663eb71972a8703d6c348b2dd3d74a712f21a23c` |
| `commands/e-s4-current-01-audit.json` | `3df8441d1096f55b2dfec0a6008b28ee780d0abde3d3e6a84807c0c24ed5c77b` |
| `commands/e-s1-current-01-solver-dls.json` | `bcdc62b002d5e8a537db456d6ec8e453ccd316011ff98405ea169bcec88084f5` |
| `commands/e-s4-current-01-solver-dls.json` | `6511199b4e05cffd0f3f61d823aafd1b2ac3145d415f538931026680629f8d69` |
