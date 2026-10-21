# 容量保护运行的场景证据

本轮为只读离线核查，CPU 22–27，没有启动仿真或修改生产代码。
工作分支普通合并至 `3e0f27f`。新增原始运行源码为
`40d43bb3712e3f99c11a59750a59d1ee65ecb525`，代码与资源归档
`output/static-capacity-20261020/guarded-preflight-01/source/sources.zip`
SHA-256 为 `c381f34a51223f99f575c26eaed9b6045601e8ea200efaadb1576e20ca8560a9`。

完整原件位于
`/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/guarded-preflight-01/`。
每个项目的 `run/` 包含数据库、manifest、配置、结果和反馈报告。
本轮分析目录为
`/home/lsn/robot/robot_safecontrol/output/static-scene-validation-20261020/guarded-scene-evidence-01/`。

主验证汇总 `output/static-capacity-20261020/guarded-preflight-01/validation-summary.json`
SHA-256 为 `1753d5d603d8997a637cbeb25ec757720bfb8fbe2e24bb23b104082d1842f836`。
六份新增完整来源均通过 `validate_run_report`，重新核对两份数据库、
路径、配置、结果、报告及派生文件身份；各 manifest 的源码身份与该汇总相同。
原有可行性来源沿用已校验清单，保持全部历史记录。

## 新 S2 的实际共同约束

固定场景采用 development-scenes-07 的原候选 7、10，
共同 `posture_speed_limit=0.02`、控制周期 100 ms、外部间距 30 mm。
三份运行均完成完整路径，功能与生命周期通过。

| 方法 | 实际恢复次数 | 不同连杆与障碍物同时距离不超过 31 mm 的候选 | 共同生效的获准 trial |
| --- | ---: | ---: | ---: |
| D | 0 | 42 | 32 |
| A | 33 | 0 | 0 |
| B | 0 | 42 | 32 |

检查逐个读取实际获准 trial 的完整 `G`、`h` 和 `solution`，
按原记录的几何梯度、CBF 参数和缩放重新核对相应约束行。
两条约束分别关联不同连杆和不同障碍物，均要求
`h - G @ solution <= 1e-7` 才记录为共同生效。
全部候选、获准 trial、距离记录及逐对象身份保留。

D/B 最接近共同对象均为 Link2/obstacle_1 与 Link5/obstacle_2，
两个距离中的最大值分别为 30.043581、30.043777 mm。
A 的同类最接近状态为 48.035178 mm，本次未观察到共同生效。
D/B 各三个代表状态、A 一个代表状态经独立 FCL 查询，
相应距离与实际候选记录差值均不超过 `1e-12 m`。

| 检查文件 | SHA-256 |
| --- | --- |
| `d-s2-simultaneous.json` | `a70f1d24f082af353c5b66f7af38b4b97d93d69f19becb49acb9642cefab6772` |
| `a-s2-simultaneous.json` | `0890cfe6191be930443c107508c49399359b80afe8439f43b89d1dceb03f20c8` |
| `b-s2-simultaneous.json` | `8c469844eab79e30d5a1564bf960fc3f1b2bbdc480b5a246a0ec03041ae408c0` |

三初态、三档进给的九组完整数值证据仍独立保存在
`s2-numerical-matrix-01/summary.json`，SHA-256 为
`bc8a5b0f2438bc3a4da698f3adfcd59df48fa0de84b0de70eb7b1aa419f22792`。
九组均到达终点，均有共同生效约束。数值记录和 ROS 完整反馈分别标记。

## S4 九组完整结果

两份新增 D/S4 完整运行已通过，独立见证与实际恢复事件均保存在
各自 manifest 和反馈报告中。最新九组 D 的完整功能全部通过。

| 初态 | 进给 0.5 的恢复次数 | 进给 1.0 的恢复次数 | 进给 1.5 的恢复次数 |
| --- | ---: | ---: | ---: |
| 1 | 6 | 3 | 1 |
| 2 | 6 | 1 | 1 |
| 3 | 6 | 1 | 1 |

所有项目 workers 均为 `[0,0]`。
initial1/feed1.5 与 initial3/feed0.5 使用容量保护的新完整来源，
其余七项沿用已有完整来源。早期失败原件及诊断保留在此前目录，
汇总中的 `previous_attempts` 保存关联身份。
E/S4 新完整运行同样通过，实际恢复 26 次，作为独立方法来源登记。

`s4-ros-matrix-summary.json` SHA-256 为
`cb3250d2283a70569c52b2d9bc5b0a057fb141df9d493eb6c7f7e2a52c7fbbc8`。
此处记录功能与场景证据，最终共同周期仍由专用测量验收。

## 72 配置的完整物理路径来源

配置定义继续使用 `configuration-feasibility-02/configurations.json`，
SHA-256 为 `18174f4272abe57fc9ca7889503eeaf07a224ef194085cdb17f795cdccec88c6`。
本轮新增六份完整来源，已有全部方法的有效来源继续保留。
当前 33/72 个配置已有完整 ROS 物理路径来源，其中新增
S2 初态 1 的三档进给。各进给的方法功能仍分别验收。

| 更新清单 | SHA-256 |
| --- | --- |
| `feasibility_matrix.json` | `bec38ea319f28b87d4a2218c9e8ee5aadb60a7cc393513250ee429bd990a948a` |
| `run_inventory.json` | `9eadf4549964b49e5f3d835a6993bfce6745d545d1ac26b78bad2627136c3250` |
| `summary.json` | `61d61b02ad8025c868d3480b8109d3290042fc4a28ef4bfe121625c955d8ee7d` |

当前 `freeze_formal` 的 `feasibility_runs` 入口要求完整 ROS 反馈报告，
`completed`、任务及几何通过，并校验原始记录身份。
物理身份包含场景、初态、执行限制、路径与运动学，未包含进给倍率。
因此 S2 初态 2、3 各一份完整 ROS 来源可分别覆盖该初态的三档
物理路径可行性；现有完整数值记录继续保留其明确证明范围。
主会话已安排这两项补充前置验证，本轮没有派发或启动仿真。

## 两项未执行的运行计划

计划文件为本分析目录的 `pending-s2-initials-plan.json`，
SHA-256 为 `172575eb2694e2d57e7214fc3a7d1e30a8a38e6c4c324b3bb08d1392f4a60935`。
两份配置均来自已完成九组数值矩阵，几何、初态和共同控制参数逐项核对。

| 初态 | 配置相对本数据根目录的路径 | 文件 SHA-256 |
| --- | --- | --- |
| 2 | `s2-numerical-matrix-01/initial-2-feed-1.yaml` | `89ad48c5d545aef203331de6384628950809763b82dd47eb0327dc303e4907a4` |
| 3 | `s2-numerical-matrix-01/initial-3-feed-1.yaml` | `30ab6b4989d5d91bb1d5262549502cc4b51f9f61d14095c8c9b1bc92e18e921a` |

计划保存完整预算生成和 `validate_static_lifecycle.py` 参数：
D、feed 1、76 组几何对象、CPU 0–15、domain 200。
环境入口为主仓库 `.scratch/host-recovery/env.sh`，`resource_root`
使用主仓库绝对路径。主会话执行时创建
`output/static-capacity-20261020/guarded-s2-initials-01/` 控制目录，
使用当时源码和目标卷身份即时生成预算，再在安静窗口串行运行。
原件输出计划位于同一 NTFS 数据根目录的 `guarded-s2-initials-01/`。
本轮没有生成运行预算或创建上述仿真输出目录。

分析脚本归档在本分析目录的 `audit_guarded_scene_evidence.py`，
SHA-256 为 `e6e451eb9836ba151c4c925600c66735101ddc129471f91ecfe89b5f2313a575`。
本轮脚本退出 0，逐来源、约束行、FCL 和配置身份断言均通过。
