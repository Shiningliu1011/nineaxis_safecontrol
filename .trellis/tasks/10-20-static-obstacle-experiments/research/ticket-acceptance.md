# #172–#176 验收证据

## 本次来源与执行范围

2026-10-20 从 GitHub 读取 #171、#172–#176 的最新正文、评论及原生依赖。
上述 issue 均为 OPEN，评论为空，parent 为 #147。依赖顺序为
#172 → #173/#174，#174 → #175，#173/#175 → #176。

核查 worktree 为 `.scratch/worktrees/static-evidence`，分支
`codex/static-evidence`，开始时 HEAD 为
`35a1a61022d451ba79c5f222d7aef09f8f72ab0c`，工作区无改动。
`git merge-base main 35a1a61` 返回
`34a2e343e1c0402974f5ccc51917556c67487c1c`，与 `main` 和原开发基准一致。
缺少执行器限制快照的评价处理提交为 `0a4559b`。

加载 `.scratch/host-recovery/env.sh` 后，将 worktree 的 `src` 和
`portable_oscbf` 放在 `PYTHONPATH` 前部；实际模块路径已核查。
FCL 扩展使用本机已构建的
`build/robot_safecontrol_moveit/src/robot_static_fcl.cpython-310-x86_64-linux-gnu.so`。
本次数值验证与重放使用 `taskset -c 16-27`，用于功能验收。
运行时间验收继续由独占 CPU 0–15 的共同周期测量负责。

## 实际命令和结果

完整脚本及日志保存在
`output/static-ticket-evidence-20261020/commands/`：

| 入口或日志 | 范围 | 实际结果 |
|---|---|---|
| `run_checks.sh`、`checks.log` | 共享参考、配置、记录、外部几何、mesh、自碰撞、Ruckig、调度、评价、控制、QP | 84 passed，2 deselected，339.65 s |
| `headless.log` | 两个 MuJoCo 场景模型用例及 controller 失败周期记录；独立 ROS domain | 3 passed，39.99 s；没有视觉检查 |
| `missing-limits-green.log` | 执行评价、逐反馈间距、恢复规则、连续几何 | 15 passed，50.80 s |
| `replay_tickets.py`、`s0-replay.log` | 本机完整 S0 的独立副本，两次重算 | 两次结果相同，pass，61.68 s；来源文件 SHA-256 前后一致 |
| `replay_tickets.py`、`s1-replay-green.log` | 历史完整 S1 的独立副本，两次重算 | 两次结果相同，任务 pass，综合 insufficient_evidence，66.77 s；来源文件 SHA-256 前后一致 |
| `audit_records.py` | 历史 S1 全部候选、准入、反馈与球形障碍物 QP 行 | 1,830 次 QP、1,734 个获准命令；未经准入激活为 0 |

执行评价的行为检查使用实际 `StaticControl`、`MotionPlanner`、
`ExecutionSchedule` 和 SQLite 记录。测试保存候选的实际初态及几何记录，
移除 manifest 的限制快照后核查两项评价均返回 `insufficient_evidence`。
完整快照继续用于实际限制及进度检查。

两次重放使用 `evaluate_run(source_copy, resource_root=worktree,
output_directory=independent_directory, save_samples=False)`。
原始文件仅复制并计算 SHA-256；独立副本、派生报告及验证清单均在
`output/static-ticket-evidence-20261020/`。

## #172：可重复的 S0 仿真与反馈评价

本票逐项证据已具备，等待主会话发布并关闭。

| 验收项目 | 证据 |
|---|---|
| 工件、台面、270.6/220.6/50 mm 几何 | `test_s0_uses_declared_geometry_and_complete_path` 核查真实 MAT 路径及配置；headless 模型检查工件、台面及初态 |
| 固定初态、sim、完整蝴蝶及显示一致性 | 本机 `output/static-storage-control-01`；配置只接受 sim；两个 headless 用例读取同一配置构造 MuJoCo |
| 500 Hz、反馈与命令独立记录、时间基准 | 87,116 条有效反馈，仿真 174.230 s，仿真频率 500 Hz，墙钟 499.128 Hz；SQLite 独立保存 controller/plant 流及 simulation/monotonic/ROS 时间 |
| 反馈 FK、路径/参考/轴向/作业距离及有序覆盖 | `s0/first/feedback_report.json` 由实际反馈重算，完整参考及有序覆盖均通过 |
| 跟随、终点、终止及来源身份 | completed；终点连续满足 3.628 s；manifest、path、result、数据库及评价源码身份全部保存 |
| 重放相同、异常保留数据 | 两次独立重放逐项相同；真实 SIGKILL 记录恢复测试、实际 controller 失败记录测试通过 |
| 有效性与缺少证据 | 缺少限制快照明确返回证据不足；非有限输入、缺少命令、数据库破坏测试通过 |
| 入口及实际闭环 | `scripts/static_obstacle_experiment.py` 的 run/replay/show 与 `docs/static_obstacle_experiments.md`；本机 S0 已运行真实 ROS controller 与 Ruckig plant |

本机 S0 的路径 RMS/P95/最大值为 0.056300/0.119455/0.172592 mm；
工具轴 RMS/最大值为 0.008842/0.062754°。最小外部/自碰撞间距为
35.616884/15.785837 mm，20 个外部组合、36 个自碰撞组合均完整有效。
QP 1,729/1,729 成功；全部反馈的参考进度重算残差为 0。

## #173：共享路径参考的五维跟随

本票逐项证据已具备；tracker 关闭顺序位于 #172 之后。

| 验收项目 | 证据 |
|---|---|
| 同一进度的三维位置、二维工具轴、roll 自由 | `moving_reference_task.py` 的公开任务输出及 `test_roll_is_free_and_antipodal_direction_cannot_authorize_motion` |
| 关节运动项与参考推进项、单位 | 共同模型 `e_dot = J_e u - xi_e nu`；s 使用 m、nu 使用 m/s；文档与三个求解方式使用相同公共任务 |
| q/s 独立导数，直线、圆弧、连接 | `test_moving_reference_derivatives_match_independent_fk` 四组真实机器人 FK/独立 SciPy Slerp 有限差分通过 |
| 0–π 真实夹角 | 同向、1e-10、0.8、π−1e-8、π 五组通过；反向奇异任务不产生获准运动 |
| 实际 S0 定量阈值 | 上述本机 S0 的路径和工具轴五项阈值均通过 |
| roll 与交叉位置顺序 | roll 1.4 rad 下五维误差为零；两次经过交叉位置的连续局部投影测试通过；实际完整有序覆盖通过 |
| 共同任务及说明 | `docs/static_obstacle_experiments.md`、`tests/test_moving_reference.py` 及共同 `StaticControl` 入口 |

## #174：单个静态障碍物的臂身避让

本票仍需当前开发 S1 的实际约束触发证据。

配置、固定坐标、OBB、逐对象 30 mm 约束、工件/台面查询、几何错误拒绝、
独立反馈评价、headless 显示及实际 S0 回归均有通过证据。
历史 S1 的初态、工具通道和 S0 进入要求间距依据保存在
`10-20-static-external-collision/research/scene-evidence.md`。

历史 `s1-joint-full-geometry-03` 由当前评价器重算：
88,248 条有效反馈，完整有序覆盖，路径 RMS/P95/最大值为
0.059308/0.125573/0.199185 mm，工具轴 RMS/最大值为
0.008696/0.043078°；最小外部间距 40.604505 mm；
球形障碍物对应的最小间距为 Link3 的 43.032795 mm。

全部 1,830 次真实 QP 中，球形障碍物约束的最小缩放余量为
0.06846281219494027，达到 1e-8 活跃阈值的记录为 0。
该记录不能证明本票要求的实际外部约束触发。
当前 `development-scenes-05/s1.yaml` 的完整 D 组运行等待 sim 使用窗口。

## #175：全部非相邻连杆的 mesh 自碰撞约束

已核查的几何行为满足本票要求；当前开发 S1 回归及 #174 依赖尚待完成。

| 验收项目 | 证据 |
|---|---|
| 闭合 mesh、FCL、全部 36 个非相邻组合 | 来源清单、URDF、十个 STL 及 provenance 的 SHA-256；实际文件顶点/三角形一致性测试通过 |
| 每组 10 mm、组合清单入记录 | S0/S1 manifest 保存完整组合；独立反馈分别计算 36 组，逐组结果保存在 clearance_report |
| 分离、接触、相交、包含、最近对象切换、无效查询 | `tests/test_static_self_collision.py` 使用真实 FCL、trimesh 和实际构型全部通过 |
| 最近点、关节梯度独立核对 | 独立凸体 SLSQP 最近距离、全部 36 组有限差分梯度通过 |
| 外部与自碰撞同时使用、拒绝无效候选 | `StaticControl`、`certify_motion`；实际 S1 每次候选含 66 个对象组合，72 个 trial 因 joint_position 拒绝 |
| 从反馈逐组重算并记录最小对象 | 本机 S0 36/36，最小 15.785837 mm；历史 S1 36/36，最小 16.999884 mm；所有有效反馈均满足要求 |
| 缺少组合不能通过 | `tests/test_static_clearance.py` 核查实际状态与完整组合 |
| 来源、范围、距离说明 | `docs/static_obstacle_experiments.md` 及本任务 mesh-source.md |

## #176：Ruckig 处理后的命令检查与进度结算

本票仍需当前格式的完整 S1 回归。

实际 Ruckig 独立计算、初始速度/加速度、完整停止过程、内部转向位置、
关节连续范围与全部几何检查通过。真实调度测试覆盖准入、过期、参考不一致、
停止期间参考保持；完整反馈评价覆盖缺少命令、错误进度及缺少限制快照。
本机 S0 的关节限制、参考结算及全部几何通过。

历史 S1 记录有 1,742 个完整周期、8 个无获准候选周期、1,830 次 QP，
1,734 个获准命令。全部有效反馈的结算残差为 0，未经准入激活为 0。
五次实际暂停后恢复的参考变化为 0，暂停期间存在真实构型调整。
历史记录缺少 `execution_limits` 与 certificate 的 `covered_pairs`，
重放综合结论保持 `insufficient_evidence`。

## 数据位置、来源身份与校验值

共同目录前缀为 `/home/lsn/robot/robot_safecontrol/`。
历史 S1 原件属于主机恢复资料，归档提交为
`68a52279423318d3773e8d6f61c21c10f1d53a24`；恢复的完整逐文件来源身份
见 `docs/host_recovery_20261020.md` 和
`output/host-recovery-migration-evidence/project-files.json`。
本机 S0 原件为 `output/static-storage-control-01/`。
原始绝对路径及 manifest 内容在独立副本中保留。

| 文件 | SHA-256 |
|---|---|
| `output/static-storage-control-01/manifest.json` | `77c8cedcbb000efa22c7533218eaa19885347690b4cd5dddc8ce1e50d6c04fa3` |
| `output/static-storage-control-01/controller.sqlite3` | `4ec8e329dcc7a819343d3dd981f902f79108ebd35b39405fa822d2f94eafae3c` |
| `output/static-storage-control-01/plant.sqlite3` | `ec9bf39b5cae023761752ecc7172c65e690cde61dd0f367141477a80a7a516f6` |
| `output/static-experiments/s1-joint-full-geometry-03/manifest.json` | `15305f468e69991631c50415c4eeb499caf1027b7992100fb10cf94e513aeae7` |
| `output/static-experiments/s1-joint-full-geometry-03/controller.sqlite3` | `732c294be77e529afc11ee42f46f07149df1e8bd0b6440d609c5380318cb02bf` |
| `output/static-experiments/s1-joint-full-geometry-03/plant.sqlite3` | `c7b339aa52cfe9f28d7f8ea511743544381662dff89ba87d78d0be69b155ef0d` |
| `output/static-experiments/s1-joint-full-geometry-03/feedback_report.json` | `fdded71875fa27e5f435c0f3e375657c928423a2827dab883c6a4feeec221f6e` |
| `output/static-ticket-evidence-20261020/s0/verification.json` | `ce2454f1e8ad85bc1110a134a14265c2ba7d014b708041b3de7b51de1236a3b2` |
| `output/static-ticket-evidence-20261020/s1/verification.json` | `4cac7875ed75574434b85a2958d8c146ca057c8f8f782a5ae908b347be7188c5` |
| `output/static-ticket-evidence-20261020/s1-legacy/raw-audit.json` | `557362f4da391425571da634477bce7b6717d864e4e7b9eb20dc2af960d51a29` |

各 `verification.json` 另行保存来源目录全部顶层文件及输出报告的 SHA-256、
CPU affinity、两次重放一致性和原件前后比较。
上述记录只支持对应开发配置；正式场景、共同周期及全部正式实验继续由
#177–#183 的验收负责。
