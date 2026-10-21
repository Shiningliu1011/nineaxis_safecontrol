# S5 周期计时诊断准备

2026-10-21。研究对象为运行源码 `40d43bb3712e3f99c11a59750a59d1ee65ecb525`
上的 D/S5。当前共同周期测量独占 CPU 0–15；本次只读取源码、规则与小型诊断
JSON，没有读取数据库、启动 sim、执行性能测试或修改运行模块。
执行 owner 为 implement-spec，记录归属为 Trellis 的 static-formal-experiments。
完整周期的统一计时设计见
[完整周期计时边界](complete-cycle-timing-design.md)。后续性能复现采用该设计完成
后的统一口径，本记录不要求并行启动新的性能任务。

## 已有观测与身份

输入为主仓库
`output/static-capacity-20261020/guarded-preflight-01/s5-cycle-parts-three-volumes.json`，
实际核对 SHA-256：
`02803c49d3d49114039c06f73aa4ddea2d4df5ad0b733dbfc99f5844f979b33e`。
该 JSON 的分析程序 SHA-256 为
`e77adaa0c17823512247d12e254c2690bcd157f8ca9bca36c3372285fd3954a8`。

三份记录各有 346 个周期。目标由 controller 数据库的 `sequence=179`、
`stream=candidate`、`start_step=2200` 和七次 trial 共同确定。
`sequence` 是 SQLite 记录序号；`proposal_sequence` 与 `command_id` 必须从
真实记录分别取得。

| 输出卷 | 目标记录前区间 ms | 目标记录后区间 ms | 完整 duration ms |
| --- | ---: | ---: | ---: |
| ext4 | 76.480481 | 18.865123 | 95.345604 |
| 第二卷 NTFS | 49.416718 | 19.452093 | 68.868811 |
| 第三卷 NTFS | 75.422883 | 19.762361 | 95.185244 |

第三卷 `sequence=183` 为五次 trial，两个区间分别为 46.288002 和
37.115855 ms。三卷首次 `sequence=1` 的记录后区间分别为 43.419658、
44.486775、44.119185 ms。现有观测没有各区间内部的时间，也没有证明三个
目标输入及前序复用状态逐值相同，尚不能据此判定 QP、FCL、JIT 或文件系统是原因。

| controller 原件 | SHA-256 |
| --- | --- |
| `output/static-capacity-20261020/guarded-ext4-s5-01/run/controller.sqlite3` | `2930af21c73acbd31f1f16df1047bdb7ff93ede814e808bfb6a64f7d3e0deb97` |
| `/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/guarded-preflight-01/d-s5/run/controller.sqlite3` | `893133a140aed31d509b9b0fdc4fc4c72e613928cd28510ee4e65ea1004ad084` |
| `/media/lsn/Windows/robot_safecontrol_static_experiments_20261021/guarded-s5-01/run/controller.sqlite3` | `62872ebb40ef995ec7e33b150f963bff7e6a90f420f1865a22667fc3a7b9a0d6` |

数据库身份来自上述 JSON，本次没有重新计算大型文件 SHA-256。
主会话提供的完整运行 P99.9 分别为 85.834、76.788、91.1207 ms。
正式判断仍依据完整报告中的 P99.9 与固定周期的 80% 比较；目标单条记录超过
80 ms 仅作为诊断信号，不能替代全部周期的验收。

## 源码确定的计时边界

行号以本次读取版本为准，以下运行模块与 `40d43bb` 内容相同。

| 入口 | 已确认行为 | 后续应观测的边界 |
| --- | --- | --- |
| `src/robot_safecontrol_moveit/static_experiment_nodes.py:253` | `callback_started` 在容量检查、active motion 重建及线程提交前取得 | callback 开始、submit、线程实际进入 propose、future 完成、主线程接收完成 |
| 同文件 `:325` | candidate 时间在 `RunRecorder.append` 调用前取得 | candidate 编码与 SQLite 写入属于记录后区间 |
| 同文件 `:347`、`:356` | command 同步记录后通过 ROS 发布 | command 编码、写入、publish、plant 收到消息 |
| 同文件 `:81`、`:101`、`:104` | plant 重建 Ruckig motion，并在线程中独立 certify；timer 检查完成 | 重建、线程等待、certify、完成至 timer 处理、admission 写入及发送 |
| 同文件 `:214`、`:227` | controller 写入 admission 后记录 cycle | 收到回复、admission 写入、cycle 计时终点、cycle 写入结束 |

因此记录前区间还包含回调准备、线程调度、完成轮询等待和记录字典构造。
记录后区间同时包含编码、SQLite、命令传输、plant 检查和回复处理。
`_record_cycle()` 在 cycle 自身写入前计算 `duration_s`，自身写入耗时需要
另行观测；这是一项计时口径事实，尚无证据说明它解释目标记录的尖峰。

`RunRecorder.append()` 的内部顺序为值检查、MessagePack、zlib6、zstd3、
选择较短 payload、SQLite INSERT、容量计数更新。它返回整段实际耗时；
controller 当前没有保存这个返回值。plant 只为超过 20 ms 的反馈写入保存
`recording_delay`。两种编码均实际执行，不能用 payload 大小代替编码或 I/O 耗时。

## warmup、几何检查与复用入口

`StaticControl.warmup()` 创建独立 `preparation` 实例，执行初态 propose，
再执行 terminal QP。controller 随后创建自己的 ThreadPoolExecutor。
真实实例保留空的 `_previous_solutions`、`_previous_motions` 和恢复历史；
`tests/test_static_controller_warmup.py` 已覆盖首条记录的来源及 terminal QP
编译边界。初态预热是否覆盖目标的全部执行路径，需要实际编译事件及参数形状验证。

`certify_motion()` 位于 `portable_oscbf/work/static_motion.py:150`，包含
Ruckig 状态采样、`transform_derivatives_batch()`、批量距离、运动范围和
`lower_bounds_batch()`，并按未证明区间继续检查。
native 边界是 `StaticGeometry.batch_distances()` 调用的
`robot_static_fcl.measure_poses()`；`portable_oscbf/native/static_fcl.cpp:136`
在批量查询期间释放 GIL。完整 certify 耗时包括 Python、NumPy、SciPy 和 native
工作，不能把整个函数耗时归为 native 查询。

后续每次 trial 同时保存 `certificate.reason`、`checked_intervals`、
`covered_pairs`、批量次数、各批次状态数和 pair 数；据此比较真实工作量。
原有 `minimum_interval_s=1e-5`、`maximum_intervals=4096` 和安全阈值保持。

QP 入口为 `StaticControl._solve_velocity()` 与
`static_velocity_solver._solve_problem()`。后者按完整问题逐值相同或零进给
条件准备可复用候选，再由 `_solve_or_reuse()` 实际检查最终健康状态。
需要观测求解耗时、iterations、KKT、hard_violation、
`qp_reused_from_trial`、`qp_reused_from_proposal`。
motion 复用要求九轴速度完全相同；跨 proposal 还要求初态、周期、geometry、
scene、planner、全部限制一致。需要同时保留
`motion_reused_from_proposal` 和实际执行 certify 的次数。
同一 proposal 内的 motion 复用当前没有独立来源字段，不能仅由该字段为空
推断 certify 一定重新执行。

## 后续复现实验顺序

1. 等共同周期测量结束，确认没有其他 sim 或性能任务占用 CPU 0–15。
   保存源码及 native `.so` 身份、恢复环境、CPU affinity、配置、预算、卷 UUID。
   从原运行 manifest 读取确切配置与 scene evidence，不推断缺失路径。
   三卷的原始数据库与全部失败运行保留。
2. 使用现有 `read_record(..., stream="candidate", sequence=179)` 提取目标，
   并读取直到目标的 controller 记录前缀与对应 plant 反馈。按 JSON 中的原件
   SHA-256 核对身份。比较三份目标的 initial、reference progress、全部七个
   QP 输入、速度、certificate、复用来源及前序事件；任何差异单独报告。
3. 在 ext4 用原配置、100 ms、D、原始进给和容量守卫执行三个独立完整 S5，
   全部保留。以现有 evaluator 的 `execution.timing.p999_s <= .08` 为完整
   计时判据，同时核查功能、准入和退出状态。报告目标尖峰的出现次数与范围；
   不预先假定新运行的目标 SQLite 序号仍然为 179。
4. 构建诊断专用重放入口，以真实记录前缀依序调用同一 `StaticControl`，
   在独立准备对象 warmup 后，通过单 worker 的 ThreadPoolExecutor 执行。
   每次重复建立新的真实控制实例，恢复完整前序调用与实际 admission rejection。
   `executed_feed` 按原 `source_feedback_step` 的 active command 及原
   `_tick()` 条件计算；初始 command 0 使用真实初态 `ExecutionSchedule`。
   不把 `executed_feed` 全部写成零，也不直接注入预期缓存结果。
5. 重放前缀的 trial 输入、输出、阶段、证书、复用来源和事件必须与原件匹配。
   验证不匹配即停止归因并保存差异。确认前缀重放正确后重复 31 次，单独记录
   目标 propose、每次 QP、Ruckig 与 certify。重放没有覆盖 ROS 和同步记录，
   因此结果只用于定位控制计算，不能替代完整周期的准入。
6. 在诊断分支添加限定 `start_step=50` 及 `1850–2350` 的分段时间观测，
   执行同样三次完整 S5。正常候选、命令、反馈及同步记录全部保留。
   诊断元数据在内存中保存，退出后单独写入，避免逐字段日志增加被测写入。
   针对每个 command_id 关联 controller、plant 的 monotonic 时间。
7. 如果短重放没有复现尖峰，保留这一结果，以完整 sim 的分段时间继续缩小范围。
   在取得能捕获原症状的实际入口前，不进入原因判定或优化修改。

未来诊断入口至少需要这些时间点：callback 开始、容量检查结束、submit、
propose 进入、problem inputs 完成、各 trial 的 solve/planner/certify 前后、
propose 返回、future 被主线程处理、记录字典完成、每个 append 前后、publish、
plant 接收、plant certify 前后、plant admission 发布、controller admission
接收、cycle 写入结束。编码内部再区分值检查、pack、zlib、zstd、INSERT。
时间点使用单调时钟；native 查询可同时记录当前线程 CPU 时间，但 JAX 可能使用
其他线程，不能直接把 wall time 与当前线程 CPU 时间之差解释成调度等待。

JAX 编译日志只在单独的身份验证运行启用。性能运行保持原来的同步方式；
`np.asarray(solution)`、`float(health.kkt_residual)` 等现有读取已参与同步，
新增 `block_until_ready()` 的影响应单独说明。探针增加的开销单独报告，正式
计时使用移除探针后的完整运行，不从测量中扣除估计开销。

## 可用命令入口

以下是共同周期测量结束后的命令模板，本次没有执行。`s5_diagnostic_config`、
`s5_scene_evidence` 从原 manifest 取得；新输出目录必须不存在。准备步骤的编码
审计会读取数据库，须在性能任务全部结束后执行。每个重复使用独立预算和输出。

```bash
source /home/lsn/robot/robot_safecontrol/.scratch/host-recovery/env.sh
python scripts/prepare_static_storage_budget.py \
  --engineering-evidence output/static-capacity-20261020/guarded-capacity-review-01/engineering-plan/engineering_evidence.json \
  --resource-root /home/lsn/robot/robot_safecontrol \
  --config "$s5_diagnostic_config" --method D --feed-factor 1 --period-s .1 \
  --volume /home/lsn/robot/robot_safecontrol/output --maximum-pairs 66 \
  --output "$s5_diagnostic_budget"
taskset -c 0-15 python -m robot_safecontrol_moveit.static_experiment run \
  --resource-root /home/lsn/robot/robot_safecontrol \
  --config "$s5_diagnostic_config" --method D --feed-factor 1 --period-s .1 \
  --scene-evidence "$s5_scene_evidence" --storage-budget "$s5_diagnostic_budget" \
  --domain 200 --output "$s5_diagnostic_output"
```

先核对原 manifest 的 period、feed factor 和 pair 数确实为上述值；存在差异时
使用原件值并记录。原入口最终依据综合 verdict 返回退出码，读取报告时必须分别
列出 functional 与 runtime，识别性能症状及其他失败。

本次 diagnosing-bugs 的工作停留在反馈入口准备：没有执行新的复现命令，
没有声明完成可重复的失败验证、原因判定或性能修复。当前交付为可执行步骤与
观测边界的研究记录；实际执行由主会话在共同周期测量结束后调度。
