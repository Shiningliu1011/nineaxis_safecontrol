# 跟踪执行与过渡结果验证（2026-10-01）

对应规格 [#146](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/146)，实现说明见[跟踪执行与过渡结果](../tracking_execution.md)。行为基准为 `5530ab8`。本次使用真实控制内核、ROS 2 Humble、MoveIt/AEB 和 `OscbfPlant` 本地仿真。

## 环境与数据身份

- 执行机器：`lsn`，Linux `6.8.0-138-generic`，x86_64，Python `3.10.12`，JAX CPU。
- 生产配置：`config/oscbf_controller.yaml`，SHA-256 `a1c85415a7a7055a980276eb0ce645697a213d5179f8b1b36c348145e6e00176`。
- 数值对照输入：`tests/test_tracking_run.py` 中的 `START`；轨迹为 `data/nurbs/ik_input.mat`，使用生产配置中的变换与参数。
- 每次真实控制器启动产生 `runtime_snapshots/`，记录有效参数、模型、源码及轨迹几何身份。报告记录 `kernel_candidate` 测量边界与样本 SHA-256。
- 中间结果：项目 `.scratch/tracking-transition-implementation/`。各 ROS 运行使用独立 domain；进程清理仅针对测试自身启动的进程。

本次实现源码的 SHA-256（目录为 `src/robot_safecontrol_moveit/`）：

| 文件 | SHA-256 |
|---|---|
| `tracking_run.py` | `7a0411bb0c21d5769f76b75a2f52868b43f9bfe4bb655ff03fd655022cb100b8` |
| `oscbf_controller.py` | `e57b650b687eece0cba756d5dbda7185aaa84466eda49c0800f7738437cd085f` |
| `transition_executor.py` | `07c45d4a978908c32131b9767ac09dd6f9abe99beb72031ca85810b66b02fa71` |
| `transition_planning_server.py` | `977bc3bf2ec3a6d2443ce62b2d6740a56981ffeabc98d500dd032aaa2844334c` |

## 行为验收入口

下表列出每项规格的可运行入口。完整测试集的运行结果单独记录，未完成的运输验证明确保留。

| 编号 | 测试或场景 | 验证内容 |
|---|---|---|
| A01 | `test_tracking_run.py::test_start_gate_consumption_and_interruption`；真实 ROS handoff | 两种开始配置、首次与重复开始、消息门控 |
| A02 | 同上；`test_oscbf_full_flow_e2e.py`；400 步数值对照 | 无新状态时步数不增长；正常输入、路径推进与命令 |
| A03 | `test_tracking_run.py::test_direct_steps_have_no_lifecycle_samples_and_own_results` | 直接控制步不采样；公开结果隔离；未测量距离为 `None` |
| A04 | `test_tracking_run.py::test_real_short_path_endpoint_and_hold_report`；完整 ROS 流程 | 真实短路径端点、`completed`、保持后样本停止增长 |
| A05 | `test_tracking_run.py::test_both_stall_rules_hold_without_further_sampling` | 合法固定输入分别触发两种现有停滞条件，保持平滑和 `held` |
| A06 | `test_transition_real_runtime.py::test_ros_request_through_moveit_replay_tracking_and_terminal_report` | 终止报告后台写入期间命令继续发布；写入期间同步导出被拒绝 |
| A07 | `test_tracking_run.py`；`test_active_controller_close_preserves_report_with_state_stream_running` | 活跃关闭记为 `interrupted`；已确定原因稳定；未采样时不生成报告 |
| A08 | `test_tracking_run.py`、`test_tracking_report_writer.py` | 真实父路径类型错误、真实权限错误、`saved`/`failed`、已有文件内容保持 |
| A09 | `test_real_obstacle_measurement_and_declared_scope`；`test_tracking_evaluator.py` | 启用与禁用障碍输入、CBF margin 与物理 clearance 区别、声明区间及区间外样本 |
| A10 | `test_transition_real_runtime.py`；`test_transition_executor.py` | 真实规划和回放成功；MoveIt 执行失败；成功码的公开结果表达。真实 MoveIt 执行成功仍未验证 |
| A11 | `test_oscbf_transition_handoff.py`；真实 runtime 与结果类型测试 | 真实成功、未请求、不可达、暂停 executor 产生的响应超时；合法拒绝数据仅验证公开结果表达 |
| A12 | `test_transition_real_runtime.py` | 停止状态流、缺失依赖、真实 IK 碰撞拒绝、FK 定位障碍产生的起点无效、错误规划器、真实执行错误 |
| A13 | `test_real_planning_and_ros_busy_share_execution`、`test_auto_waits_for_active_ros_execution_without_consuming_attempt` | ROS 与本地执行统一拒绝并发；自动重试不因 busy 消耗次数或改变仿真起点 |
| A14 | `test_real_missing_moveit_does_not_consume_auto_attempts`、自动重试相关测试 | 等待依赖、实际失败后重新选择起点、成功停止与次数上限 |
| A15 | `test_transition_planning_server.py`；真实 ROS busy 与成功响应 | 外部成功值、结果码、字段顺序与三位小数时间格式 |
| A16 | `scripts/measure_qos_latency.py`；公开快照测试 | 通过公开执行状态观察保持；实际状态流至命令流时延 |

## 数值与性能

对照脚本 `.scratch/tracking-transition-implementation/compare_control.py` 从 Git 读取基准版本控制器，在相同设备运行真实内核。热身 5 步后采集 400 步，逐步记录 9 个候选关节位置、6 个误差分量、5 个路径状态以及进给和 QP 状态。比较使用既有 `rtol=1e-5`、`atol=1e-7`；400 × 22 个值的最大绝对差为 **0**。

| 测量范围 | 版本 | 样本数 | p50 / ms | p95 / ms | 最大值 / ms |
|---|---|---:|---:|---:|---:|
| 公开 `step_once`，热身后 | `5530ab8` | 400 | 9.400 | 10.465 | 12.076 |
| 公开 `step_once`，热身后 | 当前实现 | 400 | 9.669 | 10.803 | 11.560 |
| 完整控制 callback，热身后 | `5530ab8` | 400 | 9.490 | 10.741 | 12.704 |
| 完整控制 callback，热身后 | 当前实现 | 400 | 9.794 | 10.900 | 12.409 |
| 状态发布至被控对象收到命令 | `5530ab8` | 611 | 13.495 | 14.893 | 15.967 |
| 状态发布至被控对象收到命令 | 当前实现 | 559 | 13.160 | 14.609 | 15.733 |

端到端测量采用生产 QoS，状态深度 20、命令深度 5，热身 2 秒、采样 8 秒。原始结果分别位于 `output/i13-qos-latency/20261001T060417Z-15556/` 和 `output/i13-qos-latency/20261001T064945Z-27346/`，包含独立进程日志、实际消息时间和源码摘要。

同次运行的控制器内部计时包含热身及采样区间：基准 785 步，p50/p95/max 为 8.590/9.859/13.185 ms；当前 755 步，为 8.565/9.900/11.210 ms。两次均无超过既有 20 ms 预算的控制步，QP 失败数均为零。端到端观察范围与内部控制步预算分别记录。

## 终止报告并发观测

完整 ROS 流程保存 6,447 个控制样本，终止原因为 `completed`。样本 SHA-256 为 `da57ece1f4df14e05208eaff588bb0e6e37585b09036293050110dd365f264a4`，与报告摘要一致。

同一报告记录 `task_verdict=fail`、`online_verdict=insufficient_evidence`、`full_path_verified=False` 和评价字段 `completed=False`。这些评价结果按原值保存。

外部每 10 ms 读取公开状态，观测到 `writing` 的 monotonic 区间为 15400.222369–15405.165838 秒，约 4.943 秒。该区间收到 226 条保持命令，命令到达间隔 p50/p95/max 为 18.183/48.874/102.886 ms。后台保存期间命令持续发布；此观测同时显示调度间隔存在波动。内部控制步的 20 ms 预算不涵盖这一保持阶段命令到达间隔。

`report-observations.json`、`command-times.json`、终止摘要与原始样本位于 `.scratch/tracking-transition-implementation/suite-tmp/runtime-final/test_ros_request_through_movei0/`。观测时间具有轮询分辨率；调用结构检查确认控制 callback 只提交后台工作，关闭阶段才等待 writer。测试还核对了保存完成后样本数稳定，以及写入期间显式同步导出被拒绝。

## 运行命令

工作目录为仓库根目录。以下环境设置把测试日志与临时数据保存在项目目录中：

```bash
source install/setup.bash
export TMPDIR="$PWD/.scratch/tracking-transition-implementation/suite-tmp"
export ROS_LOG_DIR="$PWD/.scratch/tracking-transition-implementation/ros-logs"
export ROS_LOCALHOST_ONLY=1
```

构建验证使用当前 ROS 的动态库与 CMake 配置：

```bash
colcon build --symlink-install --base-paths . src/aeb_rrtstar_ompl \
  --packages-select aeb_rrtstar_ompl robot_safecontrol_moveit --cmake-clean-cache
```

数值与性能命令：

```bash
python3 .scratch/tracking-transition-implementation/compare_control.py --baseline
python3 .scratch/tracking-transition-implementation/compare_control.py
python3 scripts/measure_qos_latency.py --state-depths 20 --command-depths 5 \
  --state-publish-depth 20 --production-qos --warmup-seconds 2 \
  --run-seconds 8 --min-samples 150 --base-domain 212
```

完整测试命令：

```bash
export PYTEST_ADDOPTS='-k "not test_missing_record_refuses_startup"'
unset HARDWARE_TEST_VCAN
bash run_all_tests.sh
bash scripts/agent_check.sh
```

`test_missing_record_refuses_startup` 使用固定 `/tmp` 路径，按本次目录限制排除；其他测试使用上述 `TMPDIR`。Linux vcan 集成需要单独配置，其状态以测试输出为准。

几何资产检查使用项目内环境中的固定依赖：

```bash
python3 -m venv --without-pip --system-site-packages \
  .scratch/tracking-transition-implementation/geometry-env
python3 -m pip --python .scratch/tracking-transition-implementation/geometry-env/bin/python \
  install -r portable_oscbf/requirements-geometry.txt
.scratch/tracking-transition-implementation/geometry-env/bin/python -m pytest \
  portable_oscbf/tests/test_collision_geometry_artifact.py -q
```

ROS 与内核分段验证命令：

```bash
python3 -m pytest tests/test_transition_real_runtime.py -q \
  -W error::pytest.PytestUnhandledThreadExceptionWarning
python3 -m pytest tests/test_transition_real_runtime.py -q -k active_controller_close \
  -W error::pytest.PytestUnhandledThreadExceptionWarning
python3 -m pytest portable_oscbf/tests/test_perception_interface.py -q -s
python3 -m pytest portable_oscbf/tests/test_perception_pipeline.py \
  portable_oscbf/tests/test_prepare_scene.py \
  portable_oscbf/tests/test_qp_solver_health.py \
  portable_oscbf/tests/test_safety_snapshot.py \
  portable_oscbf/tests/test_segment_proximity.py \
  portable_oscbf/tests/test_tool_axis_task.py \
  portable_oscbf/tests/test_trajectory_loading.py -q
```

## 实际结果

| 检查 | 结果 | 原始记录 |
|---|---|---|
| colcon 构建 | 2 个 package 完成 | `build-clean.log` |
| 主包用例覆盖 | 合并全量执行与相关复验：560 通过、1 跳过、1 排除 | `all-tests.log`、`runtime-final.log`、`close-final.log` |
| 真实 ROS 集成 | 13 项组合执行通过；活动关闭场景单独通过，合计 14 项 | `runtime-final.log`、`close-final.log` |
| 内核用例覆盖 | 合并分段验证：474 通过、34 跳过，共 508 项 | `all-tests.log`、`kernel-collection.log` 及下列分段记录 |
| 几何资产 | 10 通过 | `geometry-final.log` |
| 感知 interface | 3 通过 | `perception-final.log` |
| 后续 7 个内核测试文件 | 149 通过、19 跳过 | `kernel-tail.log` |
| AEB 插件 ctest | 4 通过 | `all-tests.log` |
| 快速检查 | 编译、import、shell、diff 检查及 69 项测试通过 | `agent-check-final.log` |

日志均位于 `.scratch/tracking-transition-implementation/`。覆盖数量按用例去重汇集。完整测试入口的单次运行结果为未通过。

控制内核的单进程全量运行在 `test_perception_interface.py` 的 SDF 内核初始化期间异常终止，退出码 134，位置为 JAX `compilation_cache.get_executable_and_time`。该文件在独立进程中 3 项全部通过；随后 7 个测试文件也已执行。聚合运行异常的原因尚未确定，完整入口一次执行通过仍属于验证缺口。

## 验证边界

当前环境没有供 `moveit_execute` 使用的 `FollowJointTrajectory` 仿真执行服务。真实执行请求到达 MoveIt 后返回执行失败，保留既有错误语义；`TRANSITION_EXECUTED` 仅完成结果类型与格式验证。此项真实成功运输验证没有计为通过。

部分隔离 MoveIt 2.5.10 进程在测试结束的 SIGINT 收尾阶段发生 segmentation fault，记录位于 `suite-tmp/runtime-final/**/move_group.log`。测试相位的 ROS 响应已完成；清理后本次子进程均已退出。此环境问题与用例通过结果分别记录。

本次运行验证控制软件与本地仿真行为。报告中的模型候选结果、命令观察与仿真反馈保持各自测量含义。
