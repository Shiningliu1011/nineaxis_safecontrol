# 跟踪执行与过渡结果

`TrackingRun` 位于 `src/robot_safecontrol_moveit/tracking_run.py`，负责一次跟踪的开始、轨迹推进、停滞判定、保持目标、命令发布、评价结束和报告提交。它组合真实 `JaxControlLoop`、`TrackingEvaluator` 与 `TrackingReportWriter`，没有 ROS import。`OscbfController` 负责生产配置、消息转换、调度、日志展示及 ROS 资源。

## 公开入口

| 入口 | 行为 |
|---|---|
| `TrackingRun.start()` | 首次从等待进入跟踪返回 `TRACKING_STARTED`；重复请求返回 `ALREADY_TRACKING` |
| `receive_state(q)` | 按配置中的关节顺序接收有限位置；复制输入，等待下一周期 |
| `tick(publish, obs_kwargs=...)` | 完成一个周期，向 `publish` 交付最终位置命令；生产 adapter 负责添加时间戳、关节名并发布 |
| `step_once(q, obs_kwargs=...)` | 直接计算控制步并推进路径状态；评价样本与命令发布由周期入口管理 |
| `progress_snapshot()` | 保留进度、误差、时延与报告状态，增加 `execution_state` 和 `termination` |
| `finish(reason)` / `close()` | 首次终止确定原因并提交报告；重复结束保持原有原因；关闭等待后台报告完成 |
| `tracking_report()` / `write_tracking_report(path)` | 读取评价结果副本，或在控制 callback 外同步导出；后台写入期间拒绝同步导出 |

状态为 `waiting`、`tracking`、`holding`、`closed`。报告状态独立为 `idle`、`writing`、`saved`、`failed`。调用方串行调用状态变更方法，并在停止调度后关闭资源。

节点关闭期间停止接收状态，尚未处理的开始请求返回 `TRACKING_CLOSED`，保持已确定的终止原因。

控制器保留 `step_once`、`progress_snapshot`、`tracking_report`、`write_tracking_report` 和 `write_perf_report`；`execution_state` 是供诊断工具读取的公开属性。调用方在 ROS executor 完成调度收尾后调用 `destroy_node()`。控制器关闭时取消 timer，等待正在执行的控制周期完成，结束评价并等待 writer，随后销毁 ROS 资源。

## 周期与测量

等待开始或没有有效状态时，控制周期返回。正常周期消耗一个状态，完成内核计算、评价采样、停滞与端点判定、命令有限性及关节范围检查。QP 失败事件、计数和现有命令处理保持原有规则。

两种停滞规则按顺序执行：横向误差超过 5 mm 且进给不超过 1 mm/s 持续超过一秒；或者非终点阶段经过完整五秒，参考源时间增长不足 0.01 s。第二种规则与进给大小无关。

参考到达终点后继续闭环收敛。关节反馈对应的末端到终点距离不超过 0.5 mm、当前 `tool_axis_5d` 模式的工具轴夹角不超过 0.5°，持续满足 0.5 s，且当前反馈位置在末段的投影到达完整路径终点时，报告 `END_OF_TRACKING`。弧长比较使用评价器规定的浮点舍入量；终止条件与区间覆盖检查使用相同的终点范围。终点截面之前加入 0.1 mm/s 的切向驱动分量，使运动实际覆盖最后一段；该分量经过完整 QP 和执行器运动限制。全过程精度与安全证据仍分别评价。

保持目标采用当前关节位置并限幅。正常周期消耗一份状态，直接发布经过 QP 检查的位置命令；保持时直接发布保持目标，内核步数与评价样本停止增长。状态流使用 `KEEP_LAST`、depth 1、`BEST_EFFORT`，使计算周期超过发布周期时仍使用最新反馈。

生产配置的参考领先距离为 2 mm。路径投影的局部搜索距离由 `reference_lead_m + max_projection_speed_m_s * dt_path` 决定，覆盖该弧长范围内全部线段。路径自交处仍受局部弧长范围与单向进度约束。位置反馈修正完整三维误差，包含切向的超前、滞后和横向分量；进给上限计算与名义速度计算使用相同的反馈规则。反馈经过现有饱和与速度限制，再进入 CBF-QP。末端超过参考点时，前馈随切向误差减小。

报告描述 `kernel_candidate`，位置命令来自模型积分；实际执行效果需要另行核对仿真反馈。未测量的障碍距离表示为 `None`。报告的声明区间、工具轴夹角、阈值和证据身份见[跟踪评价](tracking_evaluation.md)。

## 过渡执行

`TransitionPlanningServer.execute_plan()` 与 ROS `/plan_transition_once`、自动过渡调用同一个 `TransitionExecutor.execute_plan()`。执行器通过锁统一拒绝并发执行，返回 `PLANNING_ALREADY_RUNNING`；结束或异常后释放锁。

结果类型 `TransitionResult` 是冻结的 dataclass：`code`、`trajectory_points`、`planning_time_s`、有序诊断字段 `context`、`handoff_requested` 和 `handoff_code`。`success` 由结果码计算。`AutoPlanLoop` 直接消费结果；`auto_plan_snapshot()` 提供是否启用、是否完成和尝试次数。

自动重试的 timer 通过独立锁防止重复调度。执行开始时计入当前尝试；等待依赖期间出现其他执行时，`PLANNING_ALREADY_RUNNING` 恢复本轮计数，也不重新选择仿真起点。

回放成功使用 `TRANSITION_REPLAYED`。交接成功、不可达、超时或拒绝分别记录在 `handoff_code`，保留回放的成功判定与自动重试规则。未请求交接时 `handoff_requested=False`、`handoff_code=None`。规划用时在回放之前记录。

仿真交接使用新的关节反馈确认过渡终点：各关节与回放目标的差值小于 `1e-6`（J1 为 m，其余为 rad），并持续 0.5 s 后才通知 OSCBF 开始。重复时间戳不计入稳定时间；五秒内未完成收敛则终止本次交接。

ROS adapter 的 `format_transition_result()` 保留 Trigger 的字段顺序和三位小数时间格式，例如：

```text
error_code=TRANSITION_REPLAYED|trajectory_points=10|planning_time=2.500
```

## 验证入口

- `tests/test_tracking_run.py`：真实内核、状态门控、两个停滞条件、端点、障碍测量、评价区间、文件系统错误与报告身份。
- `tests/test_oscbf_controller_smoke.py`：生产配置与直接配置内核的数值对照、真实 ROS 开始与发布、实际被控对象时延。
- `tests/test_transition_real_runtime.py`：独立 ROS domain 中启动真实 MoveIt/AEB、生产过渡节点与 `OscbfPlant`，从 ROS 请求运行到终止报告。
- `tests/test_transition_executor.py` 与 `tests/test_transition_planning_server.py`：不可变结果、合法协议数据和 ROS 文本兼容性。
- `scripts/measure_qos_latency.py`：独立进程记录真实状态与命令消息，通过 `execution_state` 识别保持阶段；计时说明见[QoS 时延验证](validation/qos-latency-2026-09-23.md)。

运行测试时加载 `install/setup.bash`，把 `TMPDIR`、`ROS_LOG_DIR` 和 pytest `--basetemp` 指向项目 `.scratch/` 下的独立目录。真实 MoveIt 测试需要已构建的 AEB 插件；测试仅停止自身启动的进程。控制模式保持仿真，硬件准入限制保持现有规则。

规格验收、数值对照和实际运行结果见[2026-10-01 验证记录](validation/tracking-transition-2026-10-01.md)。

当前生产参数的独立仿真、全过程精度和图片说明见[跟踪平顺性与图片](tracking_smoothness.md)。完整记录保存在 `output/oscillation_fix/precision/`，包含起点和终点覆盖、全部关节反馈、模型报告及运动限值检查。
