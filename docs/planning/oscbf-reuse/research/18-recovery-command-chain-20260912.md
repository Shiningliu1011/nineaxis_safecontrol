# [18] 续研：QP 失败到最终命令的锁存与恢复链

日期：2026-09-12。该记录是 GitHub #18「[08B] 选定不可行/裕度/降级策略」的后续事实核查，不新增政策决议，也不替代已有的八项用户确认。

## 决策问题

已接受的 Q8 要求保持一阶低通，并把其时间常数纳入停止预算；Q1/Q8 还要求准入失败进入冻结/锁存路径，恢复只能在健康条件满足后由人工触发。本轮核查的问题是：当前代码是否已经把

```
QP / admission → q_next → low-pass → command topic → hardware safety gate → feedback
```

连接成这个契约，以及在不接硬件的条件下还能证明什么。

## 前置与证据范围

- 前置决议：#18 handoff 中 Q1–Q8 已逐条确认；本轮不重复征询。
- 代码范围：`portable_oscbf/work/jax_kernel_factory.py`、`portable_oscbf/work/jax_barrier_terms.py`、`src/robot_safecontrol_moveit/oscbf_controller.py`、`src/robot_safecontrol_moveit/hardware_contract.py`、`src/robot_safecontrol_moveit/hardware_bridge.py`。
- 验证范围：纯代码审计、低通离散公式的独立计算、现有安全契约/控制器 smoke 测试；未运行真实驱动、CAN、反馈闭环或物理停止。

## 事实结果

### 1. 内核的失败回退只保证“这一 tick 不积分候选速度”

弹性路径在 `jax_kernel_factory.py:341-353` 中只用有限性和 solver 收敛构造 `qp_ok`；`delta_slack`/`primal_residual` 尚未进入准入门。`apply_qp_health_gate()` 在 `jax_barrier_terms.py:111-115` 中对 `qp_ok=False` 使用零速度，再对 `q + u·dt` 做 clip。因此内核返回的 `q_next` 是当前 `q`（若当前位姿在限位内），但这不是锁存状态，也不是最终发布命令的确认。

### 2. ROS 控制器没有把 QP 失败转换为 `_hold_q`

`oscbf_controller.py:675-682` 的失败分支只增加 `_qp_fail_count` 并记录 warning，随后无条件调用 `_publish_positions(q_next)`。该分支没有设置 `_hold_q`，下一 tick 也没有“上一 tick 失败则保持”的状态判断。

这造成日志与行为契约不完全一致：日志写的是 `holding current state`，但源码实际只保持内核本 tick 的 `q_next`，没有保持控制器整个后续生命周期的命令。

### 3. 低通会让失败 tick 之后仍有非零命令输出

生产 profile 是 100 Hz，`dt=0.01 s`；控制器在 `oscbf_controller.py:684-700` 使用 `tau=0.02 s` 的离散一阶低通，系数为：

```
alpha = dt / (dt + tau) = 1/3
smooth_next = smooth + alpha * (target - smooth)
```

因此若失败 tick 的目标 `target=q` 与当前平滑命令相差 1 个单位，随后目标保持为 `q` 时，平滑命令残留比例为：

| 连续失败 tick | 相对初始平滑命令的残留 |
|---:|---:|
| 1 | 2/3 |
| 2 | 4/9 |
| 3 | 8/27 |
| 6 | 64/729 |

这是离散公式的独立计算，不是执行器停止时间测量。它证明：仅把内核候选速度置零，不能证明最终发布的 `JointState` 立即变为当前反馈位置，也不能证明机器人已停止。

### 4. 已有硬件网关的锁存机制是另一条尚未接线的契约

`hardware_contract.py:148-222` 的 `CommandSafetyGate` 会在反馈/命令/驱动/限幅异常时生成当前位置保持命令，锁存首个原因，并且只有 `acknowledge_stop()` 在健康反馈下才会清除；对应的 37 个契约/集成测试本轮实际通过。

但当前 `hardware_bridge.py:1-7,39-99` 仍处于 containment：`live` 直接拒绝启动，`sim/shadow` 不接受 CAN，shadow 的反馈是 NaN/过期并只记录被拒命令。因此现有网关测试证明了网关自身的锁存语义，不证明 `oscbf_controller` 的 QP/admission 失败已进入网关，更不证明最终命令、驱动响应和真实反馈组成闭环。

### 5. 当前测试没有覆盖这个连接缺口

本轮实际运行：

- `python3 -m pytest -q tests/test_hardware_contract.py tests/test_hardware_stack_integration.py`：**37 passed**；覆盖硬件网关的人工确认和“不自动清除”语义。
- `python3 -m pytest -q tests/test_oscbf_controller_smoke.py -k 'start_signal_unlocks_safe_state or progress_snapshot_reports_tracking_state'`：**2 passed, 12 deselected**；覆盖启动与正常 telemetry，不覆盖 `qp_ok=False`。

控制器 smoke 中现有正常回放还断言 `qp_ok` 全为真（`tests/test_oscbf_controller_smoke.py:300-306`）；没有测试“QP 失败后下一 tick 是否锁存、发布命令是否立即停、人工确认如何恢复”。`tracking_evaluator` 对 `qp_ok=False` 的测试只验证计数/字段，不是命令行为。

## 对 #18 的结论

本轮新增的是一个明确的实现证据缺口，而不是新的政策选择：

1. **已能证明**：内核在失败 tick 不积分候选速度；硬件网关独立具备锁存和人工确认逻辑。
2. **不能证明**：QP/admission 失败会锁存控制器；低通后的最终命令会立即停止；停止后人工恢复与控制器状态重置一致；任何软件命令等价于物理停止。
3. **下游落点**：这应进入 #33「最终命令、后处理与执行保护」的实现/离线故障注入范围；真实的反馈新鲜度、低通后的停止响应和制动距离仍属于 #13/#34 与 #17 的证据范围。无需为同一缺口新建同义决策票。

## 未决不确定性

- admission 采用 `primal_residual` 的按行/按类换算、阈值和长时分布仍按 #18 既有口径保持 provisional。
- 实际执行器是否有内部位置环、命令保持、看门狗和制动器，当前代码/仿真不能给出结论。
- 低通是否应在锁存路径旁路、收紧或继续保持，必须在台架阶跃响应后复验 #17/#18；本记录不替用户选择。

## 研究状态

该研究完成了目前不依赖真实硬件的事实核查；#18 继续保持 OPEN，不更新为 resolution，不修改生产代码、配置、阈值或测试，也未向 GitHub 写入评论。
