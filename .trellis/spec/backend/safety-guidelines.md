# 安全边界

## 当前 hardware mode

- `sim`：hardware bridge 不创建真实控制 I/O，不订阅 CAN，不发布硬件状态。
- `shadow`：订阅控制命令，使用未知反馈送入控制安全门，记录被拒绝的命令；不创建 CAN backend，不发布硬件状态。
- `live`：当前在 final launch 和 `HardwareBridge` 构造阶段保持 fail-closed。缺少 backend、配置、标定、真实反馈、freshness 与 watchdog 资格时必须拒绝启动。

`HardwareBridge` 的当前边界明确体现在以下检查中：

```python
if mode == "live":
    raise RuntimeError(
        "live disabled: backend/configuration/calibration and real "
        "feedback freshness/watchdog qualification are incomplete"
    )
if backend is not None:
    raise RuntimeError("sim/shadow do not accept CAN backends")
```

解除 `live` 限制属于独立的真机工作，需要完整的真实反馈、时间基准、watchdog、标定、驱动故障处理和恢复验证。普通控制改动不能顺带开放该模式。

## 反馈与命令

`HardwareState` 只能来自真实编码器或明确的仿真 plant。命令回送、齐零数组、新接收时间戳和人工 acknowledgement 都不能生成健康反馈。

状态至少要携带：位置、速度、测量时间戳、反馈健康状态、急停状态、驱动错误、mode 和 watchdog 状态。位置与速度形状必须一致，关节数量必须符合九轴顺序，所有控制数值必须有限。

`PositionCommand` 携带产生时间、有效期、来源、mode 和停止原因。过期命令、非有限命令、关节数量错误和相邻命令变化越界都进入停止流程。

## 控制安全门

`CommandSafetyGate` 只检查命令，不裁剪 CBF 输出后继续运行。限制违反会生成显式保持命令并锁存首个停止原因：

```python
if np.any(
    np.abs(requested.q - self._last_safe_position)
    > self.config.dq_per_command_limit
):
    return self._stop(state, now, "command_rate_limit")
```

恢复必须由外部监控在状态健康后调用 `acknowledge_stop()`。控制循环不能自动清除锁存原因。

保持当前位置仅在当前位置可信时具有相应含义。若反馈包含非有限值，bridge 必须禁止发送；安全门内部的替代数组不能被当作真实反馈或真机可执行证明。软件位置保持也不能代替物理急停、断电或制动链路。

## 感知与碰撞输入

- 感知标定缺失、无效或不满足准入条件时，启动应被拒绝或对应输入应保持禁用。
- 点云单帧解析错误可以丢弃该帧，不能生成空障碍物来表示环境安全。
- 碰撞距离、梯度、障碍身份和场景版本必须保持同一来源。未测量状态要由独立布尔字段表达，不能把数值 sentinel 解释为测量结果。
- CBF、QP、关节限位和碰撞限制不能为通过测试而放宽。
- containment 或测试通过只证明对应软件边界，不能直接作为真机准入结论。

`docs/domain/safety.md` 中的碰撞架构包含设计目标与已完成内容。实现前要核对当前源码、相关 ADR 和测试，不能仅凭目标描述推断功能已经存在。

## 验证入口

- `tests/test_hardware_contract.py`：watchdog、限制、锁存和人工确认。
- `tests/test_hardware_bridge.py`：mode containment、未知反馈和记录行为。
- `tests/test_launch_structure.py`：final launch 在进程启动前拒绝 `live` 和无效 mode。
- `tests/test_perception_bridge_startup.py`：标定准入。
- `portable_oscbf/tests/test_jax_qp_health.py`：QP 终态 KKT 与限制违反。
- `portable_oscbf/tests/test_obstacle_dcol.py`、`portable_oscbf/tests/test_jax_esdf_cbf.py` 与 `portable_oscbf/tests/test_obb_distance_jvp.py`：碰撞安全约束。

涉及真机的验证还要按 `docs/real_robot_runbook.md` 执行，并保留环境、设备、配置、命令和退出码。
