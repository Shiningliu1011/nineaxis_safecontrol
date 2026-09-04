# 06 — 真机执行端节点

**What to build:** 真机闭环的心脏：一条命令流（100Hz 位置命令）经安全网关灌入 CAN 帧，电机反馈反向回到状态流——与仿真执行端完全同构，一个模式开关在记录（shadow）与发送（live）间切换。修好它，真机就"听得到命令、报得出状态"。

**Blocked by:** 02 — DrEmpower 帧编解码器；03 — 安全网关（位置契约）；04 — 单位换算与零位偏移；05 — SocketCAN 后端。

**Status:** done — 2026-08-22（13 项全绿；已提交 6269a3e；launch 集成已完成）

- [x] 已实现：订阅 /oscbf_command → CommandSafetyGate → UnitConverter → encode_position → CANBusBackend.send。反馈：recv → decode_feedback → encoder_to_joint → /mujoco_joint_states。
- [x] shadow 模式：ShadowCommandRecorder.record()，无 CAN 调用；live 模式：SocketCANBus.send_position()。
- [x] estop_active/fault_code/feedback_ok 注入测试覆盖；latched_stop_reason 可观测；acknowledge_stop() 清锁存。
- [x] J1TransmissionMissingError → 跳过发送（fail-closed）；test_j1_uncalibrated_blocks_command 验证。
- [x] latched_stop_reason、send_count、CANBusMetrics（loss_rate/latency_p50/p95/p99/stale_count）通过 bus.metrics 可观测。
- [x] 13 项话题级测试全绿（FakeCANBackendForBridge 注入）；shadow 模式记录验证。

