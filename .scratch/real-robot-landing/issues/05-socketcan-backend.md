# 05 — SocketCAN 后端

**What to build:** CAN 收发不再是未验证的黑箱：后端接口参数化（虚拟/真实总线一名切换）、轮询、丢帧统计、时间戳、重连；测试用可注入替身，不接硬件也能验证收发语义。

**Blocked by:** 02 — DrEmpower 帧编解码器（帧结构由其定义）。

**Status:** done — 2026-08-22（18 项全绿；CANable 到货后替换 FakeCANBackend）

- [x] CANBusConfig 接口参数化，poll_interval_s 可配。
- [x] CANBusMetrics 丢帧/延迟百分位/离线/重连计数；FakeCANBackend 注入测试覆盖全部语义。
- [x] CANNodeState.online/axis_error 可区分；stale_count 记录离线转换。
- [x] 零 ROS import；18 项测试全绿。

