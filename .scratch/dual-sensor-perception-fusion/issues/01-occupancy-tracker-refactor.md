# 01 — OccupancyTracker：时间戳驱动三层占据模型

**What to build:** 将占据跟踪器从帧计数泄漏计数器重构为时间戳驱动的三层模型
（instant / unconfirmed / static）。新 `OccupancyTracker` 接收 `(points_world, stamp_s)`，
返回三层点云：instant（当前帧全部）、unconfirmed（占据但未达静态阈值）、static
（持续 ≥ static_confirm_s 的体素中心）。内部用 `prev_occupied` 连续性检测确保
占据中断后重新计时。旧 `StaticOccupancyTracker` 名称保留为兼容别名。

**Blocked by:** None — 可立即开始。

**Status:** ready-for-agent

- [ ] `OccupancyTracker.__init__` 接收 `spec, occupancy_timeout_s, static_confirm_s`
- [ ] `update(points_world, stamp_s)` 返回 `(static_points, unconfirmed_points, instant_points)`
- [ ] `newly_occupied = occupied & ~prev_occupied` 时才记录 `first_seen`
- [ ] 占据中断 → `first_seen` 重置为 `inf`，下次重新计时
- [ ] `(stamp_s - last_seen) > occupancy_timeout_s` → 清除 first_seen/last_seen
- [ ] `first_seen` 初始为 `inf`（不是 `-inf`）
- [ ] `last_seen` 初始为 `-inf`
- [ ] 旧 `StaticOccupancyTracker` 名称保留为兼容别名
- [ ] 单元测试：连续占据升格为 static
- [ ] 单元测试：占据中断后重新计时
- [ ] 单元测试：超时清理
- [ ] 单元测试：prev_occupied 连续性
