# 02 — 主包测试套件修复（settle harness / perf 孤立性 / e2e steps 断言）

**What to build:** 修复当前 `tests/` 的 4 个必现失败（241 passed / 4 failed）：

1. `test_transition_planning_server.py::TestPlantSettleBeforeHandoff`（2 个）：
   `_SettleServer` 假类缺少 `_wait_for_js_snapshot`，该方法是源码在持久订阅重构后
   （`_wait_for_plant_settle` 内自建订阅，见 transition_planning_server.py:301-337）
   引入的，harness 未同步。要么给假类补上方法，要么让 `_wait_for_plant_settle`
   依赖可注入接口。
2. `test_perf_report_p95_within_budget`（test_oscbf_controller_smoke.py:257）：
   依赖同模块前序测试（test_start_signal_unlocks_safe_state）泄漏的 tracking 已启动
   状态；单独运行该测试时 burst 期间 0 步，报告 p95=nan → 断言 "missing p95"。
   应显式调用 start_tracking 服务（或直接驱动 step_once）积累样本，不依赖执行顺序。
   注意：模块内顺序运行时该测试的真实失败是 p95=19.367ms>10ms，即 ticket 01 的
   性能问题，不允许通过跳过/放宽绕过。
3. `test_zero_transition_then_tracking`（test_oscbf_full_flow_e2e.py:193）：
   交接后 15s 内仅 10 步（`snapshot["steps"]=10 < 200`）。需判断是控制器 tick 门控
   （植物状态陈旧/冻结检测）导致还是时序假设问题，修复测试或修 bug（以证据为准）。

**Blocked by:** 01B（perf 断言最终口径随 01B 的预算决策而定；其余修复无阻塞，可先行）。

**Queue:** implementation-backlog（已移出 Wayfinder）
**Tracker:** #9 (https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/9)

**Status:** ready-for-agent

- [ ] 修复 settle harness（2 个测试转绿）
- [ ] perf 测试不依赖执行顺序（单独运行即能通过「状态启动」前置）
- [ ] e2e 失败根因定位：门控计时 vs 测试假设，按证据修复
- [ ] 全量 pytest tests/ 恢复 0 failed（perf 预算项在 01B 拍板前允许标记 xfail 并注明原因）
