# 01B — 选定控制率/延迟预算与实现策略（grilling）

**What to build:** HITL 决策票（只能由用户拍板，AFK agent 无法回答），由 01A 的
每阶段耗时表与优化候选清单驱动，回答：

- **10ms 预算是否保留？**若保留，走哪条优化路线（诊断裁剪 / 减少障碍行数 /
  换求解器 / C++ 内核 / 降频）——按 01A 的收益排序选择；
- **目标控制率：**保留 100Hz 还是降频（如 50Hz）？19.4ms 单步在 100Hz 下会丢周期，
  能接受什么折衷；
- **JIT 预热：**8.6s–28.75s 是否接受在 launch 前显式完成（热启动脚本）？
- **RT 边界：**Python + GIL、无 RT 调度下的实时性承诺（真机 vs 仿真）。

产出：预算决策 + 实现策略选择 + 后续优化 impl ticket 范围（**02 的 perf 断言最终口径
由本票确定**）。

**Blocked by:** 01A（先有耗时分布，再谈预算）。

**Type:** grilling（HITL）

**Queue:** wayfinder-core — 决策票：性能预算与实现策略（MAP 讨论项 G）。
**Tracker:** #17 (https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/17)

**Status:** ready-for-human

- [ ] 选定延迟预算与目标控制率（含被放弃项与理由）
- [ ] 选定实现策略/优化路线（对应收益排序），明确需动的代码面
- [ ] 明确 JIT 预热处理方式（热启动脚本 or 接受现状）
- [ ] 产出后续 impl ticket 范围
