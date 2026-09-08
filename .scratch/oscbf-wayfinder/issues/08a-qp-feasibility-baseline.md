# 08A — CBF/QP 可行性实证测量（障碍 + 自碰撞工况）— prototype

**What to build:** M12 的 qp=0 失败只覆盖无障碍基准。需要把「有障碍」工况的
QP 可行性量化出来：用现有评价器/基准 run 若干障碍场景（8 槽球障碍若干档、
聚合模式、有/无 ESDF、奇异性附近），统计：

- QP 失败次数与失败时 min_dist/min_esdf/h_vals（哪个约束失效）；
- 主动约束分布（关节限位/自碰撞/障碍/速率）与 alpha 裕度；
- 失败→零速（`apply_qp_health_gate`）触发频率与恢复时间。

产出表格 + 触发复现脚本，为「d_safe/margin/alpha 是否合理、失败策略」提供数据。
**「裕度/失败策略」由 08B（grilling 决策票）拍板**，本票只出数据。

**Blocked by:** None — 复用现有评价器与 frozen-QP 审计接口。

**Type:** prototype（必须靠实验回答；先跑原型量化数据，不先改产品实现）

**Queue:** wayfinder-core — 只测数据，决策在 08B。
**Tracker:** #6 (https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/6)

**Status:** ready-for-agent

- [ ] 写障碍场景复现脚本（可复用 tests/ 既有夹具）
- [ ] 记录每场景 QP 失败率、min_dist、主动约束、gate 触发表
- [ ] 输出对比文档（.scratch/ 或 output/）
- [ ] 标记「需要用户拍板的裕度/失败策略」决策点（移交 08B）
