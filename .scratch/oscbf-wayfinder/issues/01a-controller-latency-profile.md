# 01A — 控制器单步延迟 p95 19.4ms 回归剖面与成本分布（prototype）

**What to build:** 复现并分解 `path_tracking_step` 的延迟分布，产出「每阶段耗时表」
（JIT 编译 / 输入准备 / 内核计算 / qpax 求解 / 诊断收集 / Python 发布）。当前证据：
M12 旧配置（kp_pos=60/lead=1e-5）p95=6.177ms；新配置（dt_path=0.01 / tool_axis_5d
/ leak）实测 p95=19.367ms——真实性能回归或配置变化导致，需先分阶段测量确认，
而不是直接调预算。

本票**只做测量与候选清单**：回答「慢在哪里」+ 产出按收益排序的优化候选清单；
**不做预算决策、不改产品代码**。「10ms 预算是否保留 / 走哪条优化路线」由
**01B（grilling 决策票）** 拍板；优化落地另拆 impl ticket。

**Blocked by:** None — 可直接测量。

**Type:** prototype（先跑原型量化，不先改产品实现）

**Queue:** wayfinder-core — 只测「慢在哪」，决策在 01B。
**Tracker:** #3 (https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/3)

**Status:** 阶段性优化已保留；研究暂停（2026-09-07），记录见 ../01a-phase-times.md。

- [x] 用 `freeze_qp_problem` + `solve_frozen_qp_problem`（rate-slack 模式）单独测 qpax 求解耗时
  （注: 生产为弹性 QP 模式，`_qp_core` 为 None；弹性求解含在 `_path_solve`=0.37ms 中）
- [x] 记录 `path_tracking_step` 的 JIT 编译耗时与稳态单步耗时（区分预热的 8.6–28.75s）
  （init_cbf 共 24.8s; 预热日志 6.4s; 稳态 p50=11.2ms）
- [x] 对比 `collect_cbf_diagnostics=True/False` 两条路径的耗时差
  （path 内核恒收集；数组同步时间包含等待计算，不能当成纯主机转换成本）
- [x] 产出每阶段耗时表（写入 output/ 或 .scratch/）（.scratch/oscbf-wayfinder/01a-phase-times.md）
- [x] 整理有收益的优化与验证边界（见 01a-phase-times.md）

**结论:** 87% 成本在 `_path_constraint_terms`（cbfpy `G_qp` = 9×JVP 穿过 43 行 h）。
奇异性 SVD 替换没有运行时收益，已恢复。碰撞梯度、2×2 求解、JIT 合并和
默认输入缓存已有阶段性收益；完整系统 miss≤1% 尚未完成验收。

---

## 后续 impl（另行拆票，不在本票范围）

等 01B 结论定稿后，把优化清单落成独立 impl ticket（附每阶段耗时表与收益排序）：

- [ ] 关闭或裁剪 `collect_cbf_diagnostics` 的 fast 变体
- [ ] 减少障碍行数 / 批量出图 / 其他按耗时表排序的候选
- [ ] 若优化后仍 >10ms：降频或换求解器并按 01B 决策落地
