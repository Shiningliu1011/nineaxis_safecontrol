# 08B — 选定不可行/裕度/降级策略（grilling）

**What to build:** HITL 决策票（只能由用户拍板，AFK agent 无法回答），由 08A 的
可行性数据 + 12 的碰撞几何核对结果驱动，回答：

- **QP 失败行为：零速（现状，先停）还是降级（保性能）？**恢复策略（先停再恢复 vs 重规划）；
- **裕度取值：**d_safe / margin / alpha 用 12 核对后的理论值还是维持经验值？
  是否分级（不同约束不同裕度）；
- **汇编策略：**弹性 QP（默认）还是 hard rate-slack？允许的瞬时违反量。

**Blocked by:** 08A（必须有障碍工况可行性数据）+ 12（几何核对决定裕度取值依据）。

**Type:** grilling（HITL）

**Queue:** wayfinder-core — 决策票：失败/裕度策略（MAP 讨论项 H）。
**Tracker:** #18 (https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/18)

**Status:** ready-for-human

- [ ] 选定失败策略（零速 vs 降级 + 恢复路径）
- [ ] 选定裕度取值策略（理论值 vs 经验值 + 分级方案）
- [ ] 选定/确认 QP 汇编形式与 slack 预算
- [ ] 明确后续 impl 票面（如需改约束/汇编实现）
