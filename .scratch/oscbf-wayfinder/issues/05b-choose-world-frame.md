# 05B — 选定规范世界坐标系与标定策略（grilling）

**What to build:** HITL 决策票（只能由用户拍板，AFK agent 无法回答），由 05A 的
影响面对比表驱动，回答：

- **规范世界坐标系：`base_link`（现状）还是固定环境系（spec 要求）？**——机器人在
  执行过渡/跟踪时基座是否可能移动（决定 `world_frame` 选系）；
- **外参标定策略：**传感器外参目前全为占位（方案/数据无）——需要标定哪些量、
  如何获取（工装测量 / 外部软件 / 手眼标定）、标定失败时的默认与降级行为；
- **迁移范围：**按 05A 清单决定本次只改配置，还是需要拆 impl 票（明确票面）。

**Blocked by:** ~~05A~~ — 已解除：05A 已于 2026-09-04 resolved
（影响面清单 / CBF frame contract / 外参真实状态 / 决策输入见 issues/05a §Resolution）。

**Type:** grilling（HITL）

**Queue:** wayfinder-core — 决策票：世界系与标定策略（MAP 讨论项 D）。
**Tracker:** #15 (https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/15)

**Status:** ready-for-human

- [ ] 选定 world_frame（base_link vs 固定环境系）并给出理由
- [ ] 选定外参标定策略（方案 + 失败降级 + 数据归属）
- [ ] 确认迁移范围（只改配置 / 拆 impl 票并明确票面）
