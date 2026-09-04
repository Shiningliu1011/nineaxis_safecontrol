# 05A — 感知世界坐标系审计：base_link 与「固定环境系」冲突（research）

**What to build:** 现状：控制器与感知的 `world_frame` 均为 `base_link`，而
`docs/specs/dual_sensor_perception_fusion_spec.md` 要求固定环境坐标系。机器人在
执行过渡/跟踪时基座不动的假设下二者等价；一旦机器人基座移动（或后续任务含
移动），占据点云/障碍槽会跟着世界漂移。本票审计并产出影响面清单：

- 当前 `perception_bridge` → `/perception/tracks` → controller 各环节中
  `world_frame` 是被消费还是仅存在 YAML/备注里；
- 外参占位（LiDAR/depth extrinsics YAML placeholder）在代码里被谁消费、
  标定失败时的默认行为；
- 若要切到固定环境系，需要改哪几个接口（时间戳/变换随附、坐标系参数）；
- 给出「继续用 base_link」与「固定环境系」两种方案的代码影响面对比表。

**Blocked by:** None — 纯审计/文档产出。「选哪个坐标系」由 **05B（grilling 决策票）** 拍板。

**Type:** research（AFK）

**Queue:** wayfinder-core — 审计 AFK 可做；选系决策在 05B。
**Tracker:** #4 (https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/4)

**Status:** ready-for-agent

- [ ] 追踪 world_frame 从 spec → yaml → 代码的完整链路（标注哪些是死配置）
- [ ] 追踪外参占位在代码中的消费点与失败路径
- [ ] 产出影响面清单（需要改动的文件/接口/测试）
- [ ] 两种坐标系方案对比表写入报告
