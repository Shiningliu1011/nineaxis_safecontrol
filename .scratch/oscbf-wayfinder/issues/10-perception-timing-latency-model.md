# 10 — 感知时间同步与延迟模型（点云融合 → CBF 输入）

**What to build:** 感知链路已有「时间戳配对 + voxel 降采样」的点云级融合,但
控制器侧没有系统化的感知延迟模型:障碍槽数据进入 CBF 时,其数据年龄
（age = now - sensor_stamp）是否被记录/用于降级?现无。本票:

- 梳理 perception_bridge → /perception/tracks → controller 的时间戳流转
  (时钟域,ROS stamp 来源,配对窗口);
- 对比 CBF 常用做法(膨胀 d_safe by v_rel * age、禁用过期槽、年龄上限);
- 实现并导出「延迟/年龄」诊断(作为 ticket 08 与 01 的输入);
- 产出参数建议(最大允许 age、风险膨胀系数)。

**Blocked by:** None — 目前无消费方,属于补齐模型。

**Queue:** wayfinder-core
**Tracker:** #7 (https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/7)

**Status:** ready-for-agent

- [ ] 时间戳流转图 + 配对窗口参数现状
- [ ] 延迟/年龄诊断字段(控制器进度快照增加 max_obstacle_age_ms)
- [ ] 降级策略实现(至少:超过年龄上限的障碍不参与 CBF)
- [ ] 参数建议文档(供讨论成本/安全性权衡)
