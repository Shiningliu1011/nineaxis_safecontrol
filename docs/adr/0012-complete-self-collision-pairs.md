---
status: accepted
date: 2026-09-21
---

# 自碰撞采用完整候选集合与显式允许接触列表

碰撞安全模块从完整机器人 link 集合生成自碰撞候选关系，默认检查所有 link pair。
只有经过明确登记和几何验证的允许接触 pair 可以排除；每项排除必须记录 link 身份、
允许原因、验证依据和适用的 `geometry_hash`。规划、轨迹执行和 OSCBF 共用同一份候选
集合与允许接触列表。

## Considered Options

当前控制内核固定检查 14 个 OBB link pair。该列表没有提供全身覆盖证明，也无法随着
组合 ellipsoid、末端工具或机器人几何变化自动更新，因此不继续作为自碰撞范围来源。

## Consequences

工具或机器人几何改变后必须重新生成候选关系并复查允许接触列表。某个 pair 只有在完整
关节范围内得到不可接触证明后，才可以作为经过验证的排除项；缺少证明时继续参与查询。
候选 link pair 内部的约束组织方式见
[ADR 0013](0013-independent-primitive-pair-cbf-rows.md)。
