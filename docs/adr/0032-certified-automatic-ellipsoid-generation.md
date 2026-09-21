---
status: accepted
date: 2026-09-21
---

# 组合 ellipsoid 采用离线自动生成与四面体覆盖证明

离线工具读取经过检查的 closed collision mesh，对 mesh volume 进行四面体化，将四面体
分组并拟合固定容量 outer ellipsoids。每个四面体必须有一个 ellipsoid 同时包含其全部
顶点；ellipsoid 与四面体均为凸集合，因此该条件证明完整四面体受到覆盖。全部四面体通过
后，ellipsoid 并集才获得 link collision volume 的覆盖证明。

生成器在满足覆盖证明的候选中优化包络体积和 ellipsoid 数量，并运行 JAX/DCOL 时间测试
确定固定槽位容量。输出 geometry artifact，包括 mesh hash、槽位参数、覆盖证明、最大
包络范围、性能记录和 `geometry_hash`。人工调整只能作为生成器输入，输出仍需执行完整
证明与性能验证。

## Considered Options

曾考虑人工配置每个 ellipsoid 后运行表面采样检查。采样无法证明未采样区域受到覆盖，且
机器人 mesh 或工具变化后需要重复人工处理，因此采用体积四面体覆盖证明。

## Consequences

非 closed、拓扑无效或单位不明的 mesh 不能生成可准入 artifact。机器人本体、滑台和当前
末端工具都必须具有有效 artifact；mesh hash 变化会使旧 `geometry_hash`、规划结果与区间
证明失效。
