---
status: accepted
date: 2026-09-21
---

# 机器人使用固定容量组合 ellipsoid 包络

碰撞安全模块为每个机器人 link 预留固定容量的 ellipsoid 槽位，通过 `active_mask`
启用其中一个或多个。简单 link 可以启用较少槽位，不规则 link 可以启用更多槽位；
具体容量由完整 collision mesh 覆盖验证与实时计算测试共同确定。机器人本体、滑台和当前
末端工具使用同一表示，几何变化必须生成新的 `geometry_hash`。

## Considered Options

每个 link 只使用一个 ellipsoid 可以减少查询数量，但不规则 link 会产生较大的空白
包络并减少可通行区域。数量随运行状态改变会破坏 JAX 输入形状稳定性。因此选择固定容量
与有效槽位组合。

## Consequences

离线几何生成必须证明启用 ellipsoid 的并集覆盖完整 collision mesh，并记录覆盖误差、
槽位顺序与几何身份。规划、轨迹执行和 OSCBF 必须使用同一槽位数据；工具变化后，旧几何
对应的规划结果不能继续使用。生成与证明方法见
[ADR 0032](0032-certified-automatic-ellipsoid-generation.md)。
