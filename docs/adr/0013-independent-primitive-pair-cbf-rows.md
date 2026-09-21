---
status: accepted
date: 2026-09-21
---

# 每个有效 ellipsoid pair 保留独立 CBF 行

一个候选 link pair 内，每个由 `active_mask` 启用的 ellipsoid pair 都保留独立 DCOL
barrier、关节梯度、危险对象身份和 CBF 行。规划阶段要求全部有效行满足安全条件，OSCBF
分别处理每个方向；link pair 最小值只用于诊断展示。

## Considered Options

曾考虑对同一 link pair 的 primitive barrier 使用 `softmin`，再向 OSCBF 提供一条
聚合行。对称接触会让相反梯度互相抵消。Q4 原型的对称夹持场景中，聚合梯度为零，聚合
方法保留 `u1 = 0.8`，经过 20 ms 后最小 primitive barrier 变为 `-0.00114`；独立行
把 `u1` 限制为 `0.07431`，下一周期最小 barrier 为 `0.01338`。原型见
[Q4 临时实验](../planning/oscbf-reuse/research/throwaway-q4-independent-cbf-rows-20260921.html)。

## Consequences

QP 行数由候选 link pair 与固定 ellipsoid 槽位共同决定，并通过 pair mask 与
`active_mask` 保持固定形状。实时计算测试需要据此确定每个 link 的槽位容量；任何聚合
统计都不能替代独立约束。
