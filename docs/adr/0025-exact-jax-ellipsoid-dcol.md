---
status: accepted
date: 2026-09-21
---

# 自碰撞采用原生 JAX 精确 ellipsoid DCOL

self collision query 直接使用两个 ellipsoid scaling functions 求解最小线性
`alpha_star`，并返回关节梯度、primal/dual residual、迭代次数与明确 solver status。
solver 具有固定最大迭代容量并支持 JAX JIT；未收敛、残差超限或出现非有限值时，查询
状态无效，不能进入规划或命令准入。

## Considered Options

仓库现有 `vendor/dpax/dpax/ellipsoids.py` 把 ellipsoid 转换成 42 面 polytope 后调用 LP。
该路径的 scale 与梯度对应近似多面体，也不属于双方强凸 scaling function 的连续可微
证明范围，因此不作为生产 self collision query。

## Consequences

需要建立原生 JAX ellipsoid solver，并以官方 Julia DCOL 作为离线数值参考，覆盖分离、
接触、相交、尺度差异和接近退化的姿态。solver health 是查询结果的一部分，不能只返回
`alpha_star`。
