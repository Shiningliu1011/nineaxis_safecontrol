# [I21] 实现原生 JAX ellipsoid–ellipsoid DCOL

## Goal

通过 CollisionSafety.query 实现 ellipsoid DCOL、关节梯度、pair clearance 和求解健康，并完成解析及独立数值验收

## Requirements

- 来源：[实现原生 JAX ellipsoid–ellipsoid DCOL](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/107)、CollisionSafety 规格决定 7、8、ADR 0025、0026。
- 通过现有 `query` 返回线性 `proximity_scale`、九轴梯度、独立 pair barrier、primal/dual residual、iteration 和 solver health。
- 毫米 clearance 来自不可变 parameter artifact，margin 按两个最短半轴计算。
- JAX float64、固定容量、明确 mask；非法几何和配置立即拒绝，未收敛和不可用梯度保持非 `OK`。
- 保留生产控制路径与真实设备安全边界。

## Acceptance Criteria

- [x] 解析球体和轴对齐 ellipsoid 的分离、接触、相交与尺寸差异通过。
- [x] 一般姿态与接近退化几何通过独立高精度参考，并核对官方 Julia DCOL 离线参考。
- [x] 九轴梯度通过独立有限差分，所有有效 primitive 保留独立行。
- [x] residual、最大 iteration、非有限值、同心几何、mask、身份、deadline 和未实现环境能力均有拒绝证据。
- [x] 生产几何资产能够接入查询，相关回归通过，文档与票尾记录具有实际命令和证据。

## Notes

- 本票属于执行地图；用户授权提交和推送本票改动、维护议题及地图、归档本任务，分支仅保留 `main`。
