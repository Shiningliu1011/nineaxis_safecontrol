# [I22] 实现 ellipsoid–point barrier 与独立毫米距离

## 来源与授权

用户要求按 Trellis 与 wayfinder 处理本票，包含本地实现、验证和票据维护，并明确授权仅提交、推送本票改动及归档本地任务，分支只保留 `main`。来源为 [实现票](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/109)、[统一执行地图](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/133) 和 [CollisionSafety 规格](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/98) 决定 9–11。前置接口和几何票均已关闭。

## 要求

- 环境 support 保留独立的 `rho_mm` 与 `required_clearance_mm`。
- 环境 barrier 使用 `point_scale_sq - (1 + (rho_m + required_clearance_m) / a_min)^2`，每个有效 pair 保留九轴梯度和身份。
- 独立毫米距离计算 ellipsoid 与 support sphere 的有符号欧氏距离，并返回要求间距、剩余余量、最近 link/slot/support、双方 witness、track 状态与 scene identity。
- 距离模式不能产生有效命令 barrier 或状态准入 payload。
- 所有结果保持固定 shape、float64、identity、deadline 和无效 mask 规则。
- 保持现有生产接线与硬件边界，活动集合证明、动态误差模型、连续区间证明和生产切换由对应票完成。

## 验收条件

- 通过公开 `prepare_scene` / `query` 验证解析球体、轴对齐及旋转 ellipsoid 的分离、接触、相交、内部点、中心和重复最短半轴。
- 使用独立 80 位数值参考核对距离、最近点与九轴 barrier 梯度。
- 检查多 support、多 slot、最近 pair、track 状态、scene identity、固定容量与 inactive mask。
- 距离模式在有效、失败、超时情况下均无命令准入 mask；无效 scene、非有限计算、迭代不足和行容量不足保持拒绝。
- 相关 self DCOL、CollisionSafety、参数与几何回归通过，文档与接口同步。

## 风险范围

共享控制内核与公开 typed result，需要规范和需求双线审查。数值测试证明离线计算行为；目标设备期限和生产权限由独立验收负责。
