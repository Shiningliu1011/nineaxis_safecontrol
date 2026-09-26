# 独立审查

审查基准 `8a6a3ce`，规范与需求分别由独立 reviewer 只读检查。

## 规范

未发现待处理问题。geometry hash、policy、完整 link pair、固定槽位、共享运动学索引、float64、独立行、residual、梯度有限性和 device 完成时间符合适用规范。`git diff --check 8a6a3ce` 通过。

## 需求

未发现待处理问题。对偶目标、witness、二分方向和 envelope derivative 与原始 DCOL 问题一致；pair-specific margin 使用双方最短半轴。构造时检查半轴之和及 margin 的有限性，极端 clearance 配置会被拒绝；`test_nonfinite_scale_margin_is_rejected_before_query` 由需求 reviewer 独立运行通过。

官方 Julia 参考、80 位独立 KKT、九轴梯度、完整 primitive pair、未收敛、同心、非有限结果、环境尚未实现和 deadline 均由公开操作验证。生产控制入口保持现有行为。

## 检查边界

reviewer 检查源码与规定范围内的复现；完整测试、官方参考生成与运行证据由主会话负责。目标设备 deadline 与生产切换仍属于地图后续验收。
