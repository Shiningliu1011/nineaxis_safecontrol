---
status: accepted
date: 2026-09-21
---

# 碰撞数值参数由版本化离线 artifact 提供

新建 `CollisionParameterArtifact`，统一保存碰撞模块使用的安全范围、响应参数、容量和运行时间
限制。artifact 至少包含：

- LiDAR 距离、角度和入射方向误差范围；
- 外参标定误差；
- 逐点采集、消息传输、scene preparation 和控制延迟范围；
- 关节状态时间插值误差；
- voxel 尺寸、support `rho_mm` 计算参数和合并限制；
- 未跟踪障碍物速度与加速度上界；
- 命令、执行反馈和停车距离范围；
- pair-specific self `clearance_mm` 与 environment `clearance_mm`；
- `self_collision_cbf_rate_s_inv` 与 `environment_cbf_rate_s_inv`；
- occupied support、track、环境活动约束和机器人 ellipsoid 固定容量；
- DCOL iteration、solver residual、区间二分深度和 query batch 限制；
- scene preparation、device transfer、collision query、QP 和 100 Hz 完整路径 deadline。

每个参数记录单位、数据来源、测量方法、计算方法、适用设备、适用场景、置信要求和验证结果。
artifact 保存输入数据 hash、生成工具版本和 `parameter_hash`。`parameter_hash` 与允许接触列表、
几何身份和 kernel identity 共同生成 `collision_policy_hash`。

安全范围根据测量上界与规定置信要求计算；性能参数只能在这些范围内使用开发数据选择，并由
独立验收数据验证。运行进程只加载一个完整且 identity 验证通过的 artifact，禁止命令行、ROS
参数或运行状态覆盖其中任一安全字段。

## Decision Basis

毫米间距、传感器误差、动态范围、CBF 响应、固定容量和 deadline 共同决定同一条碰撞约束。
版本化 artifact 使数值来源、适用范围与验收结果可以追踪，并保证规划、执行和 OSCBF 加载相同
参数。

## Consequences

任何字段、输入数据或生成方法发生变化都会产生新的 `parameter_hash` 与
`collision_policy_hash`，旧路径准入、恢复确认和验收证据随即失效。新 artifact 必须重新执行
分层验收，运行期间不能进行参数自适应。
