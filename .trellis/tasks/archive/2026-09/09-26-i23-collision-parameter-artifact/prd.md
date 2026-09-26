# CollisionParameterArtifact 与 policy identity

来源：[建立 CollisionParameterArtifact 与 policy identity](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/102)，规格决定 41、42 与 ADR 0045。前置接口任务已关闭，用户已授权按执行地图完成本票。

## 需求

- 提供版本化安全参数 schema、严格加载器和稳定 identity。
- 覆盖 ADR 0045 的误差、延迟、间距、CBF 响应率、固定容量、solver、区间二分和各阶段 deadline。
- 每项参数携带单位、来源、测量与计算方法、适用设备与场景、置信要求及独立验证结果；记录输入数据 hash 和生成工具版本。
- `parameter_hash` 与允许接触列表、geometry identity、kernel identity 共同决定 `collision_policy_hash`。
- 运行对象中的安全字段不可覆盖。旧 policy 对应的 scene 及已准备的 scene 在新配置下必须拒绝。

## 验收

- 相同内容在文件格式变化及独立进程中得到相同 hash；任一参数值、元数据、数据 hash、生成方法或 policy 输入变化都会改变 identity。
- 缺少必需字段、未知字段、非有限值、错误单位、非法容量、无效验证、设备或场景不匹配以及 hash 不匹配均拒绝加载。
- 配置值来自 artifact，公开接口拒绝旧 identity，并维持无效结果 mask。
- 所有行为使用真实 JSON、真实加载器及 CollisionSafety 公开操作验证；文档说明数据准备和当前能力。

## 范围边界

本票提供 schema 与软件校验。实际安全数值、设备测量和独立验收由地图后续任务提供。当前 kernel、轨迹准入及恢复状态机的数值行为保持其既有完成范围，本票不生成新的运动权限。
