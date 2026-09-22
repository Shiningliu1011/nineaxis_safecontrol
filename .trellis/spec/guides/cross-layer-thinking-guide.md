# 跨组件数据流

## 适用场景

改动同时影响以下三个以上环节时使用本指南：ROS input、配置验证、MoveIt、OSCBF 控制内核、command stream、MuJoCo plant、viewer、感知、硬件桥、运行快照或报告。

## 状态流检查

从状态来源沿消费者逐项确认：

1. 状态由真实编码器或 MuJoCo plant 产生。
2. message 携带关节名称、位置、速度和产生时间。
3. subscriber 使用兼容 QoS，并按 `DEFAULT_JOINT_NAMES` 映射。
4. freshness 使用同一时间基准检查。
5. controller 或 transition 读取同一 frame、单位和关节顺序。
6. 运行快照记录 raw topic、resolved topic 和参数来源。

任何环节缺少真实数据时，保持未知或拒绝继续，不能补充齐零状态或新时间戳。

## 命令流检查

从控制结果沿执行路径逐项确认：

1. 控制内核返回固定 shape 的结果和独立健康字段。
2. ROS adapter 在 host boundary 完成数据转换。
3. command message 使用公共 topic、QoS 和九轴顺序。
4. plant 或 hardware bridge 只消费明确有效的命令。
5. 超时、非有限数值、限制违反和反馈故障进入停止路径。
6. 停止原因被锁存并进入日志或报告。
7. 恢复要求健康状态和明确的外部确认。

## 坐标与几何检查

- 所有控制与规划计算使用 `base_link` 的固定 Y-up 语义。
- Viewer 的 Z-up 只存在于显示边界。
- 轨迹变换统一来自 `oscbf_trajectory.py`。
- 感知点、碰撞几何和任务目标明确记录 frame。
- 圆柱拟合、表面投影和采样顺序在消费者之间一致。
- 距离、梯度、障碍身份和场景版本来自同一场景状态。

## 配置流检查

追踪每个新增字段：

1. `config/oscbf_controller.yaml` 或对应配置文件中的来源。
2. 类型、范围和组合验证。
3. 显式 override 规则。
4. 生效值的唯一消费者。
5. 运行快照中的来源记录。
6. 缺失、未知、非法和 override 场景测试。

若字段没有消费者，应删除该字段。若消费者绕过公共验证，应把读取行为移回配置边界。

## 完成检查

- 每个边界的输入、输出、所有者和失败行为已经明确。
- 关节身份、顺序、frame、单位、dtype 和时间基准保持一致。
- topic remap 后仍满足 state 与 command 分离。
- 未测量、未知和禁用状态由独立字段表达。
- 日志、结果码、快照和报告能够说明失败阶段。
- 测试覆盖正常路径、边界输入和安全拒绝路径。
