---
status: accepted
date: 2026-09-21
---

# 自碰撞与环境碰撞分别使用固定 CBF 响应率

碰撞命令 QP 使用一阶时间变化 CBF 条件。自碰撞 primitive 行满足：

`grad_h_q @ q_dot + self_collision_cbf_rate_s_inv * h >= 0`

环境 primitive 行满足：

`grad_h_q @ q_dot + partial_h_partial_t + environment_cbf_rate_s_inv * h >= 0`

`self_collision_cbf_rate_s_inv` 与 `environment_cbf_rate_s_inv` 分别配置，单位均为 `s^-1`。
同一类别的全部活动 primitive 行使用相同响应率，运行期间不按 barrier、pair 或接近速度改变
响应率。动态障碍物的名义运动只进入 `partial_h_partial_t`，不确定性只进入 `beta` 与
`beta_dot`。

新配置与诊断禁止使用 `alpha` 指代 CBF 响应率。DCOL 几何结果统一称为
`proximity_scale`，CBF 响应参数统一使用 `cbf_rate`，防止两个数学含义混用。

## Decision Basis

自碰撞与环境碰撞具有不同的几何灵敏度、时间项和不确定性来源，分别配置固定响应率可以独立
验证两类行为，同时保持每类约束的一致性和运行可复现性。

## Consequences

两个响应率必须结合关节速度上限、控制周期、传感器延迟、动态障碍物范围和 QP 可行性执行
离线参数验证。参数值及其单位进入碰撞策略身份；任何变更都会撤销旧验证结果，并要求重新
执行准入测试。
