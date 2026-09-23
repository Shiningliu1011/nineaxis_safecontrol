## 6. 关键参数（目标值）

> 以下为建议采用的目标值。标注"🟢"者与 `portable_oscbf` 代码一致；标注"🟡"者随目标改动（弹性 QP / OBB+DCOL / 可操作度零空间）需要在新实现中设定。**移植到新机器人必须按单位/限位重新标定**，不要当作已验证常数直接套用。

### 6.1 控制器

| 参数 | 值 | 说明 |
|------|-----|------|
| `dt` | 0.002 s | 名义 500Hz；部署 100Hz 也通过验证 |
| `kp_pos` | 60（fixed）/ 120（非 fixed） | 位置增益 |
| `kp_orient` | 10 | 姿态增益 |
| `damping` | 1e-3 | 标称伪逆阻尼 |
| `qdot_min / qdot_max` | 关节速度限位 | **控制器内 clip**（OSCBF 源码形式）🟡（目标；现状由 QP box 承担） |
| `w_pos` / `w_orient` / `w_joint` | 20 / 10 / 0.1 | 任务一致性 P 矩阵权重 |
| `enable_x64` | True | JAX float64 |
| `solver_tol` | 1e-3 | qpax 求解容差 |

### 6.2 CBF

| 参数 | 值 | 说明 |
|------|-----|------|
| `alpha_joint_limit` | **8.0** 🟢 | 关节限位 CBF 增益（`oscbf_velocity_config.py:82` 硬编码 8.0；⚠️ `config/nineaxis.yaml:135` 写 5.0，两处不一致需统一） |
| `joint_limit_cbf_margin` | **0.01** 🟢 | 限位内缩（`joint_limit_contract.py:12` 为 10mm，非 1mm） |
| `d_safe_collision` | 0.03 m 🟢 | 自碰撞安全距离（现状球模型值；OBB 路线改为 `dcol.d_safe` 近接触标定） |
| `obstacle_h_baseline_alpha` | 10.0 🟢 | 障碍物 CBF 增益基线 |
| `singularity_tol` | 0.005 🟢 | 奇异值下限 |
| `relax_cbf` | **True** 🟡 | **弹性 QP**（cbfpy→`qpax.solve_qp_elastic`），取消停车；⚠️ 现状 `False` 硬约束+受控停车 |
| `cbf_relaxation_penalty` | 1e4 ~ 1e6 🟡 | CBF slack 惩罚 ρ（取大值，正常工况 δ≈0） |
| `MAX_JAX_OBSTACLES` | 8 🟢 | 障碍物槽位数 |
| `smooth_min_temperature` | 0.01 🟢 | 障碍物 soft-min 聚合温度 |
| `dcol.alpha` | 逐对标定 🟡 | DCOL 每约束行 CBF 增益 α，V2 DPAX 离线校准（3/14 行已定、11 行需近接触数据） |
| `dcol.d_safe` | 按近接触标定 🟡 | DCOL 安全距离（近接触数据驱动，勿套用球模型值） |

### 6.3 可操作度零空间

| 参数 | 值 | 说明 |
|------|-----|------|
| `nullspace.policy` | `manipulability_gradient` | 在线可操作度策略（替换关节中点） |
| `manipulability.metric` | `regularized_logdet` | φ = ½logdet(JₛJₛᵀ + εI) |
| `manipulability.epsilon` | 1e-6 | 正则化 ε |
| `manipulability.gain` | 0.15 | 可操作度增益 k_m |
| `manipulability.max_weighted_speed` | 0.25 | 零空间速度上限 v_N,max |
| `gradient.method` | `autodiff` | 自动微分（JAX） |
| `gradient.filter_enabled` | true | 梯度低通 |
| `gradient.filter_alpha` | 0.2 | 低通系数 β |
| `activation.enabled` | false | 首版关闭激活，验证梯度后再启用 |
| `joint_metric.type` | `velocity_normalized` | W_q = diag(1/ẋ_max²) |
| `characteristic_length` | 0.4 | 特征长度 l_c（任务尺度矩阵） |

> 参数来自 `docs/OSCBF_在线可操作度零空间目标_架构参考.md` §13，具体数值需按机器人单位/限位重新标定。

### 6.4 路径跟踪

| 参数 | 值 | 说明 |
|------|-----|------|
| `projection_half_window_segments` | 96 | 弧长投影半窗 |
| `max_projection_speed_m_s` | 0.12 | 投影最大速度 |
| `reference_lead_m` | 1e-5 (0.01mm) | 参考超前量（投影后积分修正后，离散时序滞后已消除，0.01mm 足以让参考点略超前于投影并低于 0.1mm 验收门） |
| `cross_track_stop_m` | 1e-3 | 横向误差停车阈值 |
| `endpoint_braking_deceleration_m_s2` | 0.05 | 终点虚拟制动 |
| `maximum_tool_axis_speed_rad_s` | 0.15 | 工具轴角速度上限（防腕部反向点跳变） |
| `endpoint_settle_s` | 0.5 | 终点稳定时间 |

### 6.5 动态障碍物（若启用）

| 参数 | 值 |
|------|-----|
| `cbf_alpha` | 1.5 |
| `d_safe` | 0.08 m |
| `activation` | 0.20 m |

---
