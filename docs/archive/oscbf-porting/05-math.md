## 5. 数学模型与公式

### 5.1 运动学（POE）

```
T_ee(q) = exp([S_1]·q_1) · ... · exp([S_9]·q_9) · M
  其中 S_i 为世界系螺旋轴（零位构型），M 为 q=0 时末端 SE(3)
  转动关节: S = [ω; p×ω]
  棱柱关节: S = [0; v]
```

空间雅可比（螺旋轴）:
```
J_s(:,i) = Ad_{T_{i-1}} · S_i,    Ad = [R, 0; skew(p)R, R]
6D 雅可比: J_full = [J_pos; J_rot],  J_rot = J_s[:3,:]
```

### 5.2 标称控制（6-DOF 速度级 OSC，纯 P）

```
err_6d = [p - p_des ; e_rot(R, R_des)]          # 6D 任务误差
e_rot = -0.5 · Σ_k cross(R[:,k], R_des[:,k])     # 姿态误差 (论文方法)

# 任务速度 (P-only, 无 kd)
task_vel = [v_des ; ω_des] - K_task · err_6d      # K_task = diag(kp_pos×3, kp_orient×3)
J_hash = Jᵀ · (J·Jᵀ + λ²I)⁻¹                     # 阻尼伪逆, λ=1e-3
N = I - J_hash·J                                  # 零空间投影

# 零空间: 在线可操作度梯度 (不再用固定关节中点/回中)
φ     = ½ · log det(J_s·J_sᵀ + εI)               # J_s = S_x·J, S_x = diag(I₃, l_c·I₃)
g_m   = ∇_q φ                                    # 自动微分 (JAX)
d_m   = N·W_q⁻¹·Nᵀ·g_m                          # 投影到当前零空间
s_v   = min(1, v_N,max / ‖k_m·ρ·d_m‖)            # 整体缩放, 不逐关节裁剪
u_null = s_v · k_m · ρ(q) · d_m                  # ρ = 平滑激活系数

# 标称 + 速度限幅 (目标: OSCBF 源码形式的控制器内 clip)
u_nom = J_hash·task_vel + u_null
u_nom = clip(u_nom, qdot_min, qdot_max)   # 目标: 与 OSCBF PoseTaskVelocityController 一致
                                            # ⚠️ 现状 u_nom 不 clip, 速度限幅由 QP box 承担

# 注: QP 的 u_min ≤ u ≤ u_max 是 cbfpy QP 约束集的一部分 (与 CBF 行同类); 弹性模式下 box 行同样带 slack, 见 §5.5
```

### 5.3 OSCBF 任务一致性 P 矩阵（核心）

```
P = N_nullᵀ·W_joint²·N_null + Jᵀ·W_task²·J
  W_task = diag(w_pos·I₃, w_orient·I₃),  W_joint = w_joint·I₉
  λ_qp = 1e-3   (阻尼伪逆 J_hash = Jᵀ(J·Jᵀ + λ²I)⁻¹, 与标称共用 λ=1e-3)
```

**性质**: 权重平方后位置/零空间比 = 20²/0.1² = **40,000:1**，姿态/零空间比 = 10²/0.1² = **10,000:1**。QP 优先在零空间修正以满足 CBF，末端任务几乎不变。这是解决"避障 vs 跟踪"冲突的关键，**必须原样保留**。

> ⚠️ **与 OSCBF 源码的关键差异（务必知晓）**: OSCBF 源码 `OSCBFVelocityConfig._P()` 用**动力学一致性逆** `J_bar = M⁻¹Jᵀ(J·M⁻¹·Jᵀ)⁻¹`（需质量矩阵，零空间投影也 M 加权）；本项目用**阻尼伪逆** `J_hash = Jᵀ(J·Jᵀ + λ²I)⁻¹`（纯运动学，免动力学建模）。两者 P 矩阵形式相同，但投影子不同——本项目为移植到任意机械臂主动放弃了动力学一致性。

### 5.4 CBF 约束（相对度 2）

```
速度级:  f(q) = 0,  g(q) = I  →  ∇hᵀ·u + α(h) ≥ 0
QP 编码:  G·u ≤ α·h + dh/dt        (G_row = -∇hᵀ)
动态障碍物: dh/dt = -nᵀ·v_obs - ṙ_obs   (显式传入, cbfpy 不会自动加)
碰撞梯度: ∇h 由 DCOL 提供 —— jax.grad(dpax.endpoints.proximity)(q)
```

**固定约束拓扑**（全部在 `h_2()` 中拼接，行数恒定）:

| 约束 | 行数 | 说明 |
|------|------|------|
| 关节限位 | 2×9 | `h = q_max - q - margin` 与 `h = q - q_min - margin` |
| 自碰撞（OBB via DCOL） | N_pairs | OBB 几何 + DCOL 可微距离，`obb_collision_model.py` 标定，非相邻连杆对 |
| 障碍物（DCOL） | N_obb×8（或聚合） | DCOL 基元距离，masked，未用槽位 `h=1e6` |
| ESDF | N_obb（可选） | 距离场采样 `SDF(p_obb) - margin`（OBB 无半径，按盒表面/最近点采样；原始点云感知） |
| 奇异性 | 1 | `σ_min(J_pos) - tol` |

### 5.5 QP 问题（qpax 弹性 QP — 目标）

```
变量:  x = [u (9,) ; δ (n_cbf,)]
目标:  min  ½·uᵀ·P·u + qᵀ·u  +  ρ·‖δ‖²          # 弹性松弛, relax_cbf=True
约束:  G·u - δ ≤ h + dh/dt        (CBF 可弹性缓解, 不可行不再停车)
       u_min ≤ u ≤ u_max           (速度盒)
       δ ≥ 0
```

- 求解器: cbfpy `CBF.qp_solver` → `qpax.solve_qp_elastic`（`relax_cbf=True` 触发；现状 `relax_cbf=False` 走硬 `solve_qp`）
- `ρ` = `cbf_relaxation_penalty`（取大值，保证正常工况 δ≈0，仅在约束冲突时启用松弛）
- ⚠️ **弹性模式下 `u_min/u_max` box 行同样带 slack**（`solve_qp_elastic` 对所有不等式行统一加松弛惩罚）——不是硬约束。控制器内 clip（目标）才是保证名义命令在界内的机制
- 不活跃槽位用 `h=1e6` 填充 → `G·u ≤ 1e6` 恒满足 → QP 自动忽略，**JIT 拓扑恒定，不重编译**
- **取消受控停车**（目标）：CBF 冲突由 slack 弹性缓解；`δ_slack` 作为诊断量记录，不触发 HARD_STOP。⚠️ 现状是 `relax_cbf=False` 硬约束 + `apply_qp_health_gate` 受控停车

---
