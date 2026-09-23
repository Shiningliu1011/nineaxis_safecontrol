## 3. 控制管线（单条链路）

```
  .mat 轨迹 → ik_data_loader → 弧长路径几何 (PathGeometry)
                                   │
  主机端每步 (JaxControlLoop) ──────┤
  ┌─────────────────────────────────┴────────────────────────────┐
  │  path_tracking_step(q, path_state, obs_*, sdf_*)             │
  │                                                              │
  │  ┌──────── 模块化 JIT (仿 OSCBF 源码, 各自 @jax.jit) ────────┐ │
  │  │  ① 参考采样    PathReference(progress_m, pos, rot, omega) │ │
  │  │  ② 标称控制    6-DOF 速度级 OSC (P-only)                 │ │
  │  │     task_vel = [v_des;ω_des] - K_task·err_6d             │ │
  │  │     u_nom = J_hash@task_vel                              │ │
  │  │           + s_v·k_m·ρ·N·W_q⁻¹·Nᵀ·∇φ  (可操作度零空间)    │ │
  │  │     u_nom = clip(u_nom, qdot_min, qdot_max)  ← OSCBF 源码 │ │
  │  │  ③ 碰撞+距离   DCOL 可微距离 (OBB 自碰撞 + 障碍物)        │ │
  │  │     G_row = ∇h 经 jax.grad(proximity) 获得               │ │
  │  │  ④ CBF 构建    h_2(q, obs_*, sdf_*) 全部约束 (cbfpy)      │ │
  │  │     (OBB/DCOL 自碰撞 + 限位 + 障碍物 + ESDF + 奇异性)     │ │
  │  │  ⑤ QP 求解     弹性 QP, relax_cbf=True                   │ │
  │  │     min ½uᵀPu+qᵀu+ρ‖δ‖² s.t. G·u-δ ≤ α·h + dh/dt        │ │
  │  │     u_min ≤ u ≤ u_max (QP 固有硬约束)                    │ │
  │  │  ⑥ 积分        q_next = q + u_safe·dt                    │ │
  │  └───────────────────────────────────────────────────────────┘ │
  │    输出: u_safe, q_next, err_6d, qp_ok, dyn_min, δ_slack     │
  └──────────────────────────────────────────────────────────────┘
        │
  DCOL 障碍物距离 (JAX 内, 与自碰撞同内核)
  原始点云感知 (JAX 外): FCL/ESDF → 体素化/距离场 → 经 obs_* / sdf_* 传入
```

**单步数据流**（主机端 → 内核）：

| 输入 | 形状 | 说明 |
|------|------|------|
| `q` | `(9,)` float64 | 当前关节角 |
| `path_state` | `(n,)` | 弧长路径状态（`jax_path_following` 状态机） |
| `obs_pos` | `(8,3)` | 障碍物中心，未用槽位填 0 |
| `obs_radii` | `(8,)` | 障碍物半径 |
| `obs_enabled` | `(8,)` | 1=启用 0=禁用 |
| `obs_d_safe` | `(8,)` | 每障碍物安全距离 |
| `obs_vel` | `(8,3)` | 障碍物速度（动态 CBF 的 dh/dt） |
| `obs_radius_dot` | `(8,)` | 障碍物半径变化率 |
| `obs_alpha` | `(8,)` | 每障碍物 CBF 增益 |
| `sdf_distance` | `(nx,ny,nz)` | ESDF 距离场（可选） |
| `sdf_origin` / `sdf_voxel_size` / `sdf_enabled` / `sdf_margin` | — | 距离场栅格合同 |

**输出**: `q_next`（下一步关节角）、`u_safe`（安全速度）、`err_6d`（6D 任务误差）、`qp_ok`（QP 成功标志）、`dyn_min`（最小安全裕度）、`qp_diagnostics`。

---
