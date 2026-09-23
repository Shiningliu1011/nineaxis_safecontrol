## 附录 A: 目标架构文件清单（含复制/新建标注）

> 🟢 = `portable_oscbf/` 有参考代码，**复制改造**；🟡 = `portable_oscbf/` 没有，**按本文档规格从零实现**（或从 `DCOLuse/` 迁入）。清单描述目标架构的文件布局。先读 §1.1 对照表确定每项的实现方式。

```
portable_oscbf/work/
├── nineaxis_manipulator_jax.py   # JAX 运动学 🟢（OBB 变换方法为目标）
├── obb_collision_model.py        # OBB 包络盒数据 (贴合 STL) 🟡 不存在，需新建
├── dpax_collision.py             # DCOL 可微碰撞内核 🟡 现位于 DCOLuse/，需迁入热路径
├── dcol_alpha_calibration.py     # DCOL alpha 逐对离线校准 🟡 不存在，需新建
├── actuator_limits.py            # 限位 🟢
├── ik_data_loader.py             # 轨迹加载 🟢
├── jax_path_following.py         # 弧长路径状态机 🟢
├── jax_posture_reference.py      # 姿态参考 🟢
├── nullspace_policy.py           # 可操作度零空间策略 🟡 不存在，需新建
├── manipulability_metric.py      # φ=½logdet(JₛJₛᵀ+εI) 及其梯度 🟡 不存在，需新建
├── oscbf_velocity_config.py      # cbfpy CBFConfig 🟢（relax_cbf=True 为目标）
├── jax_barrier_terms.py          # 障碍物 CBF 🟢
├── qp_solver_health.py           # QP 健康 🟢
├── jax_kernel_factory.py         # 控制内核编译入口 🟢（模块化 JIT 为目标）
├── jax_control_facade.py         # JaxControlLoop facade 🟢
└── safety_snapshot.py            # ESDF 快照 (原始点云感知, 可选) 🟢
```

> ⚠️ `qpax_solver.py` 未列入——其 `solve_qp_elastic` 包装类现状无调用者（legacy）；目标路线由 cbfpy `CBF.qp_solver` 直接调 qpax。

**配套**: `config/`（YAML）、`data/`（轨迹）、`urdf/`（机器人模型）、`tests/`（36 个单测文件）。
