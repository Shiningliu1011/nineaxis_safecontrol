## 11. 踩坑教训（必读）

> 完整版见项目 `LESSONS_LEARNED.md`（96 条）。以下为本路线（模块化 JIT + DCOL/OBB 碰撞 + 弹性 QP + 可操作度零空间）必须遵守的教训。

| # | 教训 | 后果 |
|---|------|------|
| **L1** | 纯 P，无 kd | 显式 Euler 下 kd≥1 发散 |
| **L15** | OSCBF 任务一致性 P 矩阵必须保留 | 否则避障会干扰末端任务 |
| **L67** | JAX 边界 = 控制 + **碰撞数值图**，不是 ROS/感知 I/O | 原始点云解码/ESDF 构建留在 JAX 外；碰撞距离计算走 DCOL（JAX 内） |
| **L70** | qpax warm-start 默认关闭 | 冷启动每步迭代多但整体更快、更安全 |
| **L77** | ESDF dtype 必须 float32（固定 ABI） | float64 触发首帧重编译 |
| **L79** | 求解器 slack ≠ 物理速率违例 | 弹性 QP 下用真实残差判断，不误把 δ 当物理越界 |
| **L83** | 弧长进给在腕部反向点放大 omega_per_m | 运行时硬上限 `ell_dot ≤ 0.15/‖omega_per_m‖` |
| **L94** | 单一巨型 JAX 内核首编慢、吃满 CPU | **选模块化 JIT**（各子函数独立 `@jax.jit`），并设置单线程 XLA |
| **L95** | 初始化瞬态（首帧 0.1mm 误差）不算稳态 | 验证时区分瞬态与稳态 |
| **DCOL** | DCOL alpha 是逐约束行参数，不是全局常数 | 按 V2 DPAX 校准流程逐对离线标定；未标定的近接触行不得用于控制 |
| **DCOL** | "包络相交" ≠ "实体碰撞" | 基于独立网格/STL 可复现证据配置拓扑豁免（L6/L71 流程），不静默跳过 |
| **零空间** | 逐关节裁剪会破坏 `J·qdot_N=0` | 可操作度零空间用**整体缩放** s_v，关节限速交给 QP |
| **零空间** | 阻尼伪逆下 `J(I-J⁺J)≠0` | 记录 `nullspace_leakage=‖J·qdot_N‖`，由任务 QP 项校正 |

**JIT 预热注意事项**（L94）: XLA 编译吃满所有 CPU 核，无 swap 系统会 OOM 重启。模块化 JIT 减小单图规模，仍需设置：

```bash
export XLA_FLAGS=--xla_cpu_multi_thread_eigen=false
export JAX_NUM_THREADS=1
```

---
