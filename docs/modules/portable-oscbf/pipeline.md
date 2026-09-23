## 控制管线架构

```
轨迹参考 (弧长参数化)
  │
  ▼
6-DOF 速度级 OSC (P-only 名义控制, kp=50)
  │  P = N^T @ W_joint² @ N + J^T @ W_task² @ J
  │  W_task >> W_joint → QP 自动在零空间修正
  ▼
CBF-QP 安全滤波器 (qpax 弹性 QP)
  │  约束: 关节限位 CBF + 自碰撞 CBF + 障碍物 CBF
  ▼
安全关节速度 → Euler 积分 → 下一步 q
```

### 关键设计决策

| 决策 | 说明 |
|------|------|
| 纯 P 控制, 无 kd | 显式 Euler 离散化下 kd≥1 不稳定 |
| 固定 shape 障碍物槽位 | MAX_JAX_OBSTACLES=8, 未用槽位 h=1e6 填充, 避免 JAX 重编译 |
| 障碍物通过 h_args 传入 | 不烘焙到闭包, 避免障碍物更新触发 JAX 重编译 |
| qpax 弹性 QP | 支持自动微分 + JIT 编译 + 批量求解 |
| OSCBF task-consistent P | 让 QP 自动在零空间修正, 不干扰末端执行器任务 |
