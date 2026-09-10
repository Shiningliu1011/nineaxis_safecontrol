# 冗余候选局部比较证据

在仓库根目录运行：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 JAX_PLATFORMS=cpu python3 docs/planning/oscbf-reuse/research/5-redundancy-evidence/compare.py
```

脚本会覆盖本目录 JSON 结果与 metadata；请先复制目录保存历史结果。第一次测量使用 `output/redundancy-5/compare.py`，显式 `PYTHONPATH=portable_oscbf/vendor/dpax`；交接脚本仅调整仓库发现与 vendor 导入路径，计算内容相同。`assets.sha256.json` 固定交接时文件指纹，重跑不会自动重写此历史指纹。

- 环境：见 metadata，JAX CPU/x64；dt=0.01s，两个测试初态各300步。
- 三个候选：默认 q_des=q、已有姿态入口 q_des=关节中点、已有 ManipulabilityGradientPolicy 默认参数。不新增策略算法。
- w_pos=40、w_orient=10、w_joint=0.1、temporal_lambda=0.2、kp_pos=160、kp_orient=10、kp_joint=0.45、nullspace_speed_limit=0.18、damping=0.05，与本机配置数值一致。
- 固定工具位置/轴向，使用 tracking_step 的 Euler 积分，无外部障碍/ESDF、无 ROS、无真实执行器。不是完整路径或部署回放；也没有单独验证接触真值。
- 位置/轴向角误差在每步积分后计算；工具轴角速度为 `norm(cross(Jω @ u_safe, axis))`，不把自由 roll 算作工具轴方向变化。
- 限位余量为 `min_i(min(q_i-lo_i,hi_i-q_i)/(hi_i-lo_i))`，按各轴范围归一化，不能解释为米或弧度。初态与最终值均记录。
- sigma5 为 `diag(1,1,1,0.4,0.4) J5 diag(dq_max)` 的最小奇异值，rank 阈值为最大奇异值的1e-10。该尺度是比较口径，不是用户已定安全阈值；满秩样本的零空间维度为4，不证明所有构型都满秩。
- active_step_fraction 仅沿用内核 `abs(G @ u_candidate - h_qp)<1e-4` 的行活跃诊断；包含求解不等式，不能当成纯碰撞CBF主动率。此批次为0，没有测到约束冲突下的优先行为。
- delta_slack 是 elastic 求解器 slack 诊断；min_h 按当前内核屏障输出，不能作为独立几何真值。QP成功不等于安全验收。
- 汇总、逐步原始记录与源码SHA256同时保存。首次缺 dpax 的导入错误已由显式vendor路径解决；run.log为成功运行日志。

本批次用于比较局部行为与发现反例，不决定生产策略、任务误差阈值、硬约束实现或实时预算。完整路径、近奇异与活跃约束、最终命令和硬件响应的证据由既有后续票承接。
