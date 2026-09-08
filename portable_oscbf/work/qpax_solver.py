#!/usr/bin/env python3
"""
qpax_solver.py
==============
使用 qpax 的可微分 QP 求解器。

qpax 是 JAX 可微分 QP 求解器，支持:
- 自动微分
- 批量求解
- JIT 编译

使用弹性 QP (solve_qp_elastic) 匹配 OSQP 的弹性松弛行为。

关键设计: 固定约束槽位 (max_constraints)，避免 JAX JIT 重编译。
不活跃约束用 h=1e6 填充 (QP 自动忽略)。
参考 cbfpy 源码的固定 num_cbf 模式。
"""

import time
import numpy as np
import jax
import jax.numpy as jnp
from typing import List, Tuple, Optional

try:
    import qpax
    QPAX_AVAILABLE = True
except ImportError:
    QPAX_AVAILABLE = False


class QpaxSolver:
    """qpax QP 求解器 (固定约束槽位，避免 JIT 重编译)

    参考 cbfpy 源码: num_cbf 在初始化时固定，运行时通过 h 值区分活跃/不活跃。
    不活跃约束: h=1e6 → G@u <= 1e6 永远满足 → QP 自动忽略。
    """

    def __init__(self, max_constraints: int = 64):
        if not QPAX_AVAILABLE:
            raise ImportError("qpax 未安装。运行: pip install qpax")

        self.max_constraints = max_constraints
        self.available = True

        # OSQP 兼容属性
        self.last_qp_setup_ms = 0.0
        self.last_qp_update_ms = 0.0
        self.last_qp_solve_ms = 0.0
        self.last_qp_reused = False
        self.last_max_violation = 0.0
        self.last_slack_max = 0.0

        # JIT 预热: 用固定尺寸触发编译 (只编译一次)
        self._warmup()

    def _warmup(self):
        """JIT 预热: 用固定 max_constraints 尺寸触发编译"""
        try:
            n = 9
            m = self.max_constraints
            Q_dummy = jnp.eye(n)
            q_dummy = jnp.zeros(n)
            G_dummy = jnp.zeros((m, n))
            h_dummy = jnp.full(m, 1e6)  # 全部不活跃
            qpax.solve_qp_elastic(
                Q_dummy, q_dummy, G_dummy, h_dummy,
                penalty=1e4, backend='e', max_iter=10,
            )
        except Exception:
            pass  # 预热失败不影响后续使用

    def solve(self, u_nom, constraints, dq_max,
              w_pos=1.0, w_orient=1.0, w_joint=0.1,
              J_pos=None, u_lower=None, u_upper=None,
              slack_penalty=1e4,
              u_safe_prev=None, temporal_lambda=0.0,
              temporal_wu=None) -> Tuple[np.ndarray, bool, float]:
        """求解弹性 QP (固定约束槽位，与 OSCBFQPSolver 接口兼容)

        参考 cbfpy 源码: 所有约束填入固定大小数组，不活跃的用安全值填充。
        JAX JIT 只在首次调用编译一次 (shape 不再变化)。

        带时序近端稳定化:
          min  0.5*u^T*P_task*u + q_task^T*u
             + λ/2 ||W_u (u - u_safe_prev)||²
             + ρ * ||s||²

        Parameters
        ----------
        u_nom : np.ndarray (9,)
            标称控制输入
        constraints : List[CbfConstraint]
            CBF 约束列表 (可能包含不活跃约束)
        dq_max : np.ndarray (9,)
            关节速度限幅
        w_pos, w_orient, w_joint : float
            任务空间和关节空间权重
        J_pos : np.ndarray (6, 9), optional
            雅可比矩阵 (用于构造 P 矩阵)
        u_lower, u_upper : np.ndarray (9,), optional
            速度上下界 (加速度限幅后的速度盒)
        slack_penalty : float
            松弛惩罚系数 (传给 qpax 弹性 QP)
        u_safe_prev : np.ndarray, optional
            上一帧 u_safe, 用于 temporal proximal term
        temporal_lambda : float
            时序近端项权重 λ。0 = 禁用。
        temporal_wu : np.ndarray, optional
            每关节权重向量 W_u (对角)。None = 全 1。

        Returns
        -------
        u_safe : np.ndarray (9,)
            安全控制输入
        success : bool
            是否成功
        solve_time_ms : float
            求解时间 (ms)
        """
        t0 = time.perf_counter()

        n_joints = len(u_nom)
        u_nom_jax = jnp.array(u_nom)
        dq_max_jax = jnp.array(dq_max)
        m = self.max_constraints

        # 构造 P 矩阵 (任务一致性)
        # 参考 cbfpy/oscbf_configs.py: P = N^T W_joint^2 N + J^T W_task^2 J
        if J_pos is not None:
            J = jnp.array(J_pos)
            damping = 1e-8  # 与 OSQP 求解器一致 (非 cbfpy 的 1e-3)
            JJT = J @ J.T + damping * jnp.eye(6)
            J_hash = J.T @ jnp.linalg.inv(JJT)
            N = jnp.eye(n_joints) - J_hash @ J

            W_task_sq = jnp.diag(jnp.array([
                w_pos**2, w_pos**2, w_pos**2,
                w_orient**2, w_orient**2, w_orient**2
            ]))
            W_joint_sq = w_joint**2 * jnp.eye(n_joints)

            P_u = N.T @ W_joint_sq @ N + J.T @ W_task_sq @ J
            P = 0.5 * (P_u + P_u.T)  # 对称化
        else:
            P = jnp.eye(n_joints)

        # ---- Temporal proximal term: λ/2 ||W_u (u - u_safe_prev)||² ----
        P_task = P  # 保存不含 temporal 的原始 P, 用于 q 向量
        _temporal_wu_sq_jax = None
        if temporal_lambda > 0.0 and u_safe_prev is not None:
            if temporal_wu is not None:
                wu = jnp.array(temporal_wu).reshape(n_joints)
            else:
                wu = jnp.ones(n_joints)
            _temporal_wu_sq_jax = temporal_lambda * (wu ** 2)
            P = P + jnp.diag(_temporal_wu_sq_jax)
            P = 0.5 * (P + P.T)  # 保持对称

        # ---- 固定槽位约束构建 (参考 cbfpy 模式) ----
        # 不活跃约束: h=1e6, G=0 → G@u <= 1e6 永远满足 → QP 忽略
        G_fixed = jnp.zeros((m, n_joints))
        h_fixed = jnp.full(m, 1e6)  # 安全默认值

        # 填入实际约束 (最多 max_constraints 个)
        n_actual = min(len(constraints), m)
        for i in range(n_actual):
            c = constraints[i]
            if c.active:
                G_fixed = G_fixed.at[i].set(jnp.array(c.G_row))
                h_fixed = h_fixed.at[i].set(c.h_bound)
            # 不活跃约束保持默认 (G=0, h=1e6)

        # 速度盒约束: -dq_max <= u <= dq_max
        velocity_lower = -dq_max if u_lower is None else jnp.maximum(-dq_max, jnp.array(u_lower))
        velocity_upper = dq_max if u_upper is None else jnp.minimum(dq_max, jnp.array(u_upper))

        G_vel_upper = jnp.eye(n_joints)   # u <= velocity_upper
        h_vel_upper = velocity_upper
        G_vel_lower = -jnp.eye(n_joints)  # -u <= -velocity_lower
        h_vel_lower = -velocity_lower

        G_all = jnp.vstack([G_fixed, G_vel_upper, G_vel_lower])
        h_all = jnp.concatenate([h_fixed, h_vel_upper, h_vel_lower])

        # 目标函数: 0.5 * x^T Q x + q^T x
        # q = -P_task @ u_nom - λ * diag(wu²) @ u_safe_prev
        Q_mat = P
        q_vec = -P_task @ u_nom_jax
        if _temporal_wu_sq_jax is not None and u_safe_prev is not None:
            q_vec = q_vec - _temporal_wu_sq_jax * jnp.array(u_safe_prev).reshape(n_joints)

        # 求解弹性 QP (带松弛变量)
        # shape 固定: G_all = (max_constraints + 18, 9) → JAX 只编译一次
        try:
            x, s, z, y, converged, iters = qpax.solve_qp_elastic(
                Q_mat, q_vec, G_all, h_all,
                penalty=slack_penalty,
                backend='e',
                max_iter=100,
                solver_tol=1e-5,
            )

            u_safe = np.array(x[:n_joints])

            # 裁剪到速度盒 (安全兜底)
            u_safe = np.clip(u_safe,
                             np.array(velocity_lower),
                             np.array(velocity_upper))

            t1 = time.perf_counter()
            solve_time = (t1 - t0) * 1000

            # 更新诊断属性
            self.last_qp_solve_ms = solve_time
            if n_actual > 0:
                violations = np.array(G_fixed @ jnp.array(u_safe) - h_fixed)
                self.last_max_violation = float(np.max(violations[:n_actual]))
                slack_arr = np.array(s)
                self.last_slack_max = float(np.max(slack_arr)) if len(slack_arr) > 0 else 0.0
            else:
                self.last_max_violation = 0.0
                self.last_slack_max = 0.0

            return u_safe, bool(converged), solve_time

        except Exception as e:
            # QP 求解失败，回退到裁剪
            velocity_lower_np = np.array(velocity_lower)
            velocity_upper_np = np.array(velocity_upper)
            u_safe = np.clip(u_nom, velocity_lower_np, velocity_upper_np)
            t1 = time.perf_counter()
            return u_safe, False, (t1 - t0) * 1000
