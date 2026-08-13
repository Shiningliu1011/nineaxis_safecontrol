#!/usr/bin/env python3
"""
oscbf_qp_solver.py
==================
独立的 OSCBF QP 求解器 — 使用 osqp 直接 Python API (无 cvxpy)。

QP 形式:
  min  0.5 * u^T * P * u + q^T * u
  s.t. l <= A * u <= u    (CBF 不等式 + 速度限幅)

CBF 约束 G*u <= h 编码为 A*u <= h, 其中 A = G, upper = h.
关节速度限幅编码为 lb <= u <= ub.
"""

import logging
import numpy as np
from typing import List, Tuple, Optional
from scipy.sparse import csc_matrix, triu
from dynamic_obstacles import CbfConstraint

_logger = logging.getLogger(__name__)


class OSCBFQPSolver:
    """使用 osqp 直接 API 的 QP 求解器"""

    def __init__(self, reuse_osqp: bool = True, max_constraints: int = 32):
        self._osqp = None
        self.reuse_osqp = bool(reuse_osqp)
        self.max_constraints = int(max_constraints)
        self.setup_count = 0
        self.update_count = 0
        self.fallback_count = 0
        self.last_status = ""
        self.last_max_violation = 0.0
        self.last_slack_max = 0.0
        self.last_rate_slack_max = 0.0
        # Per-call QP timing breakdown
        self.last_qp_setup_ms = 0.0
        self.last_qp_update_ms = 0.0
        self.last_qp_solve_ms = 0.0
        self.last_qp_reused = False
        # Solver diagnostics (用于跳变分析)
        self.last_iter = 0
        self.last_dual_residual = 0.0
        self.last_objective = 0.0
        # Fallback projection settings
        self._fallback_max_iter = 20
        # Slack upper bound: physically meaningful (0.1m = 100mm for distance CBFs)
        self._slack_upper_bound = 0.1
        # Pre-allocated padded arrays (created on first solve, grown when capacity exceeded)
        self._P_pad = None
        self._q_pad = None
        self._A_pad = None
        self._l_pad = None
        self._u_pad = None
        self._current_capacity = max_constraints
        self._prob = None  # cached OSQP problem for workspace reuse
        self._last_n_con = -1  # tracks n_con for sparsity pattern stability
        # CSC pattern caching: when n_con and capacity are unchanged, skip np.array_equal
        self._csp_key = None  # (n_con, capacity) tuple for CSC validity
        self._n_joints = None
        # Pre-computed slack diagonal for P matrix (avoids per-step loop)
        self._slack_diag = None  # shape (capacity,), value=slack_penalty for active, 0 for padded
        # Cached CSC matrices for fast data-only update
        self._P_csc_cached = None
        self._A_csc_cached = None
        # P matrix caching: reuse when J_full barely changes
        self._prev_J_full = None
        self._prev_P_u = None
        self._prev_w = (0.0, 0.0, 0.0)  # (w_pos, w_orient, w_joint)
        self._p_cache_rel_threshold = 1e-4  # relative Frobenius norm threshold
        try:
            import osqp
            self._osqp = osqp
            self._available = True
        except ImportError:
            self._available = False

    @property
    def available(self) -> bool:
        return self._available

    def _solve_osqp(self, P: np.ndarray, q_vec: np.ndarray, A: np.ndarray,
                    l_vec: np.ndarray, u_vec: np.ndarray, n_con: int = 0,
                    n_pad: int = 0):
        """Solve QP via OSQP with workspace reuse when sparsity pattern is stable.

        Reuse is possible when n_con and capacity are unchanged between calls,
        because the padded matrices have identical CSC sparsity patterns.
        Uses epsilon trick to ensure G_row zero entries are structurally non-zero.

        Optimization: when (n_con, capacity) matches the previous call, the CSC
        sparsity pattern is guaranteed identical (same padded shape, same G_row
        density), so we skip the expensive np.array_equal checks and go straight
        to data-only update.
        """
        import time as _time
        n_vars = P.shape[0]
        csp_key = (n_con, n_pad, n_vars)  # n_vars changes when rate slack is added

        # Fast path: same (n_con, capacity) → guaranteed same sparsity pattern
        # Reuse the OSQP workspace and update numeric data. This still refreshes
        # CSC data arrays from current dense P/A values; it avoids OSQP setup, not
        # all sparse assembly cost.
        if self._prob is not None and csp_key == self._csp_key:
            try:
                # Only update data values, skip full CSC conversion
                # Use cached CSC matrices if available, otherwise create and cache
                if self._P_csc_cached is None or self._A_csc_cached is None:
                    A_data = A.copy()
                    A_data[:n_con, :n_vars] += 1e-30 * (A_data[:n_con, :n_vars] == 0)
                    self._P_csc_cached = triu(csc_matrix(P), format='csc')
                    self._A_csc_cached = csc_matrix(A_data)
                else:
                    # Update cached CSC data in-place
                    A_data = A.copy()
                    A_data[:n_con, :n_vars] += 1e-30 * (A_data[:n_con, :n_vars] == 0)
                    self._P_csc_cached.data[:] = triu(csc_matrix(P), format='csc').data
                    self._A_csc_cached.data[:] = csc_matrix(A_data).data

                t_update = _time.perf_counter()
                self._prob.update(
                    Px=self._P_csc_cached.data,
                    Ax=self._A_csc_cached.data,
                    q=q_vec,
                    l=l_vec,
                    u=u_vec,
                )
                self.last_qp_update_ms = (_time.perf_counter() - t_update) * 1000.0
                t_solve = _time.perf_counter()
                result = self._prob.solve()
                self.last_qp_solve_ms = (_time.perf_counter() - t_solve) * 1000.0
                self.last_qp_setup_ms = 0.0
                self.last_qp_reused = True
                self.update_count += 1
                return result
            except Exception:
                # Unexpected — fall through to fresh setup
                self._P_csc_cached = None
                self._A_csc_cached = None
                pass

        # Slow path: different (n_con, capacity) → full CSC conversion + pattern check
        # Invalidate cached CSC matrices since pattern changed
        self._P_csc_cached = None
        self._A_csc_cached = None

        A_sparse = A.copy()
        A_sparse[:n_con, :n_vars] += 1e-30 * (A_sparse[:n_con, :n_vars] == 0)
        P_csc = triu(csc_matrix(P), format='csc')
        A_csc = csc_matrix(A_sparse)

        # Check if we can reuse despite different csp_key (same pattern but different key)
        can_reuse = (
            self._prob is not None
            and n_con == self._last_n_con
            and self._P_csc_indptr is not None
            and self._P_csc_indices is not None
            and self._A_csc_indptr is not None
            and self._A_csc_indices is not None
            and np.array_equal(P_csc.indptr, self._P_csc_indptr)
            and np.array_equal(P_csc.indices, self._P_csc_indices)
            and np.array_equal(A_csc.indptr, self._A_csc_indptr)
            and np.array_equal(A_csc.indices, self._A_csc_indices)
        )

        if can_reuse:
            try:
                t_update = _time.perf_counter()
                self._prob.update(
                    Px=P_csc.data,
                    Ax=A_csc.data,
                    q=q_vec,
                    l=l_vec,
                    u=u_vec,
                )
                self.last_qp_update_ms = (_time.perf_counter() - t_update) * 1000.0
                t_solve = _time.perf_counter()
                result = self._prob.solve()
                self.last_qp_solve_ms = (_time.perf_counter() - t_solve) * 1000.0
                self.last_qp_setup_ms = 0.0
                self.last_qp_reused = True
                self.update_count += 1
                return result
            except Exception as e:
                _logger.debug('OSQP solve exception: %s: %s', type(e).__name__, e)

        # Fresh setup
        t_setup = _time.perf_counter()
        prob = self._osqp.OSQP()
        prob.setup(
            P=P_csc,
            q=q_vec,
            A=A_csc,
            l=l_vec,
            u=u_vec,
            eps_abs=1e-5,
            eps_rel=1e-5,
            max_iter=4000,
            polish=False,
            verbose=False,
            warm_start=True,
        )
        self.last_qp_setup_ms = (_time.perf_counter() - t_setup) * 1000.0
        self.setup_count += 1
        self._prob = prob
        self._last_n_con = n_con
        self._csp_key = csp_key
        self._P_csc_indptr = P_csc.indptr.copy()
        self._P_csc_indices = P_csc.indices.copy()
        self._A_csc_indptr = A_csc.indptr.copy()
        self._A_csc_indices = A_csc.indices.copy()
        self.last_qp_update_ms = 0.0
        self.last_qp_reused = False
        t_solve = _time.perf_counter()
        result = prob.solve()
        self.last_qp_solve_ms = (_time.perf_counter() - t_solve) * 1000.0
        return result

    def solve(self, u_nom: np.ndarray,
              constraints: List[CbfConstraint],
              dq_max: np.ndarray,
              w_pos: float = 1.0,
              w_orient: float = 1.0,
              w_joint: float = 0.1,
              J_pos: Optional[np.ndarray] = None,
              u_lower: Optional[np.ndarray] = None,
              u_upper: Optional[np.ndarray] = None,
              slack_penalty: float = 1e4,
              u_safe_prev: Optional[np.ndarray] = None,
              temporal_lambda: float = 0.0,
              temporal_wu: Optional[np.ndarray] = None,
              rate_limit_du_max: Optional[np.ndarray] = None,
              rate_limit_penalty: float = 1e3,
              ) -> Tuple[np.ndarray, bool, float]:
        """
        求解弹性 OSCBF QP (6-DOF 版本)，带时序近端稳定化 + soft rate limit。

        变量: x = [u (9,); s_cbf (n_pad,); s_rate (2,)]  其中 s >= 0

        目标函数:
          min  0.5*u^T*P_task*u + q_task^T*u
             + λ/2 ||W_u (u - u_safe_prev)||²
             + ρ_cbf * ||s_cbf||²
             + ρ_rate * ||s_rate||²

        约束:
          G*u - s_cbf <= h              (弹性 CBF 约束)
          u_lower <= u <= u_upper       (速度盒)
          u >= u_prev - du_max*dt - s_rate[0]  (soft rate 下界)
          u <= u_prev + du_max*dt + s_rate[1]  (soft rate 上界)
          s_cbf >= 0, s_rate >= 0

        Parameters
        ----------
        u_safe_prev : np.ndarray, optional
            上一帧实际发送的 u_safe，用于 temporal proximal term。
        temporal_lambda : float
            时序近端项权重 λ。0 = 禁用 (向后兼容)。
        temporal_wu : np.ndarray, optional
            每关节权重向量 W_u (对角)。None = 全 1。
        rate_limit_du_max : np.ndarray, optional
            每关节最大速度变化率 (rad/s per step)。None = 禁用 soft rate limit。
        rate_limit_penalty : float
            Rate limit slack 惩罚权重。默认 1e3。

        返回: (u_safe, success, solve_time_ms)
        """
        import time
        t0 = time.perf_counter()

        n_joints = len(u_nom)
        velocity_lower = -np.asarray(dq_max, dtype=float)
        velocity_upper = np.asarray(dq_max, dtype=float)
        if u_lower is not None:
            velocity_lower = np.maximum(velocity_lower, np.asarray(u_lower, dtype=float).reshape(n_joints))
        if u_upper is not None:
            velocity_upper = np.minimum(velocity_upper, np.asarray(u_upper, dtype=float).reshape(n_joints))
        invalid_bounds = velocity_lower > velocity_upper
        if np.any(invalid_bounds):
            center = np.clip(np.asarray(u_nom, dtype=float).reshape(n_joints),
                             -np.asarray(dq_max, dtype=float),
                             np.asarray(dq_max, dtype=float))
            velocity_lower[invalid_bounds] = center[invalid_bounds]
            velocity_upper[invalid_bounds] = center[invalid_bounds]
        active = [c for c in constraints if c.active]
        n_con = len(active)

        # ---- Soft rate limit: 计算 rate 约束 ----
        use_rate_limit = (rate_limit_du_max is not None and u_safe_prev is not None
                          and np.any(np.asarray(rate_limit_du_max) > 0))
        rate_lower = None
        rate_upper = None
        n_rate = 0
        if use_rate_limit:
            du_max = np.asarray(rate_limit_du_max, dtype=float).reshape(n_joints)
            u_prev = np.asarray(u_safe_prev, dtype=float).reshape(n_joints)
            rate_lower = u_prev - du_max
            rate_upper = u_prev + du_max
            # 只在 rate 约束比 velocity 约束更紧时才激活
            rate_active = np.any(rate_lower > velocity_lower + 1e-6) or \
                          np.any(rate_upper < velocity_upper - 1e-6)
            if rate_active:
                n_rate = 2  # s_rate_lower, s_rate_upper
            else:
                use_rate_limit = False

        # Rate limit active check complete
        if n_con == 0 and not use_rate_limit:
            # 无约束: 带 temporal proximal 的解析解
            # min 0.5||u-u_nom||² + λ/2||Wu(u-u_prev)||²
            # 解: u_i = (u_nom_i + λ·wu_i²·u_prev_i) / (1 + λ·wu_i²)
            if temporal_lambda > 0.0 and u_safe_prev is not None:
                wu = np.asarray(temporal_wu, dtype=float).reshape(n_joints) if temporal_wu is not None else np.ones(n_joints)
                scale = temporal_lambda * (wu ** 2)
                u = (u_nom + scale * np.asarray(u_safe_prev, dtype=float).reshape(n_joints)) / (1.0 + scale)
                u = np.clip(u, velocity_lower, velocity_upper)
                self.last_status = "unconstrained_proximal"
            else:
                u = np.clip(u_nom, velocity_lower, velocity_upper)
                self.last_status = "unconstrained"
            self.last_max_violation = 0.0
            self.last_slack_max = 0.0
            return u, True, 0.0

        # ---- 构建 QP 矩阵 ----
        # 变量顺序: [u_0..u_8, s_cbf_0..s_cbf_{n_pad-1}, s_rate_0..s_rate_1]
        n_pad = self._current_capacity
        n_vars = n_joints + n_pad + n_rate

        # P 矩阵: OSCBF 任务一致性版本 (带缓存)
        # P = N^T @ W_joint^2 @ N + J^T @ W_task^2 @ J
        # W_task >> W_joint 时, QP 优先在零空间中修正, 末端轨迹几乎不变
        if J_pos is not None:
            # 检查是否可复用上一步的 P_u (J_full 变化 < 阈值)
            w_cur = (w_pos, w_orient, w_joint)
            reuse_p = False
            if (self._prev_J_full is not None and self._prev_P_u is not None
                    and self._prev_w == w_cur
                    and self._prev_J_full.shape == J_pos.shape):
                diff_norm = np.linalg.norm(J_pos - self._prev_J_full, 'fro')
                ref_norm = max(np.linalg.norm(self._prev_J_full, 'fro'), 1e-10)
                if diff_norm / ref_norm < self._p_cache_rel_threshold:
                    reuse_p = True

            if reuse_p:
                P_u = self._prev_P_u
            else:
                # 阻尼伪逆 + 零空间投影
                # NOTE: λ_qp=1e-8 独立于标称控制器的 λ_nom=1e-3.
                # λ_qp 仅用于构造 OSCBF 任务一致性 P 矩阵和零空间投影 N.
                # 小 λ 保证 N ≈ I - J^+ J 是近乎精确的正交投影算子,
                # 从而 P_u @ u_nom ≡ J^T W_task^2 v_task + w_joint^2 N u_null (机器精度等价).
                damping = 1e-8
                n_task = J_pos.shape[0]
                if n_task == 6:
                    # 6D 雅可比 — pre-compute J @ J.T once, reuse for J_hash and N_null
                    W_task_diag = np.array([w_pos, w_pos, w_pos, w_orient, w_orient, w_orient])
                    W_task_sq = np.diag(W_task_diag ** 2)
                    JJt = J_pos @ J_pos.T + damping * np.eye(6)
                    JJt_inv = np.linalg.inv(JJt)
                    J_hash = J_pos.T @ JJt_inv
                else:
                    # 3D 雅可比
                    W_task_sq = w_pos**2 * np.eye(n_task)
                    JJt = J_pos @ J_pos.T + damping * np.eye(n_task)
                    JJt_inv = np.linalg.inv(JJt)
                    J_hash = J_pos.T @ JJt_inv

                N_null = np.eye(n_joints) - J_hash @ J_pos  # 零空间投影

                # OSCBF 任务一致性 P 矩阵
                P_u = N_null.T @ (w_joint**2 * np.eye(n_joints)) @ N_null \
                    + J_pos.T @ W_task_sq @ J_pos
                P_u = 0.5 * (P_u + P_u.T)  # 对称化
                self._prev_J_full = J_pos.copy()
                self._prev_P_u = P_u
                self._prev_w = w_cur
        else:
            P_u = np.eye(n_joints)

        # ---- Temporal proximal term: λ/2 ||W_u (u - u_safe_prev)||² ----
        # 展开后: P_new = P_task + λ * diag(wu²)
        #          q_new = -P_task @ u_nom - λ * diag(wu²) @ u_safe_prev
        # 这惩罚当前 u 偏离上一帧实际发送的 u_safe_prev, 抑制跨时刻跳变。
        _temporal_wu_sq = None  # 缓存, 下面 q_vec 也要用
        P_task = P_u  # 保存不含 temporal 的原始 P, 用于 q 向量
        if temporal_lambda > 0.0 and u_safe_prev is not None:
            if temporal_wu is not None:
                wu = np.asarray(temporal_wu, dtype=float).reshape(n_joints)
            else:
                wu = np.ones(n_joints)
            _temporal_wu_sq = temporal_lambda * (wu ** 2)
            P_u = P_u + np.diag(_temporal_wu_sq)
            P_u = 0.5 * (P_u + P_u.T)  # 保持对称

        # Pre-allocate padded arrays on first call
        # 需要重新分配: n_joints 变化, capacity 不足, 或 rate slack 从 0→2
        _need_alloc = (self._P_pad is None or self._n_joints != n_joints
                       or n_con > self._current_capacity
                       or (use_rate_limit and not hasattr(self, '_has_rate_slack'))
                       or (not use_rate_limit and getattr(self, '_has_rate_slack', False)))
        if _need_alloc:
            self._n_joints = n_joints
            capacity = max(self.max_constraints, n_con)
            self._current_capacity = capacity
            self._has_rate_slack = use_rate_limit
            n_rate_alloc = 2 if use_rate_limit else 0
            self._prob = None  # invalidate cached OSQP problem
            self._last_n_con = -1
            self._csp_key = None
            self._P_csc_indptr = None
            self._P_csc_indices = None
            self._A_csc_indptr = None
            self._A_csc_indices = None
            n_vars_cap = n_joints + capacity + n_rate_alloc
            # Rows: CBF(capacity) + rate_limit(2*n_joints if enabled) + box(n_vars_cap)
            n_rate_rows = 2 * n_joints if use_rate_limit else 0
            n_rows_cap = capacity + n_rate_rows + n_vars_cap
            self._P_pad = np.zeros((n_vars_cap, n_vars_cap))
            self._q_pad = np.zeros(n_vars_cap)
            self._A_pad = np.zeros((n_rows_cap, n_vars_cap))
            # Box rows start after CBF + rate rows
            box_start = capacity + n_rate_rows
            # Box rows: identity for u
            self._A_pad[box_start:box_start + n_joints, :n_joints] = np.eye(n_joints)
            # Box rows: identity for s_cbf
            self._A_pad[box_start + n_joints:box_start + n_joints + capacity,
                        n_joints:n_joints + capacity] = np.eye(capacity)
            # Box rows: identity for s_rate
            if use_rate_limit:
                self._A_pad[box_start + n_joints + capacity:,
                            n_joints + capacity:] = np.eye(n_rate_alloc)
            self._l_pad = np.full(n_rows_cap, -np.inf)
            self._u_pad = np.zeros(n_rows_cap)
            # Pre-compute slack diagonal template
            self._slack_diag = np.zeros(capacity)
            n_pad = capacity

        P = self._P_pad
        q_vec = self._q_pad
        A = self._A_pad
        l_vec = self._l_pad
        u_vec = self._u_pad

        # Reset only the CBF bounds region (box constraints are persistent)
        l_vec[:n_pad] = -np.inf
        u_vec[:n_pad] = 0.0

        # Fill P: task-consistent block + slack diagonal (vectorized)
        P[:] = 0.0
        P[:n_joints, :n_joints] = P_u
        # Pre-compute slack diagonal: active constraints get penalty, padded get 0
        self._slack_diag[:n_con] = slack_penalty
        self._slack_diag[n_con:n_pad] = 0.0
        np.fill_diagonal(P[n_joints:n_joints + n_pad, n_joints:n_joints + n_pad],
                         self._slack_diag[:n_pad])
        # Rate slack penalty in P
        if use_rate_limit:
            P[n_joints + n_pad:, n_joints + n_pad:] = rate_limit_penalty * np.eye(n_rate)

        # q vector: q = -P_task @ u_nom - λ * diag(wu²) @ u_safe_prev
        q_vec[:n_joints] = -P_task @ u_nom
        if _temporal_wu_sq is not None and u_safe_prev is not None:
            q_vec[:n_joints] -= _temporal_wu_sq * np.asarray(u_safe_prev, dtype=float).reshape(n_joints)
        q_vec[n_joints:] = 0.0

        # Constraint rows: [G, -I_pad] for active (vectorized)
        if n_con > 0:
            G_stack = np.array([c.G_row for c in active])  # (n_con, n_joints)
            A[:n_con, :n_joints] = G_stack
            A[:n_con, n_joints:n_joints + n_con] = -np.eye(n_con)

        # Inequality bounds: G*u - s <= h  (active), 0 <= 0 (padded)
        h_bounds = np.array([c.h_bound for c in active])
        u_vec[:n_con] = h_bounds
        # Padded rows: already zero from allocation, no reset needed

        # Rate limit constraint rows (在 CBF 约束之后, box 约束之前)
        if use_rate_limit:
            rate_row_base = n_pad  # rate 约束从第 n_pad 行开始
            # Row 0: -I @ u + [0..0 I(2)] @ s_rate >= -rate_upper
            #   即 -rate_upper <= -u + s_rate[0] <= inf
            A[rate_row_base, :n_joints] = -np.eye(n_joints)[0]  # placeholder, 填充下面
            # 实际填充: -I(n_joints) 在 u 列
            A[rate_row_base:rate_row_base + n_joints, :n_joints] = -np.eye(n_joints)
            # s_rate[0] 列: 每行 1 (所有关节共享同一个 s_rate_lower)
            A[rate_row_base:rate_row_base + n_joints,
              n_joints + n_pad] = 1.0
            # Row n_joints: I @ u + [0..0 I(2)] @ s_rate >= rate_lower
            #   即 rate_lower <= u + s_rate[1] <= inf
            A[rate_row_base + n_joints:rate_row_base + 2 * n_joints,
              :n_joints] = np.eye(n_joints)
            A[rate_row_base + n_joints:rate_row_base + 2 * n_joints,
              n_joints + n_pad + 1] = 1.0
            # Bounds
            l_vec[rate_row_base:rate_row_base + n_joints] = -rate_upper
            u_vec[rate_row_base:rate_row_base + n_joints] = np.inf
            l_vec[rate_row_base + n_joints:rate_row_base + 2 * n_joints] = rate_lower
            u_vec[rate_row_base + n_joints:rate_row_base + 2 * n_joints] = np.inf

        # Box constraints: velocity_lower <= u <= velocity_upper, 0 <= s_cbf <= slack_upper, 0 <= s_rate
        n_rate_rows = 2 * n_joints if use_rate_limit else 0
        box_start = n_pad + n_rate_rows
        l_vec[box_start:box_start + n_joints] = velocity_lower
        u_vec[box_start:box_start + n_joints] = velocity_upper
        # CBF slack: 0 <= s_cbf <= slack_upper_bound
        l_vec[box_start + n_joints:box_start + n_joints + n_pad] = 0.0
        u_vec[box_start + n_joints:box_start + n_joints + n_pad] = self._slack_upper_bound
        # Rate slack: 0 <= s_rate (无上界, 由目标函数惩罚)
        if use_rate_limit:
            l_vec[box_start + n_joints + n_pad:] = 0.0
            u_vec[box_start + n_joints + n_pad:] = 1e20  # effectively unbounded

        # ---- 求解 ----
        if self._available:
            try:
                res = self._solve_osqp(P, q_vec, A, l_vec, u_vec, n_con, n_pad)
                self.last_status = str(res.info.status)
                self._record_osqp_diagnostics(res)
                if res.info.status in ['solved', 'solved inaccurate']:
                    u = np.clip(res.x[:n_joints], velocity_lower, velocity_upper)
                    self._record_solution_diagnostics_padded(res.x, active, n_joints, n_con)
                    dt_ms = (time.perf_counter() - t0) * 1000
                    return u, True, dt_ms
                if self.last_qp_reused:
                    self._prob = None
                    self._last_n_con = -1
                    self._csp_key = None
                    self._P_csc_indptr = None
                    self._P_csc_indices = None
                    self._A_csc_indptr = None
                    self._A_csc_indices = None
                    res = self._solve_osqp(P, q_vec, A, l_vec, u_vec, n_con, n_pad)
                    self.last_status = str(res.info.status)
                    self._record_osqp_diagnostics(res)
                    if res.info.status in ['solved', 'solved inaccurate']:
                        u = np.clip(res.x[:n_joints], velocity_lower, velocity_upper)
                        self._record_solution_diagnostics_padded(res.x, active, n_joints, n_con)
                        dt_ms = (time.perf_counter() - t0) * 1000
                        return u, True, dt_ms
            except Exception as e:
                _logger.debug('OSQP solve exception: %s: %s', type(e).__name__, e)

        # ---- 回退: 迭代投影 (POCS) 直到收敛 ----
        self.fallback_count += 1
        self.last_status = "fallback"
        u = u_nom.copy()
        for _ in range(self._fallback_max_iter):
            max_viol = 0.0
            for c in active:
                g = c.G_row
                v = g @ u - c.h_bound
                if v > 0:
                    g_norm_sq = np.dot(g, g)
                    if g_norm_sq > 1e-12:
                        u = u - v * g / g_norm_sq
                        if v > max_viol:
                            max_viol = v
            if max_viol < 1e-6:
                break
        u = np.clip(u, velocity_lower, velocity_upper)
        if active:
            self.last_max_violation = float(max(
                0.0,
                max(float(c.G_row @ u - c.h_bound) for c in active)
            ))
        else:
            self.last_max_violation = 0.0
        self.last_slack_max = 0.0

        # 诊断: slack 用量过大时 warning
        if self.last_slack_max > 0.5 * self._slack_upper_bound:
            _logger.warning(
                'QP slack usage high: %.3f > %.3f (50%% of upper bound)',
                self.last_slack_max, 0.5 * self._slack_upper_bound)

        dt_ms = (time.perf_counter() - t0) * 1000
        return u, False, dt_ms  # fallback (顺序投影) — 与 OSQP 成功区分

    def _record_osqp_diagnostics(self, res) -> None:
        """Record solver-level diagnostics from OSQP result."""
        self.last_iter = int(getattr(res.info, 'iter', 0))
        self.last_dual_residual = float(getattr(res.info, 'dual_res', 0.0))
        self.last_objective = float(getattr(res.info, 'obj_val', 0.0))

    def _record_solution_diagnostics_padded(self, x: np.ndarray, active: List[CbfConstraint],
                                            n_joints: int, n_con: int) -> None:
        """Record diagnostics from padded solution vector."""
        if x is None:
            self.last_max_violation = float("inf")
            self.last_slack_max = float("inf")
            self.last_rate_slack_max = 0.0
            return
        u = x[:n_joints]
        if active:
            self.last_max_violation = float(max(
                0.0,
                max(float(c.G_row @ u - c.h_bound) for c in active)
            ))
            slack = x[n_joints:n_joints + n_con]
            self.last_slack_max = float(np.max(slack)) if n_con > 0 else 0.0
        else:
            self.last_max_violation = 0.0
            self.last_slack_max = 0.0
        # Rate slack diagnostics
        n_pad = self._current_capacity
        n_rate = len(x) - n_joints - n_pad
        if n_rate > 0:
            rate_slack = x[n_joints + n_pad:]
            self.last_rate_slack_max = float(np.max(rate_slack))
        else:
            self.last_rate_slack_max = 0.0
