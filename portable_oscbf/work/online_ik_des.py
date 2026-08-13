"""Online IK q_des calculator — manipulability-based selection.

Computes q_des (nullspace posture reference) online at each control step
by solving 6D IK with a few seeds and selecting the solution with the
highest manipulability score.

Replaces the offline profile approach (DP + quintic interpolation) for
generating the nullspace posture reference.

Usage:
    online_ik = OnlineIKDes()
    q_des = online_ik.compute(target_pos, target_R, q_current, kin)
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass

import numpy as np

from newaxis.ik_start_selection import normalized_6d_task_sigma_min

logger = logging.getLogger(__name__)

_NUM_JOINTS = 9
_DEFAULT_N_PERTURBATIONS = 3
_DEFAULT_PERTURBATION_RAD = 0.08
_DEFAULT_MAX_ITER = 100
_DEFAULT_POS_TOL = 1e-4
_DEFAULT_ROT_TOL = np.deg2rad(0.1)
_MIN_CALL_INTERVAL_S = 0.200  # 200ms minimum between calls (~5 Hz, every 20 steps)
_MAX_Q_DES_CHANGE_RAD = 0.02  # Max change per update to avoid rate-slip HARD_STOP


@dataclass
class OnlineIKDesConfig:
    """Configuration for online IK q_des calculator."""
    n_perturbations: int = _DEFAULT_N_PERTURBATIONS
    perturbation_rad: float = _DEFAULT_PERTURBATION_RAD
    max_iter: int = _DEFAULT_MAX_ITER
    pos_tol_m: float = _DEFAULT_POS_TOL
    rot_tol_rad: float = _DEFAULT_ROT_TOL
    min_call_interval_s: float = _MIN_CALL_INTERVAL_S
    max_q_des_change_rad: float = _MAX_Q_DES_CHANGE_RAD


class OnlineIKDes:
    """Online IK q_des calculator with manipulability-based selection.

    Each call solves 6D IK with (1 + n_perturbations) seeds:
    - Seed 0: current q (best guess, fast convergence)
    - Seeds 1..n: random perturbations of current q

    The solution with the highest normalized 6D task manipulability
    (sigma_min) is selected as q_des.
    """

    def __init__(self, config: OnlineIKDesConfig | None = None):
        self._config = config or OnlineIKDesConfig()
        self._last_call_time = 0.0
        self._last_q_des: np.ndarray | None = None
        self._last_dq_des_dell: np.ndarray | None = None
        self._call_count = 0
        self._ik_fail_count = 0
        self._rng = np.random.RandomState(42)

    def compute(
        self,
        target_pos: np.ndarray,
        target_R: np.ndarray,
        q_current: np.ndarray,
        kin,
        feedrate_m_s: float = 0.030,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Compute q_des and dq_des_dell via online IK with manipulability selection.

        Args:
            target_pos: Target end-effector position (3,).
            target_R: Target end-effector rotation matrix (3, 3).
            q_current: Current joint configuration (9,).
            kin: NineaxisKinematics instance.
            feedrate_m_s: Current path feedrate for dq_des_dell computation.

        Returns:
            (q_des, dq_des_dell): Best IK solution and its arc-length derivative.
            dq_des_dell is zero when no previous q_des is available.
        """
        now = time.perf_counter()
        cfg = self._config

        # Rate limit: skip if called too soon (return last result)
        if (self._last_q_des is not None
                and (now - self._last_call_time) < cfg.min_call_interval_s):
            return self._last_q_des.copy(), self._last_dq_des_dell.copy()

        self._last_call_time = now
        self._call_count += 1
        logger.debug(f'OnlineIKDes.compute called #{self._call_count}')

        q_min = kin.joint_limits.q_min
        q_max = kin.joint_limits.q_max
        dq_max = q_max - q_min  # Use range as velocity proxy for scoring

        # Build seeds: previous q_des (continuity) + current q + perturbations
        # Using previous q_des as primary seed ensures smooth nullspace motion.
        seed_q = self._last_q_des if self._last_q_des is not None else q_current
        seeds = [seed_q.copy(), q_current.copy()]
        for _ in range(cfg.n_perturbations):
            perturb = self._rng.randn(_NUM_JOINTS) * cfg.perturbation_rad
            seed = np.clip(seed_q + perturb, q_min, q_max)
            seeds.append(seed)

        best_q = None
        best_sigma = -1.0

        for seed in seeds:
            try:
                q_sol = kin.ik(
                    target_pos, target_R=target_R, q_init=seed,
                    max_iter=cfg.max_iter,
                    tol_pos=cfg.pos_tol_m,
                    tol_rot=cfg.rot_tol_rad,
                )
            except Exception:
                continue

            if q_sol is None:
                continue
            if not np.all(np.isfinite(q_sol)):
                continue

            # Check joint limits with margin
            margin = min(
                float(np.min(q_sol - q_min)),
                float(np.max(q_max - q_sol)))
            if margin < 0:
                continue

            # Score by manipulability
            try:
                J = kin.compute_full_jacobian(q_sol)
                sigma = normalized_6d_task_sigma_min(J, dq_max)
            except (ValueError, np.linalg.LinAlgError):
                sigma = 0.0

            if sigma > best_sigma:
                best_sigma = sigma
                best_q = q_sol.copy()

        if best_q is None:
            self._ik_fail_count += 1
            # Fallback: keep current q (no nullspace motion)
            if self._last_q_des is not None:
                return self._last_q_des.copy(), self._last_dq_des_dell.copy()
            dq_des_dell = np.zeros(_NUM_JOINTS)
            self._last_dq_des_dell = dq_des_dell
            return q_current.copy(), dq_des_dell

        # Clip q_des change to avoid large nullspace motions that would
        # trigger rate-slip HARD_STOP in the CBF-QP.
        if self._last_q_des is not None:
            delta = best_q - self._last_q_des
            max_change = cfg.max_q_des_change_rad
            delta = np.clip(delta, -max_change, max_change)
            best_q = self._last_q_des + delta

        # Compute dq_des_dell (derivative of q_des w.r.t. arc length).
        # dq_des_dell = dq_des_dt / (dl/dt) = dq_des_dt / feedrate
        dt = now - self._last_call_time if self._last_call_time > 0 else cfg.min_call_interval_s
        dt = max(dt, 1e-6)
        feedrate = max(feedrate_m_s, 1e-6)
        if self._last_q_des is not None:
            dq_des_dt = (best_q - self._last_q_des) / dt
            dq_des_dell = dq_des_dt / feedrate
        else:
            dq_des_dell = np.zeros(_NUM_JOINTS)

        self._last_q_des = best_q
        self._last_dq_des_dell = dq_des_dell
        return best_q, dq_des_dell

    @property
    def stats(self) -> dict:
        """Return diagnostic statistics."""
        return {
            'call_count': self._call_count,
            'ik_fail_count': self._ik_fail_count,
            'fail_rate': (self._ik_fail_count / max(self._call_count, 1)),
        }
