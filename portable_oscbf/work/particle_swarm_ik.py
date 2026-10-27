from dataclasses import dataclass
from typing import Callable

import numpy as np
from pymoo.algorithms.soo.nonconvex.pso import PSO
from pymoo.core.problem import Problem
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class ParticleSwarmResult:
    q: np.ndarray
    success: bool
    position_error_m: float
    rotation_error_rad: float
    iterations: int
    evaluations: int
    restarts: int
    convergence_curve: tuple[float, ...]


class _PoseProblem(Problem):
    def __init__(
        self,
        forward_poses: Callable[[np.ndarray], np.ndarray],
        target_pos: np.ndarray,
        target_rotation: np.ndarray | None,
        lower: np.ndarray,
        upper: np.ndarray,
        tol_pos: float,
        tol_rot: float,
    ):
        super().__init__(n_var=len(lower), n_obj=1, xl=0.0, xu=1.0)
        self.forward_poses = forward_poses
        self.target_pos = target_pos
        self.target_rotation = target_rotation
        self.lower = lower
        self.width = upper - lower
        self.tol_pos = tol_pos
        self.tol_rot = tol_rot
        self.best_cost = np.inf
        self.best_q = None
        self.best_position_error = np.inf
        self.best_rotation_error = np.inf
        self.solution = None
        self.evaluations = 0

    def _evaluate(self, x, out, *args, **kwargs):
        if not np.all(np.isfinite(x)) or np.any(x < 0.0) or np.any(x > 1.0):
            raise ValueError("PSO 粒子必须位于归一化关节范围 [0, 1]")
        q = self.lower + x * self.width
        transforms = self.forward_poses(q)
        position_error = np.linalg.norm(transforms[:, :3, 3] - self.target_pos, axis=1)
        rotation_error = np.zeros(len(x))
        if self.target_rotation is not None:
            relative = self.target_rotation @ transforms[:, :3, :3].swapaxes(1, 2)
            rotation_error = Rotation.from_matrix(relative).magnitude()
        # 搜索使用归一化加权误差，通过条件分别检查两个物理量。
        cost = (position_error / self.tol_pos) ** 2 + 25.0 * (rotation_error / self.tol_rot) ** 2
        if not np.all(np.isfinite(cost)):
            raise ValueError("PSO 位姿误差必须为有限数值")
        out["F"] = cost
        self.evaluations += len(x)
        best = int(np.argmin(cost))
        if cost[best] < self.best_cost:
            self.best_cost = float(cost[best])
            self.best_q = q[best].copy()
            self.best_position_error = float(position_error[best])
            self.best_rotation_error = float(rotation_error[best])
        accepted = (position_error <= self.tol_pos) & (rotation_error <= self.tol_rot)
        if np.any(accepted):
            index = int(np.argmin(np.where(accepted, cost, np.inf)))
            self.solution = (
                q[index].copy(), float(position_error[index]), float(rotation_error[index])
            )


def solve_particle_swarm(
    forward_poses: Callable[[np.ndarray], np.ndarray],
    target_pos: np.ndarray,
    target_rotation: np.ndarray | None,
    q_init: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    *,
    max_iter: int = 1500,
    tol_pos: float = 1e-4,
    tol_rot: float = 1e-3,
    swarm_size: int = 128,
    seed: int = 0,
) -> ParticleSwarmResult:
    """使用三个 PSO 子群求解；内部长度为 m，转动关节为 rad。"""
    target_pos = np.asarray(target_pos, dtype=float)
    q_init = np.asarray(q_init, dtype=float)
    lower, upper = np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)
    if target_pos.shape != (3,) or not np.all(np.isfinite(target_pos)):
        raise ValueError("target_pos 必须为有限的三维位置，单位 m")
    if (lower.ndim != 1 or upper.shape != lower.shape
            or not np.all(np.isfinite([lower, upper])) or np.any(lower >= upper)):
        raise ValueError("lower、upper 必须为有限且严格递增的关节限位")
    if (q_init.shape != lower.shape or not np.all(np.isfinite(q_init))
            or np.any(q_init < lower) or np.any(q_init > upper)):
        raise ValueError("q_init 必须满足关节限位，J1 使用 m，转动关节使用 rad")
    if target_rotation is not None:
        target_rotation = np.asarray(target_rotation, dtype=float)
        if target_rotation.shape != (3, 3) or not np.all(np.isfinite(target_rotation)):
            raise ValueError("target_rotation 必须为有限的 3×3 旋转矩阵")
        if (not np.allclose(target_rotation.T @ target_rotation, np.eye(3), atol=1e-8, rtol=0)
                or not np.isclose(np.linalg.det(target_rotation), 1.0, atol=1e-8, rtol=0)):
            raise ValueError("target_rotation 必须满足正交性且行列式为 1")
    if isinstance(max_iter, bool) or not isinstance(max_iter, (int, np.integer)) or max_iter < 1:
        raise ValueError("max_iter 必须为正整数")
    if isinstance(swarm_size, bool) or not isinstance(swarm_size, (int, np.integer)) or swarm_size < 8:
        raise ValueError("swarm_size 必须为不小于 8 的整数")
    if isinstance(seed, bool) or not isinstance(seed, (int, np.integer)) or seed < 0:
        raise ValueError("seed 必须为非负整数")
    if not np.isfinite(tol_pos) or tol_pos <= 0 or not np.isfinite(tol_rot) or tol_rot <= 0:
        raise ValueError("tol_pos、tol_rot 必须为有限正数")

    problem = _PoseProblem(forward_poses, target_pos, target_rotation, lower, upper, tol_pos, tol_rot)
    normalized_seed = (q_init - lower) / (upper - lower)
    problem.evaluate(normalized_seed[None, :])
    history = [problem.best_cost]
    iterations = 0
    restarts = 0
    if problem.solution is None:
        generators = np.random.SeedSequence(seed).spawn(3)
        random_generators = [np.random.default_rng(sequence) for sequence in generators]

        def create_swarm(index, restart):
            rng = random_generators[index]
            if index == 0:
                center = (problem.best_q - lower) / (upper - lower) if restart else normalized_seed
                spread = 0.1 if restart else 0.03
                initial = np.clip(center + rng.normal(0.0, spread, (swarm_size, len(lower))), 0.0, 1.0)
                initial[0] = center
            else:
                initial = rng.uniform(0.0, 1.0, (swarm_size, len(lower)))
                if not restart:
                    initial[0] = normalized_seed
            swarm = PSO(
                pop_size=swarm_size, sampling=initial, adaptive=False,
                w=0.7298, c1=1.49618, c2=1.49618,
                initial_velocity="zero", max_velocity_rate=0.2, pertube_best=True,
            )
            swarm.setup(problem, termination=("n_gen", max_iter), seed=int(rng.integers(0, 2**32)), verbose=False)
            return swarm

        swarms = [create_swarm(index, False) for index in range(3)]
        checkpoint_costs = np.full(3, np.inf)
        for iteration in range(max_iter):
            for index, swarm in enumerate(swarms):
                swarm.next()
                if problem.solution is not None:
                    break
                if (iteration + 1) % 100 == 0:
                    current_cost = float(swarm.opt[0].F[0])
                    previous_cost = checkpoint_costs[index]
                    if np.isfinite(previous_cost) and previous_cost - current_cost < 0.005 * previous_cost:
                        swarms[index] = create_swarm(index, True)
                        checkpoint_costs[index] = np.inf
                        restarts += 1
                    else:
                        checkpoint_costs[index] = current_cost
            iterations = iteration + 1
            history.append(problem.best_cost)
            if problem.solution is not None:
                break

    if problem.solution is not None:
        q, position_error, rotation_error = problem.solution
        success = True
    else:
        q = problem.best_q
        position_error = problem.best_position_error
        rotation_error = problem.best_rotation_error
        success = False
    return ParticleSwarmResult(
        q=q.copy(), success=success, position_error_m=position_error,
        rotation_error_rad=rotation_error, iterations=iterations,
        evaluations=problem.evaluations, restarts=restarts, convergence_curve=tuple(history),
    )
