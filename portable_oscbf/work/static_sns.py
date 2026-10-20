from __future__ import annotations

from dataclasses import dataclass

import numpy as np


# 依据 Amirhossein Kazemipour 的 Generalized-SNS，CC BY 4.0。
# 来源、版本、数值参数与接口说明见 docs/static_obstacle_experiments.md。
def damped_pseudoinverse(matrix):
    matrix = np.asarray(matrix, dtype=float)
    U, singular, Vh = np.linalg.svd(matrix, full_matrices=False)
    if not len(singular):
        return np.zeros(matrix.T.shape)
    if singular[-1] > .1:
        inverse = 1 / singular
    else:
        damping_squared = (1 - (singular[-1] / .1) ** 2) * .001 ** 2
        inverse = np.divide(
            singular, singular ** 2 + damping_squared,
            out=np.zeros_like(singular), where=singular > 1e-10,
        )
    return (Vh.T * inverse) @ U.T


@dataclass(frozen=True)
class SnsSolution:
    velocity: np.ndarray
    task_scale: float
    iterations: int
    saturated: tuple[int, ...]
    task_residual: float
    hard_violation: float


def _feasible_scale(direction, bias, lower, upper, available):
    low, high, critical = 0., 1., None
    for index in np.flatnonzero(available):
        slope, offset = direction[index], bias[index]
        if abs(slope) <= 1e-12:
            if offset < lower[index] - 1e-10 or offset > upper[index] + 1e-10:
                return 0., index
            continue
        first, last = sorted(((lower[index] - offset) / slope, (upper[index] - offset) / slope))
        low = max(low, first)
        if last < high:
            high, critical = last, index
    if low > high or high < 0:
        return 0., critical
    return float(high), critical


def generalized_sns(jacobian, desired, matrix, lower, upper) -> SnsSolution:
    J, desired, A, lower, upper = [np.asarray(value, dtype=float) for value in (
        jacobian, desired, matrix, lower, upper,
    )]
    if (J.ndim != 2 or A.ndim != 2 or J.shape[1] != A.shape[1]
            or desired.shape != (J.shape[0],) or lower.shape != (len(A),)
            or upper.shape != (len(A),) or not len(A) or not J.shape[0]
            or not np.all(np.isfinite(np.concatenate([
                J.ravel(), desired, A.ravel(), lower, upper,
            ]))) or np.any(lower > upper)):
        raise ValueError("Generalized-SNS 输入无效或广义限制上下界顺序无效")
    identity = np.eye(J.shape[1])
    active = np.empty((0, J.shape[1]))
    saturated_values = np.empty(0)
    available = np.ones(len(A), dtype=bool)
    projection = identity.copy()
    best_velocity, best_scale = np.zeros(J.shape[1]), 0.
    selected = []
    for iteration in range(1, len(A) + 2):
        inverse = damped_pseudoinverse(J @ projection)
        direction = inverse @ desired
        bias = (identity - inverse @ J) @ np.linalg.pinv(active, rcond=1e-10) @ saturated_values
        velocity = direction + bias
        measured = A @ velocity
        violation = max(0., float(np.max(lower - measured)), float(np.max(measured - upper)))
        if violation <= 1e-10:
            best_velocity, best_scale = velocity, 1.
            break
        scale, critical = _feasible_scale(A @ direction, A @ bias, lower, upper, available)
        scaled = scale * direction + bias
        scaled_measured = A @ scaled
        if (scale > best_scale and np.all(scaled_measured >= lower - 1e-10)
                and np.all(scaled_measured <= upper + 1e-10)):
            best_velocity, best_scale = scaled, scale
        if critical is None:
            break
        selected.append(int(critical))
        available[critical] = False
        active = np.concatenate([active, A[critical:critical + 1]])
        saturated_values = np.r_[saturated_values, np.clip(measured[critical], lower[critical], upper[critical])]
        projection = identity - np.linalg.pinv(active, rcond=1e-10) @ active
        if np.any(np.linalg.svd(J @ projection, compute_uv=False) < 1e-6):
            break
    else:
        raise RuntimeError("Generalized-SNS 饱和迭代超出限制数量")
    measured = A @ best_velocity
    violation = max(0., float(np.max(lower - measured)), float(np.max(measured - upper)))
    if violation > 1e-9 or not np.all(np.isfinite(best_velocity)):
        raise RuntimeError("Generalized-SNS 结果不满足广义限制")
    return SnsSolution(
        best_velocity, best_scale, iteration, tuple(selected),
        float(np.linalg.norm(J @ best_velocity - best_scale * desired)), violation,
    )


def add_posture(solution, jacobian, posture, matrix, lower, upper):
    jacobian, posture = np.asarray(jacobian), np.asarray(posture)
    direction = (np.eye(jacobian.shape[1]) - np.linalg.pinv(jacobian, rcond=1e-10) @ jacobian) @ posture
    factor, _ = _feasible_scale(
        matrix @ direction, matrix @ solution.velocity, lower, upper, np.ones(len(matrix), dtype=bool),
    )
    velocity = solution.velocity + factor * direction
    measured = matrix @ velocity
    violation = max(0., float(np.max(lower - measured)), float(np.max(measured - upper)))
    if violation > 1e-9 or not np.all(np.isfinite(velocity)):
        raise RuntimeError("Generalized-SNS 构型辅助不满足广义限制")
    if np.linalg.norm(jacobian @ (velocity - solution.velocity)) > 1e-9:
        raise RuntimeError("Generalized-SNS 构型辅助改变了主任务")
    return SnsSolution(
        velocity, solution.task_scale, solution.iterations, solution.saturated,
        solution.task_residual, violation,
    )
