from __future__ import annotations

from dataclasses import dataclass

import jax
import jax.numpy as jnp
import numpy as np
from qpax import solve_qp

from work.qp_solver_health import terminal_qp_health
from work.static_collision import CollisionMeasurement, GeometryQueryError
from work.static_sns import generalized_sns, add_posture


class SolverFailure(RuntimeError):
    def __init__(self, message, problem, solver="qp"):
        super().__init__(message)
        self.problem = problem
        self.solver = solver


@dataclass(frozen=True)
class VelocityLimits:
    lower: np.ndarray
    upper: np.ndarray
    max_velocity: np.ndarray

    def __post_init__(self):
        for name in ("lower", "upper", "max_velocity"):
            value = np.asarray(getattr(self, name), dtype=float).copy()
            if value.shape != (9,) or not np.all(np.isfinite(value)):
                raise ValueError(f"{name} 必须为九维有限数组")
            value.setflags(write=False)
            object.__setattr__(self, name, value)
        if np.any(self.lower >= self.upper) or np.any(self.max_velocity <= 0):
            raise ValueError("关节范围必须具有正宽度，速度限制必须大于零")


@dataclass(frozen=True)
class VelocityConstraints:
    G: np.ndarray
    h: np.ndarray
    labels: tuple[tuple[str, str], ...]


@dataclass(frozen=True)
class VelocitySolution:
    velocity: np.ndarray
    feed_m_s: float
    solution: np.ndarray
    problem: dict[str, np.ndarray]
    kkt_residual: float | None
    hard_violation: float
    iterations: int
    solver: str = "qp"
    task_scale: float | None = None
    task_residual: float | None = None
    saturated: tuple[int, ...] = ()
    dual: np.ndarray | None = None
    reused_zero_feed: bool = False
    reused_problem: bool = False


def build_constraints(
        q, limits: VelocityLimits, measurements: tuple[CollisionMeasurement, ...],
        *, period_s: float, alpha: float, clearance_reserve_m: float = 0.) -> VelocityConstraints:
    q = np.asarray(q, dtype=float)
    if q.shape != (9,) or not np.all(np.isfinite(q)):
        raise ValueError("关节位置必须为九维有限数组")
    if np.any(q < limits.lower) or np.any(q > limits.upper):
        raise ValueError("关节位置超出允许范围")
    if not np.isfinite(period_s) or period_s <= 0 or not np.isfinite(alpha) or alpha <= 0:
        raise ValueError("控制周期与 alpha 必须为有限正数")
    if not np.isfinite(clearance_reserve_m) or clearance_reserve_m < 0:
        raise ValueError("控制间距余量必须为有限非负数")
    rows = [np.eye(9), -np.eye(9), np.eye(9), -np.eye(9)]
    position_rate = min(alpha, 1 / period_s)
    bounds = [
        limits.max_velocity, limits.max_velocity,
        position_rate * (limits.upper - q), position_rate * (q - limits.lower),
    ]
    labels = [
        (f"J{index + 1}", bound)
        for bound in ("velocity_upper", "velocity_lower", "position_upper", "position_lower")
        for index in range(9)
    ]
    for measurement in measurements:
        if (not measurement.distance.valid or measurement.gradient is None
                or not np.all(np.isfinite(measurement.gradient))):
            raise GeometryQueryError(f"{measurement.pair}: 距离梯度无效")
        margin = measurement.distance.distance_m - measurement.required_m
        if not np.isfinite(margin) or margin < 0:
            raise GeometryQueryError(f"{measurement.pair}: 实际间距不满足要求")
        rows.append(-measurement.gradient.reshape(1, 9))
        bounds.append(np.array([alpha * (margin - clearance_reserve_m)]))
        labels.append(measurement.pair)
    return VelocityConstraints(np.concatenate(rows), np.concatenate(bounds), tuple(labels))


def common_posture_velocity(
        q, limits: VelocityLimits, task_jacobian, measurements,
        *, speed_limit: float = .1, influence_m: float = .08,
        joint_margin_fraction: float = .05) -> np.ndarray:
    q, jacobian = np.asarray(q, dtype=float), np.asarray(task_jacobian, dtype=float)
    if (q.shape != (9,) or jacobian.shape != (5, 9)
            or not np.all(np.isfinite(q)) or not np.all(np.isfinite(jacobian))
            or not np.isfinite(speed_limit) or speed_limit <= 0
            or not np.isfinite(influence_m) or influence_m <= 0
            or not np.isfinite(joint_margin_fraction) or not 0 < joint_margin_fraction <= 1
            or np.any(q < limits.lower) or np.any(q > limits.upper)):
        raise ValueError("构型辅助参数无效")
    span = limits.upper - limits.lower
    normalized = 2 * (q - limits.lower) / span - 1
    gradient = -normalized / (span * (1 - normalized ** 2 + joint_margin_fraction ** 2))
    for measurement in measurements:
        if not measurement.distance.valid or measurement.gradient is None:
            raise GeometryQueryError(f"{measurement.pair}: 构型辅助距离无效")
        margin = measurement.distance.distance_m - measurement.required_m
        if margin < influence_m:
            gradient += (1 - max(0., margin) / influence_m) * measurement.gradient / influence_m
    # 相同零空间投影与整体速度比例用于全部实验方法。
    projector = np.eye(9) - np.linalg.pinv(jacobian, rcond=1e-10) @ jacobian
    velocity = projector @ gradient
    maximum = np.minimum(limits.max_velocity, speed_limit)
    factor = max(1., float(np.max(np.abs(velocity) / maximum)))
    if factor > 1.:
        factor = np.nextafter(factor, np.inf)
    return velocity / factor


@jax.jit
def _solve(P, c, G, h):
    A, b = jnp.zeros((0, P.shape[0]), dtype=P.dtype), jnp.zeros(0, dtype=P.dtype)
    solution, slack, dual, equality_dual, converged, iterations = solve_qp(
        P, c, A, b, G, h, solver_tol=1e-9, max_iter=100,
    )
    health = terminal_qp_health(
        P, c, A, b, G, h, solution, slack, dual, equality_dual, solver_tol=1e-8,
    )
    return solution, health, converged, iterations, dual


@jax.jit
def _solve_joint(P, c, G, h):
    solution, health, _, iterations, original_dual = _solve(P, c, G, h)

    def boundary_solution():
        # 固定进给后重新求解九轴运动，两个分支在预热时共同编译。
        boundary = jnp.clip(solution[9], 0., 1.)
        reduced, reduced_health, _, reduced_iterations, dual = _solve(
            P[:9, :9], c[:9] + P[:9, 9] * boundary,
            G[:-2, :9], h[:-2] - G[:-2, 9] * boundary,
        )
        corrected = jnp.concatenate([reduced, jnp.array([boundary])])
        derivative = (P @ corrected + c + G[:-2].T @ dual)[9]
        boundary_dual = jnp.where(
            boundary == 1., jnp.array([-derivative, 0.]), jnp.array([0., derivative]),
        )
        dual = jnp.concatenate([dual, boundary_dual])
        corrected_health = terminal_qp_health(
            P, c, jnp.zeros((0, 10)), jnp.zeros(0), G, h, corrected,
            h - G @ corrected, dual, jnp.zeros(0), solver_tol=1e-8,
        )
        final_health = jax.tree.map(
            lambda valid, invalid: jnp.where(reduced_health.accepted, valid, invalid),
            corrected_health, reduced_health,
        )
        return corrected, final_health, iterations + reduced_iterations, dual

    return jax.lax.cond(
        health.accepted & ((solution[9] < 0.) | (solution[9] > 1.)),
        boundary_solution, lambda: (solution, health, iterations, original_dual),
    )


@jax.jit
def _solve_or_reuse(P, c, G, h, solution, dual, can_reuse):
    size = len(solution)
    health = terminal_qp_health(
        P, c, jnp.zeros((0, size)), jnp.zeros(0), G, h, solution,
        h - G @ solution, dual, jnp.zeros(0), solver_tol=1e-8,
    )

    def solve_new_problem():
        if size == 10:
            result, checked, iterations, multipliers = _solve_joint(P, c, G, h)
        else:
            result, checked, _, iterations, multipliers = _solve(P, c, G, h)
        return result, checked, iterations, multipliers, False

    return jax.lax.cond(
        can_reuse & health.accepted,
        lambda: (solution, health, jnp.asarray(0), dual, True),
        solve_new_problem,
    )


def _zero_feed_candidate(problem, previous, *, joint):
    if previous is None or previous.solver != "qp" or previous.feed_m_s != 0.:
        return None
    previous_joint = len(previous.solution) == 10
    old_rows = slice(None, -2) if previous_joint else slice(None)
    rows = slice(None, -2) if joint else slice(None)
    original = previous.problem
    if previous.dual is None or not all((
        np.array_equal(problem["P"][:9, :9], original["P"][:9, :9]),
        np.array_equal(problem["c"][:9], original["c"][:9]),
        np.array_equal(problem["G"][rows, :9], original["G"][old_rows, :9]),
        np.array_equal(problem["h"][rows], original["h"][old_rows]),
    )):
        return None
    solution = np.r_[previous.velocity, 0.] if joint else previous.velocity.copy()
    dual = previous.dual[old_rows].copy()
    if joint:
        derivative = (problem["P"] @ solution + problem["c"]
                      + problem["G"][:-2].T @ dual)[9]
        dual = np.r_[dual, 0., derivative]
    return solution, dual


def _same_problem_candidate(problem, previous):
    if (previous is None or previous.solver != "qp" or previous.dual is None
            or set(problem) != set(previous.problem)
            or not all(np.array_equal(value, previous.problem[key]) for key, value in problem.items())):
        return None
    return previous.solution.copy(), previous.dual.copy()


def _solve_problem(problem, *, joint, previous_zero_feed=None, previous_problem=None):
    exact = _same_problem_candidate(problem, previous_problem)
    candidate = exact if exact is not None else _zero_feed_candidate(problem, previous_zero_feed, joint=joint)
    solution, dual = (
        (np.zeros(len(problem["c"])), np.zeros(len(problem["h"])))
        if candidate is None else candidate
    )
    result = _solve_or_reuse(
        *[jnp.asarray(problem[key]) for key in ("P", "c", "G", "h")],
        jnp.asarray(solution), jnp.asarray(dual), jnp.asarray(candidate is not None),
    )
    return (*result, exact is not None)


def _separated_feed(jacobian, reference_rate, correction, posture, constraints, nominal):
    inverse = np.linalg.pinv(jacobian, rcond=1e-10)
    bias = inverse @ correction + posture
    slope = inverse @ reference_rate
    coefficients = constraints.G @ slope
    available = constraints.h - constraints.G @ bias
    lower, upper = 0., nominal
    for coefficient, bound in zip(coefficients, available):
        if abs(coefficient) < 1e-12:
            if bound < 0:
                return 0.
        elif coefficient > 0:
            upper = min(upper, bound / coefficient)
        else:
            lower = max(lower, bound / coefficient)
    return float(upper) if upper >= lower else 0.


def solve_velocity(
        terms, constraints: VelocityConstraints, *, nominal_feed: float,
        previous_velocity, posture_velocity, mode: str,
        gains=(40., 40., 40., 20., 20.),
        task_weights=(1e4, 1e4, 1e4, 1e3, 1e3),
        posture_weight: float = .01, smooth_weight: float = .02,
        feed_weight: float = 100., max_velocity=None, fixed_feed: bool = False,
        previous_zero_feed: VelocitySolution | None = None,
        previous_problem: VelocitySolution | None = None) -> VelocitySolution:
    if mode not in ("separated", "joint", "sns"):
        raise ValueError("solver_mode 必须为 separated、joint 或 sns")
    if not np.isfinite(nominal_feed) or nominal_feed < 0:
        raise ValueError("名义进给必须为有限非负值")
    if type(fixed_feed) is not bool:
        raise ValueError("fixed_feed 必须为布尔值")
    jacobian = np.asarray(terms.jacobian, dtype=float)
    rate = np.asarray(terms.reference_rate, dtype=float)
    error = np.asarray(terms.error, dtype=float)
    previous, posture = np.asarray(previous_velocity, dtype=float), np.asarray(posture_velocity, dtype=float)
    gains, weights = np.asarray(gains, dtype=float), np.asarray(task_weights, dtype=float)
    if (not bool(terms.valid) or jacobian.shape != (5, 9) or rate.shape != (5,)
            or error.shape != (5,) or gains.shape != (5,) or weights.shape != (5,)
            or previous.shape != (9,) or posture.shape != (9,)
            or constraints.G.ndim != 2 or constraints.G.shape[1] != 9
            or constraints.h.shape != (constraints.G.shape[0],)):
        raise ValueError("五维任务或 QP 输入形状无效")
    values = np.concatenate([
        jacobian.ravel(), rate, error, previous, posture, gains, weights,
        constraints.G.ravel(), constraints.h,
        [posture_weight, smooth_weight, feed_weight],
    ])
    if not np.all(np.isfinite(values)) or np.any(gains <= 0) or np.any(weights <= 0):
        raise ValueError("QP 输入必须有限，任务增益和权重必须大于零")
    if min(posture_weight, smooth_weight, feed_weight) <= 0:
        raise ValueError("QP 目标权重必须大于零")
    correction = -gains * error
    if mode == "sns":
        vmax = np.asarray(max_velocity, dtype=float)
        if vmax.shape != (9,) or not np.all(np.isfinite(vmax)) or np.any(vmax <= 0):
            raise ValueError("Generalized-SNS 要求九轴实际速度限制")
        desired = correction + rate * nominal_feed
        lower = -np.abs(constraints.G) @ vmax
        problem = {
            "J": jacobian, "desired": desired, "G": constraints.G, "h": constraints.h,
            "lower": lower, "nominal_feed": np.array([nominal_feed]),
        }
        try:
            result = generalized_sns(jacobian, desired, constraints.G, lower, constraints.h)
            result = add_posture(
                result, jacobian, (posture_weight * posture + smooth_weight * previous)
                / (posture_weight + smooth_weight),
                constraints.G, lower, constraints.h,
            )
        except RuntimeError as error:
            raise SolverFailure(str(error), problem, solver="sns") from error
        return VelocitySolution(
            result.velocity, nominal_feed * result.task_scale, result.velocity, problem,
            None, result.hard_violation, result.iterations, "sns",
            result.task_scale, result.task_residual, result.saturated,
        )
    joint = mode == "joint" and nominal_feed > 0 and not fixed_feed
    if joint:
        task = np.column_stack([jacobian, -rate * nominal_feed])
        target = correction
        G = np.concatenate([
            np.column_stack([constraints.G, np.zeros(len(constraints.h))]),
            np.array([[0.] * 9 + [1.], [0.] * 9 + [-1.]]),
        ])
        h = np.concatenate([constraints.h, [1., 0.]])
        feed = None
    else:
        feed = (nominal_feed if fixed_feed else
                _separated_feed(jacobian, rate, correction, posture, constraints, nominal_feed))
        task, target = jacobian, correction + rate * feed
        G, h = constraints.G.copy(), constraints.h.copy()
    P = 2 * task.T @ (weights[:, None] * task)
    c = -2 * task.T @ (weights * target)
    P[:9, :9] += 2 * (posture_weight + smooth_weight) * np.eye(9)
    c[:9] -= 2 * (posture_weight * posture + smooth_weight * previous)
    if joint:
        P[9, 9] += 2 * feed_weight * nominal_feed ** 2
        c[9] -= 2 * feed_weight * nominal_feed ** 2
    original_G, original_h = G.copy(), h.copy()
    scale = np.maximum.reduce([np.linalg.norm(G, axis=1), np.abs(h), np.ones(len(h))])
    G, h = G / scale[:, None], h / scale
    problem = {
        "P": P, "c": c, "G": G, "h": h,
        "variable_scale": np.r_[np.ones(9), nominal_feed] if joint else np.ones(9),
    }
    solution, health, iterations, dual, reused, exact = _solve_problem(
        problem, joint=joint, previous_zero_feed=previous_zero_feed,
        previous_problem=previous_problem,
    )
    solution = np.asarray(solution)
    residual = float(health.kkt_residual)
    violation = max(0., float(np.max(original_G @ solution - original_h)))
    if not bool(health.accepted) or violation > 1e-9 or not np.all(np.isfinite(solution)):
        raise SolverFailure(f"QP 未通过最终状态检查：KKT={residual}, violation={violation}", problem)
    if joint:
        feed = float(solution[9]) * nominal_feed
        if feed < 0 or feed > nominal_feed:
            raise SolverFailure(f"QP 进给超出允许范围：{feed}", problem)
    return VelocitySolution(
        solution[:9].copy(), feed, solution, problem, residual, violation, int(iterations),
        dual=np.asarray(dual).copy(), reused_zero_feed=bool(reused) and not exact,
        reused_problem=bool(reused) and exact,
    )
