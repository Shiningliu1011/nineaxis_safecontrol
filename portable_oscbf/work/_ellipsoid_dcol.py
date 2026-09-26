from __future__ import annotations

from typing import NamedTuple

import jax
import jax.numpy as jnp

from work._collision_geometry import _build_world_geometry


class _DcolResult(NamedTuple):
    scale: jax.Array
    primal: jax.Array
    dual: jax.Array
    iteration: jax.Array
    healthy: jax.Array


def _solve_pair(
    center_a: jax.Array,
    factor_a: jax.Array,
    center_b: jax.Array,
    factor_b: jax.Array,
    max_iterations: int,
    primal_limit: float,
    dual_limit: float,
) -> _DcolResult:
    # 同比例归一化保持 scale，减少米制尺寸差异造成的数值范围变化。
    length = jax.lax.stop_gradient(jnp.maximum(
        jnp.maximum(jnp.max(jnp.abs(factor_a)), jnp.max(jnp.abs(factor_b))),
        jnp.max(jnp.abs(center_b - center_a)),
    ))
    a, b = factor_a / length, factor_b / length
    d = (center_b - center_a) / length
    qa, qb = a @ a.T, b @ b.T

    def terms(t: jax.Array) -> tuple[jax.Array, ...]:
        shape = (1.0 - t) * qa + t * qb
        direction = jnp.linalg.solve(shape, d)
        displacement = (1.0 - t) * (qa @ direction)
        ua = jnp.linalg.solve(a, displacement)
        ub = jnp.linalg.solve(b, displacement - d)
        na, nb = jnp.linalg.norm(ua), jnp.linalg.norm(ub)
        square = t * (1.0 - t) * jnp.dot(d, direction)
        # 同心时保留零 scale；health 拒绝该处不唯一的线性 scale 梯度。
        scale = jnp.where(square > 0.0, jnp.sqrt(jnp.where(square > 0.0, square, 1.0)), 0.0)
        denominator = jnp.where(scale > 0.0, scale, 1.0)
        primal = jnp.maximum(jnp.maximum(na, nb) - scale, 0.0)
        stationarity = (
            t * jnp.linalg.solve(a.T, ua) + (1.0 - t) * jnp.linalg.solve(b.T, ub)
        ) / denominator
        dual = jnp.max(jnp.array([
            jnp.abs((t * na + (1.0 - t) * nb) / denominator - 1.0),
            jnp.linalg.norm(stationarity),
            jnp.abs(na - nb) / denominator,
            jnp.abs(t * na * (na - scale) / denominator),
            jnp.abs((1.0 - t) * nb * (nb - scale) / denominator),
            jnp.linalg.norm(shape @ direction - d),
        ]))
        finite = jnp.all(jnp.isfinite(jnp.array([scale, primal, dual, na, nb])))
        healthy = finite & (square > 0.0) & (primal <= primal_limit) & (dual <= dual_limit)
        return scale, primal, dual, healthy, na - nb

    initial = terms(jnp.asarray(0.5))

    def continue_solve(state: tuple) -> jax.Array:
        _, _, _, iteration, values = state
        return (iteration < max_iterations) & ~values[3]

    def bisect(state: tuple) -> tuple:
        lower, upper, t, iteration, values = state
        lower = jnp.where(values[4] > 0.0, t, lower)
        upper = jnp.where(values[4] > 0.0, upper, t)
        t = (lower + upper) * 0.5
        return lower, upper, t, iteration + 1, terms(t)

    _, _, t, iteration, values = jax.lax.while_loop(
        continue_solve, bisect,
        (jnp.asarray(0.0), jnp.asarray(1.0), jnp.asarray(0.5), jnp.int32(0), initial),
    )
    # 最优值的 envelope derivative 不需要对二分分支求导。
    t = jax.lax.stop_gradient(t)
    scale, primal, dual, healthy, _ = terms(t)
    return _DcolResult(scale, primal, dual, iteration, healthy)


def _build_self_query(geometry, max_iterations: int, primal_limit: float, dual_limit: float):
    pairs = geometry.pairs
    world_geometry = _build_world_geometry(geometry)

    def solve(ca, la, cb, lb):
        return _solve_pair(ca, la, cb, lb, max_iterations, primal_limit, dual_limit)

    def one_state(q: jax.Array):
        centers, factors = world_geometry(q)
        dc, dl = jax.jacfwd(world_geometry)(q)
        ca, cb = centers[pairs[:, 0]], centers[pairs[:, 1]]
        la, lb = factors[pairs[:, 0]], factors[pairs[:, 1]]

        def scale_with_aux(ca, la, cb, lb):
            result = solve(ca, la, cb, lb)
            return result.scale, result

        (_, result), derivatives = jax.vmap(jax.value_and_grad(
            scale_with_aux, argnums=(0, 1, 2, 3), has_aux=True
        ))(ca, la, cb, lb)
        gradient = (
            jnp.einsum("pi,pij->pj", derivatives[0], dc[pairs[:, 0]])
            + jnp.einsum("pik,pikj->pj", derivatives[1], dl[pairs[:, 0]])
            + jnp.einsum("pi,pij->pj", derivatives[2], dc[pairs[:, 1]])
            + jnp.einsum("pik,pikj->pj", derivatives[3], dl[pairs[:, 1]])
        )
        return result._replace(healthy=result.healthy & jnp.all(jnp.isfinite(gradient), axis=-1)), gradient

    return jax.jit(jax.vmap(one_state))
