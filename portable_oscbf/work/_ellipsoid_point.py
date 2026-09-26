from __future__ import annotations

from typing import NamedTuple

import jax
import jax.numpy as jnp

from work._collision_geometry import _build_world_geometry


class _PointBarrier(NamedTuple):
    barrier: jax.Array
    gradient: jax.Array
    scale: jax.Array
    time_derivative: jax.Array
    healthy: jax.Array


class _PointDistance(NamedTuple):
    distance: jax.Array
    robot_point: jax.Array
    support_point: jax.Array
    healthy: jax.Array


def _closest_point(point, radii, max_iterations, tolerance_m):
    # 平移 multiplier 后，内部点的根位于 delta >= 0，最短半轴附近保持有效精度。
    length = jnp.max(radii)
    a, p = radii / length, point / length
    a2 = a * a
    minimum = jnp.min(a2)
    gap = a2 - minimum
    shortest = gap == 0.0
    on_surface = jnp.sum((p / a) ** 2) == 1.0
    outside = jnp.sum((p / a) ** 2) > 1.0
    singular = jnp.where(shortest, 0.0, a2 * p / jnp.where(shortest, 1.0, gap))
    singular_norm_sq = jnp.sum((singular / a) ** 2)
    hard_case = jnp.all(jnp.where(shortest, p == 0.0, True)) & (singular_norm_sq <= 1.0)
    axis = jnp.argmin(a2)
    singular = singular.at[axis].set(a[axis] * jnp.sqrt(jnp.maximum(1.0 - singular_norm_sq, 0.0)))

    def candidate(delta):
        denominator = gap + delta
        # 零坐标对方程贡献为零；其他零分母仅出现在根区间端点。
        return jnp.where(p == 0.0, 0.0, a2 * p / jnp.where(p == 0.0, 1.0, denominator))

    lower = jnp.where(outside, minimum, 0.0)
    upper = jnp.where(outside, minimum + jnp.linalg.norm(a * p), minimum)

    def witness(lower, upper):
        middle = candidate((lower + upper) * 0.5)
        norm = jnp.linalg.norm(middle / a)
        surface = middle / jnp.where(norm > 0.0, norm, 1.0)
        error = (jnp.linalg.norm(candidate(lower) - candidate(upper))
                 + jnp.linalg.norm(surface - middle)) * length
        return surface, error

    def continue_solve(state):
        lower, upper, iteration = state
        _, error = witness(lower, upper)
        return (~hard_case & ~on_surface & (iteration < max_iterations)
                & ~(error <= tolerance_m))

    def bisect(state):
        lower, upper, iteration = state
        middle = (lower + upper) * 0.5
        residual = jnp.sum((candidate(middle) / a) ** 2) - 1.0
        return (jnp.where(residual > 0.0, middle, lower),
                jnp.where(residual > 0.0, upper, middle), iteration + 1)

    lower, upper, _ = jax.lax.while_loop(continue_solve, bisect, (lower, upper, jnp.int32(0)))
    surface, error = witness(lower, upper)
    surface = jnp.where(hard_case, singular, jnp.where(on_surface, p, surface))
    error = jnp.where(hard_case | on_surface, 0.0, error)
    normal = surface / a2
    normal = normal / jnp.linalg.norm(normal)
    signed = jnp.where(outside, 1.0, -1.0) * jnp.linalg.norm(p - surface) * length
    healthy = (jnp.all(jnp.isfinite(surface)) & jnp.all(jnp.isfinite(normal))
               & jnp.isfinite(signed) & (error <= tolerance_m))
    return signed, surface * length, normal, healthy


def _build_environment_queries(geometry, max_iterations: int, tolerance_m: float):
    world_geometry = _build_world_geometry(geometry)
    ids = geometry.active_ids
    radii = geometry.radii[ids]

    def local_points(q, points):
        centers, factors = world_geometry(q)
        centers = centers[ids]
        rotations = factors[ids] / radii[:, None, :]
        local = jnp.einsum("gij,gsj->gsi", jnp.swapaxes(rotations, -1, -2),
                           points[None, :, :] - centers[:, None, :])
        return local, centers, rotations

    def barrier_state(q, points, rho_m, clearance_m, velocity):
        def square(q):
            local, _, _ = local_points(q, points)
            return jnp.sum((local / radii[:, None, :]) ** 2, axis=-1)

        scale_sq = square(q)
        gradient = jax.jacfwd(square)(q)
        local, _, rotations = local_points(q, points)
        local_velocity = jnp.einsum("gji,sj->gsi", rotations, velocity)
        time_derivative = 2.0 * jnp.sum(local * local_velocity / radii[:, None, :] ** 2, axis=-1)
        margin = 1.0 + (rho_m + clearance_m)[None, :] / jnp.min(radii, axis=-1)[:, None]
        barrier = scale_sq - margin ** 2
        healthy = (jnp.isfinite(barrier) & jnp.isfinite(scale_sq) & jnp.isfinite(time_derivative)
                   & jnp.all(jnp.isfinite(gradient), axis=-1))
        return _PointBarrier(barrier, gradient, jnp.sqrt(scale_sq), time_derivative, healthy)

    def distance_state(q, points, rho_m):
        local, centers, rotations = local_points(q, points)

        def one_ellipsoid(local, a):
            return jax.vmap(lambda p: _closest_point(p, a, max_iterations, tolerance_m))(local)

        signed, surface, normal, healthy = jax.vmap(one_ellipsoid)(local, radii)
        robot_point = centers[:, None, :] + jnp.einsum("gij,gsj->gsi", rotations, surface)
        world_normal = jnp.einsum("gij,gsj->gsi", rotations, normal)
        support_point = points[None, :, :] - rho_m[None, :, None] * world_normal
        distance = signed - rho_m[None, :]
        healthy &= (jnp.isfinite(distance) & jnp.all(jnp.isfinite(robot_point), axis=-1)
                    & jnp.all(jnp.isfinite(support_point), axis=-1))
        return _PointDistance(distance, robot_point, support_point, healthy)

    return (jax.jit(jax.vmap(barrier_state, in_axes=(0, None, None, None, None))),
            jax.jit(jax.vmap(distance_state, in_axes=(0, None, None))))
