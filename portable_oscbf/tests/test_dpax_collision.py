#!/usr/bin/env python3
"""M3 acceptance: DCOL OBB distance kernel and its gradients."""

from __future__ import annotations

import jax

# The control pipeline runs with JAX x64; enable it before building models.
jax.config.update("jax_enable_x64", True)

import _path_setup  # noqa: F401
import numpy as np
import jax.numpy as jnp
import pytest

from work.dpax_collision import (
    self_collision_distances_jit,
    self_collision_grads_jit,
)
from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
from work.obb_collision_model import (
    OBB_COLLISION_PAIRS,
    OBB_HALF_EXTENTS_M,
    OBB_LOCAL_CENTERS_M,
    OBB_LOCAL_ROTATIONS,
)


def _fcl_box_distance(transform, i, j) -> float:
    import fcl

    objects = []
    for index in (i, j):
        rotation = transform[index][:3, :3] @ OBB_LOCAL_ROTATIONS[index]
        center = (
            transform[index][:3, :3] @ OBB_LOCAL_CENTERS_M[index]
            + transform[index][:3, 3])
        half = OBB_HALF_EXTENTS_M[index]
        tf = fcl.Transform()
        tf.setRotation(rotation)
        tf.setTranslation(center)
        box = fcl.Box(2.0 * half[0], 2.0 * half[1], 2.0 * half[2])
        objects.append(fcl.CollisionObject(box, tf))
    request = fcl.DistanceRequest()
    request.enable_nearest_points = False
    result = fcl.DistanceResult()
    fcl.distance(objects[0], objects[1], request, result)
    return float(result.min_distance)


@pytest.fixture(scope="module")
def robot():
    return NineaxisManipulatorJAX()


@pytest.fixture(scope="module")
def sample_qs(robot):
    rng = np.random.default_rng(20260806)
    lower = np.asarray(robot.joint_lower_limits)
    upper = np.asarray(robot.joint_upper_limits)
    margin = 0.05 * np.ones(9)
    margin[0] = 0.05
    return [rng.uniform(lower + margin, upper - margin) for _ in range(6)]


def test_distance_matches_fcl_box_reference(robot, sample_qs):
    """DCOL OBB distance must match FCL Box distance within 5%."""

    max_relative_error = 0.0
    for q in sample_qs:
        transforms = np.asarray(robot._compute_all_link_transforms(jnp.asarray(q)))
        distances = np.asarray(
            self_collision_distances_jit(jnp.asarray(q)))
        for pair_index, (i, j) in enumerate(OBB_COLLISION_PAIRS):
            fcl_distance = _fcl_box_distance(transforms, int(i), int(j))
            relative = abs(distances[pair_index] - fcl_distance) / max(
                fcl_distance, 1e-9)
            max_relative_error = max(max_relative_error, float(relative))
    assert max_relative_error < 0.05, (
        f"DCOL vs FCL Box max relative error {max_relative_error:.4f}")


def test_distance_gradient_matches_finite_difference(robot, sample_qs):
    """jax.grad of a pair distance must match central differences."""

    pair_indices = [0, 5, 9]
    epsilon = 1e-6
    for q in sample_qs[:3]:
        q_jax = jnp.asarray(q)
        grads = np.asarray(self_collision_grads_jit(
            q_jax, jnp.arange(len(OBB_COLLISION_PAIRS))))
        for pair_index in pair_indices:
            analytic = grads[pair_index]
            finite = np.zeros(9)
            for k in range(9):
                q_plus = q.copy()
                q_minus = q.copy()
                q_plus[k] += epsilon
                q_minus[k] -= epsilon
                d_plus = float(np.asarray(self_collision_distances_jit(
                    jnp.asarray(q_plus)))[pair_index])
                d_minus = float(np.asarray(self_collision_distances_jit(
                    jnp.asarray(q_minus)))[pair_index])
                finite[k] = (d_plus - d_minus) / (2.0 * epsilon)
            absolute = np.max(np.abs(analytic - finite))
            assert absolute < 1e-6, (
                f"gradient absolute error {absolute:.2e} "
                f"(q={q}, pair={pair_index})")
            significant = np.abs(analytic) >= 1e-6
            if np.any(significant):
                relative = np.max(
                    np.abs(analytic[significant] - finite[significant])
                    / np.abs(analytic[significant]))
                assert relative < 1e-3, (
                    f"gradient relative error {relative:.2e} "
                    f"(q={q}, pair={pair_index})")


def test_jit_cache_is_stable(robot, sample_qs):
    """First frame compiles; subsequent calls must reuse one cache entry."""

    fn = self_collision_distances_jit
    for q in sample_qs:
        fn(jnp.asarray(q)).block_until_ready()
    assert fn._cache_size() == 1, (
        f"expected 1 JIT cache entry, got {fn._cache_size()}")


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
