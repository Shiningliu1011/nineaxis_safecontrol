"""Disabled obstacle slots must skip geometry without changing active terms."""

import jax
jax.config.update("jax_enable_x64", True)
import jax.numpy as jnp
import numpy as np
import pytest

from work import dpax_collision
from work.jax_barrier_terms import compute_dcol_obstacle_clearance


def test_disabled_obstacles_skip_geometry_and_reenable_without_recompile(monkeypatch):
    calls = []
    original = dpax_collision.obb_sphere_clearance

    def counted(*args):
        jax.debug.callback(lambda: calls.append(True), ordered=True)
        return original(*args)

    monkeypatch.setattr(dpax_collision, "obb_sphere_clearance", counted)
    q = jnp.array([0.359, -0.7, -1.396, -1.471, 1.937,
                   1.183, 0.306, 0.658, 0.125])
    positions = jnp.full((8, 3), 2.0)
    radii = jnp.full(8, 0.1)
    margins = jnp.full(8, 0.05)
    velocities = jnp.full((8, 3), 0.01)
    growth = jnp.full(8, 0.001)
    traces = []

    def evaluate(q, enabled):
        traces.append(True)
        return compute_dcol_obstacle_clearance(
            q, positions, radii, margins, velocities, growth,
            obs_enabled=enabled)

    compiled = jax.jit(evaluate)
    inactive = compiled(q, jnp.zeros(8))
    jax.block_until_ready(inactive)
    jax.effects_barrier()
    assert calls == []
    for result in inactive:
        np.testing.assert_array_equal(result, np.zeros((10, 8)))
    inactive_grad = jax.jit(jax.jacfwd(
        lambda q: evaluate(q, jnp.zeros(8))[0]))(q)
    np.testing.assert_array_equal(inactive_grad, np.zeros((10, 8, 9)))
    jax.effects_barrier()
    assert calls == []

    expected = original(q, positions, radii, velocities, growth, margins)
    trace_count = len(traces)
    for enabled in (jnp.ones(8), jnp.zeros(8).at[3].set(1.0)):
        active = compiled(q, enabled)
        jax.block_until_ready(active)
        jax.effects_barrier()
        for actual, reference in zip(active, expected):
            np.testing.assert_allclose(actual, reference, atol=1e-10, rtol=1e-10)
    assert len(calls) == 2
    assert len(traces) == trace_count


@pytest.mark.parametrize("aggregate", [False, True])
def test_masked_cbf_values_and_gradients_match_unconditional_geometry(monkeypatch, aggregate):
    from work import jax_barrier_terms
    from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
    from work.oscbf_velocity_config import NineaxisOSCBFVelocityConfig

    config = NineaxisOSCBFVelocityConfig(
        NineaxisManipulatorJAX(), aggregate_dynamic_obstacles=aggregate)
    q = jnp.array([0.359, -0.7, -1.396, -1.471, 1.937,
                   1.183, 0.306, 0.658, 0.125])
    positions = jnp.full((8, 3), 2.0)
    radii = jnp.full(8, 0.1)

    def terms(q, enabled):
        return config._compute_dcol_obstacle_constraints(
            q, positions, radii, enabled)

    optimized = jax.jit(lambda q, enabled: (
        terms(q, enabled), jax.jacfwd(terms)(q, enabled)))
    masks = (jnp.zeros(8), jnp.zeros(8).at[3].set(1.0), jnp.ones(8))
    results = [optimized(q, mask) for mask in masks]
    jax.block_until_ready(results)

    def unconditional(*args, obs_enabled=None):
        return compute_dcol_obstacle_clearance(*args)

    monkeypatch.setattr(jax_barrier_terms, "compute_dcol_obstacle_clearance", unconditional)
    reference = jax.jit(lambda q, enabled: (
        terms(q, enabled), jax.jacfwd(terms)(q, enabled)))
    for actual, mask in zip(results, masks):
        expected = reference(q, mask)
        for value, baseline in zip(actual, expected):
            np.testing.assert_allclose(value, baseline, atol=1e-10, rtol=1e-10)
