"""Parity tests for the fixed-shape JAX arc-length path state machine."""

import _path_setup  # noqa: F401
import jax
import jax.numpy as jnp
import numpy as np
from scipy.spatial.transform import Rotation

from path_following import (
    PathFollowerState,
    PathFollowingConfig,
    PathGeometry,
    advance_path_state,
    feedrate_limit_from_box,
    feedrate_limit_from_inequalities,
)
from work.jax_path_following import (
    as_jax_path_config,
    as_jax_path_geometry,
    advance_path_state_jax,
    feedrate_limit_from_box_jax,
    feedrate_limit_from_inequalities_jax,
    initial_path_state_jax,
    reconcile_path_state_after_motion_jax,
    sample_path_jax,
)


def _geometry():
    positions = np.array([
        [0.0, 0.0, 0.0],
        [0.10, 0.0, 0.0],
        [0.20, 0.10, 0.0],
    ])
    rotations = Rotation.from_euler("z", [0.0, 15.0, 30.0], degrees=True).as_matrix()
    return PathGeometry.from_samples(
        positions, rotations,
        np.array([0.04, 0.06, 0.03]),
        np.array([0.0, 0.5, 1.0]),
    )


def test_jax_sample_matches_host_path_geometry():
    geometry = _geometry()
    jgeometry = as_jax_path_geometry(geometry)
    progress = 0.13

    host = geometry.sample(progress)
    jax_sample = sample_path_jax(jgeometry, jnp.asarray(progress))

    np.testing.assert_allclose(jax_sample.position_m, host.position_m, atol=2e-6)
    np.testing.assert_allclose(jax_sample.tangent, host.tangent, atol=2e-6)
    np.testing.assert_allclose(jax_sample.rotation, host.rotation, atol=2e-6)
    np.testing.assert_allclose(jax_sample.omega_per_m, host.omega_per_m, atol=2e-6)
    assert np.isclose(float(jax_sample.feedrate_m_s), host.feedrate_m_s, atol=2e-6)


def test_jax_one_way_progress_matches_host_and_reuses_one_jit_cache_entry():
    geometry = _geometry()
    config = PathFollowingConfig(reference_lead_m=0.005)
    jgeometry = as_jax_path_geometry(geometry)
    jconfig = as_jax_path_config(config)
    state = PathFollowerState(
        reference_progress_m=0.03,
        projected_progress_m=0.02,
        projection_segment=0,
    )
    ee_position = np.array([0.028, 0.0002, 0.0])
    host = advance_path_state(
        geometry, config, state, ee_position, dt_s=0.01,
        feedrate_joint_limit_m_s=0.05,
        feedrate_cbf_limit_m_s=0.045,
        feedrate_rate_limit_m_s=0.04,
    )
    jstate = initial_path_state_jax().at[0].set(state.reference_progress_m)
    jstate = jstate.at[1].set(state.projected_progress_m)
    jstate = jstate.at[2].set(state.projection_segment)
    step = jax.jit(lambda path_state, point: advance_path_state_jax(
        jgeometry, jconfig, path_state, point, dt_s=0.01,
        feedrate_joint_limit_m_s=jnp.asarray(0.05),
        feedrate_cbf_limit_m_s=jnp.asarray(0.045),
        feedrate_rate_limit_m_s=jnp.asarray(0.04),
    ))
    result = step(jstate, jnp.asarray(ee_position))
    _ = step(result.state, jnp.asarray([0.03, 0.0001, 0.0]))

    np.testing.assert_allclose(result.state[:2], [
        host.next_state.reference_progress_m,
        host.next_state.projected_progress_m,
    ], atol=2e-6)
    assert np.isclose(float(result.feedrate_m_s), host.feedrate_m_s, atol=2e-6)
    assert step._cache_size() == 1


def test_jax_phase_catch_up_matches_host_for_a_projection_ahead_of_reference():
    geometry = _geometry()
    config = PathFollowingConfig(reference_lead_m=0.005)
    state = PathFollowerState(
        reference_progress_m=0.03,
        projected_progress_m=0.03,
        projection_segment=0,
    )
    ee_position = np.array([0.06, 0.0, 0.0])
    host = advance_path_state(
        geometry, config, state, ee_position, dt_s=0.01,
        feedrate_joint_limit_m_s=0.08,
        feedrate_cbf_limit_m_s=0.08,
        feedrate_rate_limit_m_s=0.08,
    )
    jstate = initial_path_state_jax().at[0].set(state.reference_progress_m)
    jstate = jstate.at[1].set(state.projected_progress_m)
    jstate = jstate.at[2].set(state.projection_segment)
    result = advance_path_state_jax(
        as_jax_path_geometry(geometry), as_jax_path_config(config), jstate,
        jnp.asarray(ee_position), dt_s=0.01,
        feedrate_joint_limit_m_s=jnp.asarray(0.08),
        feedrate_cbf_limit_m_s=jnp.asarray(0.08),
        feedrate_rate_limit_m_s=jnp.asarray(0.08),
    )

    np.testing.assert_allclose(result.state[:2], [
        host.next_state.reference_progress_m,
        host.next_state.projected_progress_m,
    ], atol=2e-6)
    assert float(result.state[0]) >= float(result.state[1])


def test_jax_reconcile_phase_catch_up_matches_host():
    geometry = _geometry()
    config = PathFollowingConfig()
    state = PathFollowerState(
        reference_progress_m=0.03,
        projected_progress_m=0.03,
        projection_segment=0,
    )
    ee_position = np.array([0.06, 0.0, 0.0])
    from path_following import reconcile_path_state_after_motion
    host = reconcile_path_state_after_motion(
        geometry, config, state, ee_position, dt_s=0.01)
    jstate = initial_path_state_jax().at[0].set(state.reference_progress_m)
    jstate = jstate.at[1].set(state.projected_progress_m)
    jstate = jstate.at[2].set(state.projection_segment)
    result = reconcile_path_state_after_motion_jax(
        as_jax_path_geometry(geometry), as_jax_path_config(config), jstate,
        jnp.asarray(ee_position), dt_s=0.01)

    np.testing.assert_allclose(result[:2], [
        host.reference_progress_m,
        host.projected_progress_m,
    ], atol=2e-6)


def test_jax_feedrate_caps_match_host_closed_form():
    bias = np.array([0.1, -0.2])
    direction = np.array([2.0, 1.0])
    lower = np.array([-1.0, -1.0])
    upper = np.array([1.0, 1.0])
    G = np.array([[1.0, 0.0], [0.0, -1.0]])
    h = np.array([0.7, 0.4])

    host_box = feedrate_limit_from_box(bias, direction, lower, upper)
    host_cbf = feedrate_limit_from_inequalities(G, h, bias, direction)
    jax_box = feedrate_limit_from_box_jax(
        jnp.asarray(bias), jnp.asarray(direction), jnp.asarray(lower), jnp.asarray(upper))
    jax_cbf = feedrate_limit_from_inequalities_jax(
        jnp.asarray(G), jnp.asarray(h), jnp.asarray(bias), jnp.asarray(direction))

    assert np.isclose(float(jax_box), host_box, atol=1e-7)
    assert np.isclose(float(jax_cbf), host_cbf, atol=1e-7)


def test_jax_endpoint_brake_matches_host_before_the_final_sample():
    geometry = _geometry()
    config = PathFollowingConfig(
        reference_lead_m=0.005,
        endpoint_braking_deceleration_m_s2=0.05,
    )
    state = PathFollowerState(
        reference_progress_m=geometry.total_length_m - 0.001,
        projected_progress_m=geometry.total_length_m - 0.001,
        projection_segment=geometry.num_segments - 1,
    )
    ee_position = geometry.sample(state.reference_progress_m).position_m
    host = advance_path_state(
        geometry, config, state, ee_position, dt_s=0.01,
        feedrate_joint_limit_m_s=1.0,
        feedrate_cbf_limit_m_s=1.0,
        feedrate_rate_limit_m_s=1.0,
    )
    jstate = initial_path_state_jax().at[0].set(state.reference_progress_m)
    jstate = jstate.at[1].set(state.projected_progress_m)
    jstate = jstate.at[2].set(state.projection_segment)
    jax_result = advance_path_state_jax(
        as_jax_path_geometry(geometry), as_jax_path_config(config), jstate,
        jnp.asarray(ee_position), dt_s=0.01,
        feedrate_joint_limit_m_s=jnp.asarray(1.0),
        feedrate_cbf_limit_m_s=jnp.asarray(1.0),
        feedrate_rate_limit_m_s=jnp.asarray(1.0),
    )

    assert np.isclose(
        float(jax_result.feedrate_endpoint_brake_limit_m_s),
        host.feedrate_endpoint_brake_limit_m_s,
        atol=2e-6,
    )
    assert np.isclose(float(jax_result.feedrate_m_s), host.feedrate_m_s, atol=2e-6)
