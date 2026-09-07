"""Default device inputs may be reused; live control/obstacle state may not."""

from types import SimpleNamespace
import numpy as np

from work.jax_control_facade import JaxControlLoop


def test_default_inputs_reused_without_stale_control_or_obstacles():
    loop = JaxControlLoop(dt=0.002, enable_x64=True)
    loop._config = SimpleNamespace(d_safe_collision=0.05, obstacle_h_baseline_alpha=1.0)
    first = loop._prepare_jax_inputs()
    loop._last_u_safe = np.arange(9, dtype=float)
    second = loop._prepare_jax_inputs()
    assert second['obs_pos'] is first['obs_pos']
    np.testing.assert_array_equal(second['u_safe_prev'], loop._last_u_safe)

    active = loop._prepare_jax_inputs(
        obs_pos=np.full((8, 3), 2.0), obs_radii=np.full(8, 0.1),
        obs_enabled=np.ones(8), u_safe_prev=np.full(9, 0.25))
    np.testing.assert_array_equal(active['obs_enabled'], np.ones(8))
    np.testing.assert_array_equal(active['u_safe_prev'], np.full(9, 0.25))
    inactive = loop._prepare_jax_inputs()
    np.testing.assert_array_equal(inactive['obs_enabled'], np.zeros(8))
    np.testing.assert_array_equal(inactive['u_safe_prev'], loop._last_u_safe)

    # Returned mappings are independent; configuration changes invalidate
    # the defaults even when the controller object is reused.
    second.pop('obs_pos')
    assert 'obs_pos' in loop._prepare_jax_inputs()
    loop._config.d_safe_collision = 0.08
    updated = loop._prepare_jax_inputs()
    np.testing.assert_allclose(updated['obs_d_safe'], 0.08)
    np.testing.assert_allclose(updated['sdf_margin'], 0.08)
