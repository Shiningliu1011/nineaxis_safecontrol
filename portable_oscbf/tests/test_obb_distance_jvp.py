"""The cheaper scalar linearization must preserve the original CBF Jacobian."""

import jax
jax.config.update("jax_enable_x64", True)
import jax.numpy as jnp
import numpy as np

from work import dpax_collision as dc


def test_obb_distance_linearization_matches_original_geometry():
    pairs = dc.OBB_COLLISION_PAIRS

    def original(q):
        transforms = dc.link_transforms(q)
        i, j = pairs[:, 0], pairs[:, 1]
        return jax.vmap(dc._obb_pair_distance_impl)(
            transforms[i], dc.OBB_LOCAL_CENTERS_M[i],
            dc.OBB_LOCAL_ROTATIONS[i], dc.OBB_HALF_EXTENTS_M[i],
            transforms[j], dc.OBB_LOCAL_CENTERS_M[j],
            dc.OBB_LOCAL_ROTATIONS[j], dc.OBB_HALF_EXTENTS_M[j])

    reference = jax.jit(lambda q: (original(q), jax.jacfwd(original)(q)))
    optimized = jax.jit(lambda q: (
        dc.self_collision_distances(q), jax.jacfwd(dc.self_collision_distances)(q)))
    q = np.array([0.359, -0.7, -1.396, -1.471, 1.937,
                  1.183, 0.306, 0.658, 0.125])
    perturbations = np.vstack([
        np.zeros(9), np.random.default_rng(20260907).uniform(-0.1, 0.1, (12, 9))])
    for delta in perturbations:
        expected = reference(jnp.asarray(q + delta))
        actual = optimized(jnp.asarray(q + delta))
        for result, baseline in zip(actual, expected):
            assert np.all(np.isfinite(result))
            assert np.all(np.isfinite(baseline))
            np.testing.assert_allclose(result, baseline, atol=1e-10, rtol=1e-10)
