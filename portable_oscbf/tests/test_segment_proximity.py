"""Specialized 2-D solve retains dpax's regularization and envelope gradient."""

import jax
jax.config.update('jax_enable_x64', True)
import jax.numpy as jnp
import numpy as np

from dpax.endpoints import proximity
from work import dpax_collision as dc


def test_segment_proximity_matches_dpax_values_and_gradients():
    rng = np.random.default_rng(20260907)
    endpoints = rng.normal(size=(512, 4, 3))
    # Parallel and nearly parallel edges exercise the regularized solve.
    endpoints[:64, 3] = endpoints[:64, 2] + endpoints[:64, 1] - endpoints[:64, 0]
    endpoints[64:128, 3] = (endpoints[64:128, 2] + endpoints[64:128, 1]
                            - endpoints[64:128, 0] + 1e-7)
    endpoints[128:256] *= 0.01

    def evaluate(fn):
        def one(points):
            return fn(0.0, points[0], points[1], 0.0, points[2], points[3])
        return jax.jit(jax.vmap(jax.value_and_grad(one)))(jnp.asarray(endpoints))

    expected = evaluate(proximity)
    actual = evaluate(dc._segment_proximity)
    for result, baseline in zip(actual, expected):
        assert np.all(np.isfinite(result))
        np.testing.assert_allclose(result, baseline, atol=1e-10, rtol=1e-10)
