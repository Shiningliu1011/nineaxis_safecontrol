"""Host/JAX parity tests for fixed arc-length null-space references."""

import _path_setup  # noqa: F401
import jax
import jax.numpy as jnp
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from path_following import PathGeometry
from path_posture_reference import PathPostureReference
from work.jax_posture_reference import (
    as_jax_path_posture_reference,
    sample_path_posture_reference_jax,
)


def _geometry() -> PathGeometry:
    return PathGeometry.from_samples(
        np.array([
            [0.0, 0.0, 0.0],
            [0.10, 0.0, 0.0],
            [0.20, 0.10, 0.0],
        ]),
        Rotation.identity(3).as_matrix(),
        np.full(3, 0.04),
        np.array([0.0, 0.5, 1.0]),
    )


def _waypoints() -> np.ndarray:
    return np.array([
        [0.10, -0.30, -0.20, -0.10, -0.50, -0.20, -0.30, -0.20, -0.10],
        [0.20, -0.20, -0.10, 0.00, -0.40, -0.10, -0.20, -0.10, 0.00],
        [0.30, -0.10, 0.00, 0.10, -0.30, 0.00, -0.10, 0.00, 0.10],
    ])


def test_posture_reference_interpolates_on_the_compacted_path_arc_length():
    geometry = _geometry()
    reference = PathPostureReference.from_path_geometry(geometry, _waypoints())
    progress = 0.13

    expected = reference.sample(progress)
    actual = sample_path_posture_reference_jax(
        as_jax_path_posture_reference(reference), jnp.asarray(progress))

    np.testing.assert_allclose(actual, expected, atol=2.0e-6)
    assert reference.q_waypoints.flags.writeable is False


def test_posture_reference_rejects_shape_and_hard_limit_errors():
    geometry = _geometry()
    with pytest.raises(ValueError, match='shape'):
        PathPostureReference.from_path_geometry(geometry, np.zeros((2, 9)))

    q_min = np.full(9, -1.0)
    q_max = np.full(9, 1.0)
    invalid = _waypoints()
    invalid[1, 4] = 1.1
    with pytest.raises(ValueError, match='hard-joint-limit'):
        PathPostureReference.from_path_geometry(
            geometry, invalid, q_min=q_min, q_max=q_max)


def test_jax_posture_sampling_reuses_one_cache_entry_for_progress_changes():
    reference = PathPostureReference.from_path_geometry(_geometry(), _waypoints())
    jreference = as_jax_path_posture_reference(reference)
    sampler = jax.jit(lambda progress: sample_path_posture_reference_jax(
        jreference, progress))

    _ = sampler(jnp.asarray(0.02))
    _ = sampler(jnp.asarray(0.18))

    assert sampler._cache_size() == 1
