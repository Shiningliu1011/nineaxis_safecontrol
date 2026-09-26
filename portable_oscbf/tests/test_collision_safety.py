from __future__ import annotations

from pathlib import Path
from dataclasses import replace
import os
import subprocess
import sys

import jax
import jax.numpy as jnp
import numpy as np
import pytest

from collision_parameter_inputs import DEVICE, SCENARIO, parameter_payload
from work.collision_parameters import CollisionParameterArtifact, CollisionPolicy

from work.collision_safety import (
    CollisionIdentities,
    CollisionSafety,
    CollisionSafetyConfig,
    CollisionScene,
    CollisionStatus,
    QueryBatch,
    QueryMode,
    SegmentBatch,
)


def _config(**changes: object) -> CollisionSafetyConfig:
    payload = parameter_payload()
    for name, value in changes.items():
        payload["parameters"][name]["value"] = value
    artifact = CollisionParameterArtifact.create(payload, device=DEVICE, scenario=SCENARIO)
    return CollisionSafetyConfig.from_policy(
        CollisionPolicy(artifact, geometry_hash=b"g" * 32, kernel_version=b"k" * 32)
    )


def _identities(config: CollisionSafetyConfig | None = None) -> CollisionIdentities:
    config = config or _config()
    return CollisionIdentities(
        scene_epoch=jnp.asarray([1] + [0] * 15, dtype=jnp.uint8),
        scene_revision=jnp.asarray(7, dtype=jnp.int64),
        geometry_hash=jnp.asarray(np.frombuffer(b"g" * 32, dtype=np.uint8)),
        kernel_version=jnp.asarray(np.frombuffer(b"k" * 32, dtype=np.uint8)),
        collision_policy_hash=jnp.asarray(np.frombuffer(config.collision_policy_hash, dtype=np.uint8)),
    )


def _scene(status: CollisionStatus = CollisionStatus.OK) -> CollisionScene:
    return CollisionScene(
        support_points_m=jnp.asarray(
            [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6], [0.0, 0.0, 0.0]], dtype=jnp.float32
        ),
        support_radii_mm=jnp.asarray([10.0, 20.0, 0.0], dtype=jnp.float64),
        required_clearance_mm=jnp.asarray([30.0, 30.0, 0.0], dtype=jnp.float64),
        support_velocity_m_s=jnp.zeros((3, 3), dtype=jnp.float64),
        support_ids=jnp.asarray([11, 12, 0], dtype=jnp.int32),
        support_mask=jnp.asarray([True, True, False]),
        source_stamp_ns=jnp.asarray(100, dtype=jnp.int64),
        prepared_stamp_ns=jnp.asarray(120, dtype=jnp.int64),
        status=jnp.asarray(int(status), dtype=jnp.int32),
    )


def _query_batch() -> QueryBatch:
    return QueryBatch(
        q=jnp.zeros((2, 9), dtype=jnp.float64),
        active_mask=jnp.asarray([True, False]),
    )


def _segment_batch() -> SegmentBatch:
    return SegmentBatch(
        q_start=jnp.zeros((2, 9), dtype=jnp.float64),
        q_end=jnp.ones((2, 9), dtype=jnp.float64),
        time_start_s=jnp.asarray([0.0, 0.0], dtype=jnp.float64),
        time_end_s=jnp.asarray([0.1, 0.0], dtype=jnp.float64),
        active_mask=jnp.asarray([True, False]),
    )


def test_prepare_scene_converts_lidar_coordinates_and_compiles() -> None:
    module = CollisionSafety(_config())
    prepared = jax.jit(module.prepare_scene)(_scene(), _identities())

    assert prepared.support_points_m.dtype == jnp.float64
    assert prepared.support_points_m.shape == (3, 3)
    assert prepared.support_radii_mm.dtype == jnp.float64
    assert int(prepared.status) == CollisionStatus.OK
    assert int(prepared.identities.scene_revision) == 7
    assert np.array_equal(np.asarray(prepared.support_mask), [True, True, False])


@pytest.mark.parametrize("mode", list(QueryMode))
def test_query_returns_fixed_fail_closed_result_for_each_mode(mode: QueryMode) -> None:
    module = CollisionSafety(_config())
    prepared = module.prepare_scene(_scene(), _identities())
    result = module.query(_query_batch(), prepared, mode)
    compiled_result = jax.jit(lambda value: value)(result)

    expected = CollisionStatus.DEADLINE_MISSED if bool(result.header.deadline_missed) else CollisionStatus.SOLVER_UNHEALTHY
    assert int(result.header.status) == expected
    assert int(result.query_mode) == mode
    assert result.valid_mask.shape == (2, 4)
    assert result.grad_h_q.shape == (2, 4, 9)
    assert result.distance_mm.shape == (2,)
    assert result.barrier.dtype == jnp.float64
    assert result.grad_h_q.dtype == jnp.float64
    assert not np.any(np.asarray(result.valid_mask))
    assert not np.any(np.asarray(result.state_valid_mask))
    assert not np.any(np.asarray(result.distance_valid_mask))
    assert not np.any(np.asarray(compiled_result.valid_mask))
    assert int(result.header.scene_revision) == 7
    assert np.array_equal(np.asarray(result.header.geometry_hash), np.frombuffer(b"g" * 32, dtype=np.uint8))
    assert int(result.header.started_ns) <= int(result.header.completed_ns)
    assert int(result.header.runtime_ns) == (
        int(result.header.completed_ns) - int(result.header.started_ns)
    )
    assert bool(result.header.deadline_missed) == (int(result.header.runtime_ns) > module.config.query_deadline_ns)


def test_certify_returns_fixed_unproved_result() -> None:
    module = CollisionSafety(_config())
    prepared = module.prepare_scene(_scene(), _identities())
    result = module.certify(_segment_batch(), prepared)
    compiled_result = jax.jit(lambda value: value)(result)

    assert int(result.header.status) == CollisionStatus.CERTIFICATE_FAILED
    assert result.valid_mask.shape == (2,)
    assert result.lower_bound.shape == (2,)
    assert result.failure_interval_s.shape == (2, 2)
    assert result.lower_bound.dtype == jnp.float64
    assert not np.any(np.asarray(result.valid_mask))
    assert not np.any(np.asarray(result.certified_mask))
    assert not np.any(np.asarray(compiled_result.certified_mask))


@pytest.mark.parametrize("status", list(CollisionStatus))
def test_all_public_status_codes_preserve_fail_closed_masks(status: CollisionStatus) -> None:
    module = CollisionSafety(_config())
    prepared = module.prepare_scene(_scene(status), _identities())
    query = module.query(_query_batch(), prepared, QueryMode.OSCBF_BARRIER)
    certificate = module.certify(_segment_batch(), prepared)

    assert int(prepared.status) == status
    expected_query = CollisionStatus.SOLVER_UNHEALTHY if status is CollisionStatus.OK else status
    if bool(query.header.deadline_missed):
        expected_query = CollisionStatus.DEADLINE_MISSED
    expected_certificate = CollisionStatus.CERTIFICATE_FAILED if status is CollisionStatus.OK else status
    assert int(query.header.status) == expected_query
    assert int(certificate.header.status) == expected_certificate
    assert not np.any(np.asarray(query.valid_mask))
    assert not np.any(np.asarray(certificate.valid_mask))


def test_deadline_miss_overrides_unavailable_kernel_status() -> None:
    module = CollisionSafety(_config(query_deadline_ns=1, certify_deadline_ns=1))
    prepared = module.prepare_scene(_scene(), _identities(module.config))
    query = module.query(_query_batch(), prepared, QueryMode.STATE_VALIDITY)
    certificate = module.certify(_segment_batch(), prepared)

    assert int(query.header.status) == CollisionStatus.DEADLINE_MISSED
    assert int(certificate.header.status) == CollisionStatus.DEADLINE_MISSED
    assert bool(query.header.deadline_missed)
    assert bool(certificate.header.deadline_missed)
    assert not np.any(np.asarray(query.valid_mask))
    assert not np.any(np.asarray(certificate.valid_mask))


def test_invalid_scene_values_and_identity_return_invalid_scene() -> None:
    module = CollisionSafety(_config())
    invalid_points = _scene()._replace(
        support_points_m=jnp.asarray([[jnp.nan, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=jnp.float32)
    )
    invalid_identity = _identities()._replace(
        geometry_hash=jnp.asarray(np.frombuffer(b"x" * 32, dtype=np.uint8))
    )

    for scene, identities in ((invalid_points, _identities()), (_scene(), invalid_identity)):
        prepared = module.prepare_scene(scene, identities)
        assert int(prepared.status) == CollisionStatus.INVALID_SCENE
        query = module.query(_query_batch(), prepared, QueryMode.OSCBF_BARRIER)
        assert int(query.header.status) == (
            CollisionStatus.DEADLINE_MISSED if bool(query.header.deadline_missed) else CollisionStatus.INVALID_SCENE
        )


def test_invalid_shape_dtype_mode_and_numeric_batch_fail_at_interface() -> None:
    module = CollisionSafety(_config())
    prepared = module.prepare_scene(_scene(), _identities())

    with pytest.raises(ValueError, match="scene.support_points_m"):
        module.prepare_scene(_scene()._replace(support_points_m=jnp.zeros((2, 3), dtype=jnp.float32)), _identities())
    with pytest.raises(TypeError, match="scene.support_radii_mm"):
        module.prepare_scene(_scene()._replace(support_radii_mm=jnp.zeros((3,), dtype=jnp.float32)), _identities())
    with pytest.raises(ValueError, match="query_batch.q"):
        module.query(_query_batch()._replace(q=jnp.zeros((1, 9), dtype=jnp.float64)), prepared, QueryMode.OSCBF_BARRIER)
    with pytest.raises(ValueError, match="query_mode"):
        module.query(_query_batch(), prepared, 99)
    with pytest.raises(ValueError, match="query_batch.q must be finite"):
        module.query(_query_batch()._replace(q=jnp.full((2, 9), jnp.nan)), prepared, QueryMode.OSCBF_BARRIER)
    with pytest.raises(ValueError, match="segment_batch.q_start"):
        module.certify(_segment_batch()._replace(q_start=jnp.zeros((1, 9), dtype=jnp.float64)), prepared)
    with pytest.raises(ValueError, match="segment_batch numeric fields"):
        module.certify(_segment_batch()._replace(time_end_s=jnp.asarray([jnp.inf, 0.0])), prepared)
    with pytest.raises(ValueError, match="prepared_scene.support_velocity_m_s"):
        module.query(
            _query_batch(),
            prepared._replace(support_velocity_m_s=jnp.zeros((2, 3), dtype=jnp.float64)),
            QueryMode.OSCBF_BARRIER,
        )


@pytest.mark.parametrize(
    "changes",
    [
        {"max_support_points": 0},
        {"query_batch_size": True},
        {"query_deadline_ns": -1},
        {"geometry_hash": b"short"},
    ],
)
def test_invalid_configuration_is_rejected(changes: dict[str, object]) -> None:
    with pytest.raises(ValueError):
        CollisionSafety(replace(_config(), **changes))


def test_collision_safety_operations_run_without_ros() -> None:
    project = Path(__file__).resolve().parents[1]
    environment = os.environ.copy()
    environment["PYTHONPATH"] = f"{project}:{environment.get('PYTHONPATH', '')}"
    code = (
        "import sys; "
        "from work.collision_safety import CollisionSafety, QueryMode; "
        "from test_collision_safety import _config, _scene, _identities, _query_batch, _segment_batch; "
        "module = CollisionSafety(_config()); "
        "prepared = module.prepare_scene(_scene(), _identities()); "
        "assert int(module.query(_query_batch(), prepared, QueryMode.OSCBF_BARRIER).header.status) != 0; "
        "assert int(module.certify(_segment_batch(), prepared).header.status) != 0; "
        "assert 'rclpy' not in sys.modules"
    )
    completed = subprocess.run(
        [sys.executable, "-c", code],
        cwd=project / "tests",
        env=environment,
        check=True,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0
