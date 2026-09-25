from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import time
from typing import NamedTuple

import jax
import jax.numpy as jnp
import numpy as np

from work.kinematics_data import N_JOINTS


class CollisionStatus(IntEnum):
    OK = 0
    REVISION_PENDING = 1
    INVALID_SCENE = 2
    UNKNOWN_REQUIRED_SPACE = 3
    CAPACITY_OVERFLOW = 4
    CONSTRAINT_OVERFLOW = 5
    SOLVER_UNHEALTHY = 6
    CERTIFICATE_FAILED = 7
    DEADLINE_MISSED = 8


class QueryMode(IntEnum):
    STATE_VALIDITY = 0
    OSCBF_BARRIER = 1
    DISTANCE_MM = 2


@dataclass(frozen=True)
class CollisionSafetyConfig:
    max_support_points: int
    query_batch_size: int
    max_primitive_rows: int
    segment_batch_size: int
    query_deadline_ns: int
    certify_deadline_ns: int
    geometry_hash: bytes
    kernel_version: bytes
    collision_policy_hash: bytes

    def __post_init__(self) -> None:
        for name in (
            "max_support_points",
            "query_batch_size",
            "max_primitive_rows",
            "segment_batch_size",
            "query_deadline_ns",
            "certify_deadline_ns",
        ):
            value = getattr(self, name)
            if type(value) is not int or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        for name in ("geometry_hash", "kernel_version", "collision_policy_hash"):
            value = getattr(self, name)
            if type(value) is not bytes or len(value) != 32:
                raise ValueError(f"{name} must contain 32 bytes")


class CollisionIdentities(NamedTuple):
    scene_epoch: jax.Array
    scene_revision: jax.Array
    geometry_hash: jax.Array
    kernel_version: jax.Array
    collision_policy_hash: jax.Array


class CollisionScene(NamedTuple):
    support_points_m: jax.Array
    support_radii_mm: jax.Array
    required_clearance_mm: jax.Array
    support_velocity_m_s: jax.Array
    support_ids: jax.Array
    support_mask: jax.Array
    source_stamp_ns: jax.Array
    prepared_stamp_ns: jax.Array
    status: jax.Array


class PreparedScene(NamedTuple):
    support_points_m: jax.Array
    support_radii_mm: jax.Array
    required_clearance_mm: jax.Array
    support_velocity_m_s: jax.Array
    support_ids: jax.Array
    support_mask: jax.Array
    source_stamp_ns: jax.Array
    prepared_stamp_ns: jax.Array
    identities: CollisionIdentities
    status: jax.Array


class QueryBatch(NamedTuple):
    q: jax.Array
    active_mask: jax.Array


class SegmentBatch(NamedTuple):
    q_start: jax.Array
    q_end: jax.Array
    time_start_s: jax.Array
    time_end_s: jax.Array
    active_mask: jax.Array


class ResultHeader(NamedTuple):
    status: jax.Array
    scene_epoch: jax.Array
    scene_revision: jax.Array
    geometry_hash: jax.Array
    kernel_version: jax.Array
    collision_policy_hash: jax.Array
    started_ns: jax.Array
    completed_ns: jax.Array
    runtime_ns: jax.Array
    deadline_ns: jax.Array
    deadline_missed: jax.Array


class QueryResult(NamedTuple):
    header: ResultHeader
    query_mode: jax.Array
    state_valid_mask: jax.Array
    state_valid: jax.Array
    valid_mask: jax.Array
    barrier: jax.Array
    grad_h_q: jax.Array
    partial_h_partial_t: jax.Array
    proximity_scale: jax.Array
    primitive_pair_id: jax.Array
    solver_primal_residual: jax.Array
    solver_dual_residual: jax.Array
    solver_iteration: jax.Array
    solver_healthy: jax.Array
    distance_valid_mask: jax.Array
    distance_mm: jax.Array
    required_clearance_mm: jax.Array
    remaining_margin_mm: jax.Array
    nearest_pair_id: jax.Array
    nearest_robot_point_m: jax.Array
    nearest_support_point_m: jax.Array
    track_status: jax.Array


class SegmentCertificateBatch(NamedTuple):
    header: ResultHeader
    valid_mask: jax.Array
    certified_mask: jax.Array
    lower_bound: jax.Array
    bisection_depth: jax.Array
    failure_interval_s: jax.Array


def _array(value: object, name: str, shape: tuple[int, ...], dtypes: tuple[jnp.dtype, ...]) -> jax.Array:
    array = jnp.asarray(value)
    if array.shape != shape:
        raise ValueError(f"{name} must have shape {shape}, got {array.shape}")
    if array.dtype not in dtypes:
        expected = ", ".join(str(dtype) for dtype in dtypes)
        raise TypeError(f"{name} must have dtype {expected}, got {array.dtype}")
    return array


class CollisionSafety:
    def __init__(self, config: CollisionSafetyConfig) -> None:
        if not isinstance(config, CollisionSafetyConfig):
            raise TypeError("config must be CollisionSafetyConfig")
        jax.config.update("jax_enable_x64", True)
        if not jax.config.jax_enable_x64:
            raise RuntimeError("CollisionSafety requires JAX x64")
        self.config = config
        self._geometry_hash = jnp.asarray(np.frombuffer(config.geometry_hash, dtype=np.uint8))
        self._kernel_version = jnp.asarray(np.frombuffer(config.kernel_version, dtype=np.uint8))
        self._collision_policy_hash = jnp.asarray(
            np.frombuffer(config.collision_policy_hash, dtype=np.uint8)
        )

    def prepare_scene(
        self, scene: CollisionScene, identities: CollisionIdentities
    ) -> PreparedScene:
        if not isinstance(scene, CollisionScene):
            raise TypeError("scene must be CollisionScene")
        if not isinstance(identities, CollisionIdentities):
            raise TypeError("identities must be CollisionIdentities")
        count = self.config.max_support_points
        points = _array(
            scene.support_points_m,
            "scene.support_points_m",
            (count, 3),
            (jnp.dtype("float32"), jnp.dtype("float64")),
        ).astype(jnp.float64)
        radii = _array(
            scene.support_radii_mm, "scene.support_radii_mm", (count,), (jnp.dtype("float64"),)
        )
        clearance = _array(
            scene.required_clearance_mm,
            "scene.required_clearance_mm",
            (count,),
            (jnp.dtype("float64"),),
        )
        velocity = _array(
            scene.support_velocity_m_s,
            "scene.support_velocity_m_s",
            (count, 3),
            (jnp.dtype("float64"),),
        )
        ids = _array(scene.support_ids, "scene.support_ids", (count,), (jnp.dtype("int32"),))
        mask = _array(scene.support_mask, "scene.support_mask", (count,), (jnp.dtype("bool"),))
        source_stamp = _array(
            scene.source_stamp_ns, "scene.source_stamp_ns", (), (jnp.dtype("int64"),)
        )
        prepared_stamp = _array(
            scene.prepared_stamp_ns, "scene.prepared_stamp_ns", (), (jnp.dtype("int64"),)
        )
        source_status = _array(scene.status, "scene.status", (), (jnp.dtype("int32"),))
        epoch = _array(
            identities.scene_epoch, "identities.scene_epoch", (16,), (jnp.dtype("uint8"),)
        )
        revision = _array(
            identities.scene_revision, "identities.scene_revision", (), (jnp.dtype("int64"),)
        )
        geometry = _array(
            identities.geometry_hash, "identities.geometry_hash", (32,), (jnp.dtype("uint8"),)
        )
        kernel = _array(
            identities.kernel_version, "identities.kernel_version", (32,), (jnp.dtype("uint8"),)
        )
        policy = _array(
            identities.collision_policy_hash,
            "identities.collision_policy_hash",
            (32,),
            (jnp.dtype("uint8"),),
        )
        valid = (
            jnp.all(jnp.isfinite(points))
            & jnp.all(jnp.isfinite(radii))
            & jnp.all(jnp.isfinite(clearance))
            & jnp.all(jnp.isfinite(velocity))
            & jnp.all(radii >= 0.0)
            & jnp.all(clearance >= 0.0)
            & (source_stamp > 0)
            & (prepared_stamp >= source_stamp)
            & jnp.any(epoch != 0)
            & (revision >= 0)
            & jnp.array_equal(geometry, self._geometry_hash)
            & jnp.array_equal(kernel, self._kernel_version)
            & jnp.array_equal(policy, self._collision_policy_hash)
            & jnp.isin(source_status, jnp.asarray([int(item) for item in CollisionStatus], dtype=jnp.int32))
        )
        status = jnp.where(valid, source_status, int(CollisionStatus.INVALID_SCENE)).astype(jnp.int32)
        return PreparedScene(
            points,
            radii,
            clearance,
            velocity,
            ids,
            mask,
            source_stamp,
            prepared_stamp,
            CollisionIdentities(epoch, revision, geometry, kernel, policy),
            status,
        )

    def query(
        self, query_batch: QueryBatch, prepared_scene: PreparedScene, query_mode: QueryMode
    ) -> QueryResult:
        if not isinstance(query_batch, QueryBatch):
            raise TypeError("query_batch must be QueryBatch")
        self._validate_prepared_scene(prepared_scene)
        if type(query_mode) is not QueryMode:
            raise ValueError("query_mode must be a QueryMode value")
        count = self.config.query_batch_size
        q = _array(query_batch.q, "query_batch.q", (count, N_JOINTS), (jnp.dtype("float64"),))
        _array(query_batch.active_mask, "query_batch.active_mask", (count,), (jnp.dtype("bool"),))
        if not bool(np.all(np.isfinite(np.asarray(q)))):
            raise ValueError("query_batch.q must be finite")
        started_ns = time.perf_counter_ns()
        scene_status = CollisionStatus(int(np.asarray(prepared_scene.status)))
        status = scene_status if scene_status is not CollisionStatus.OK else CollisionStatus.SOLVER_UNHEALTHY
        rows = self.config.max_primitive_rows
        batch = count
        result_fields = dict(
            query_mode=jnp.asarray(int(query_mode), dtype=jnp.int32),
            state_valid_mask=jnp.zeros((batch,), dtype=jnp.bool_),
            state_valid=jnp.zeros((batch,), dtype=jnp.bool_),
            valid_mask=jnp.zeros((batch, rows), dtype=jnp.bool_),
            barrier=jnp.zeros((batch, rows), dtype=jnp.float64),
            grad_h_q=jnp.zeros((batch, rows, N_JOINTS), dtype=jnp.float64),
            partial_h_partial_t=jnp.zeros((batch, rows), dtype=jnp.float64),
            proximity_scale=jnp.zeros((batch, rows), dtype=jnp.float64),
            primitive_pair_id=jnp.zeros((batch, rows, 2), dtype=jnp.int32),
            solver_primal_residual=jnp.zeros((batch, rows), dtype=jnp.float64),
            solver_dual_residual=jnp.zeros((batch, rows), dtype=jnp.float64),
            solver_iteration=jnp.zeros((batch, rows), dtype=jnp.int32),
            solver_healthy=jnp.zeros((batch, rows), dtype=jnp.bool_),
            distance_valid_mask=jnp.zeros((batch,), dtype=jnp.bool_),
            distance_mm=jnp.zeros((batch,), dtype=jnp.float64),
            required_clearance_mm=jnp.zeros((batch,), dtype=jnp.float64),
            remaining_margin_mm=jnp.zeros((batch,), dtype=jnp.float64),
            nearest_pair_id=jnp.zeros((batch, 2), dtype=jnp.int32),
            nearest_robot_point_m=jnp.zeros((batch, 3), dtype=jnp.float64),
            nearest_support_point_m=jnp.zeros((batch, 3), dtype=jnp.float64),
            track_status=jnp.zeros((batch,), dtype=jnp.int32),
        )
        completed_ns = time.perf_counter_ns()
        header = self._header(
            status, prepared_scene.identities, started_ns, completed_ns, self.config.query_deadline_ns
        )
        return QueryResult(header=header, **result_fields)

    def certify(
        self, segment_batch: SegmentBatch, prepared_scene: PreparedScene
    ) -> SegmentCertificateBatch:
        if not isinstance(segment_batch, SegmentBatch):
            raise TypeError("segment_batch must be SegmentBatch")
        self._validate_prepared_scene(prepared_scene)
        count = self.config.segment_batch_size
        q_start = _array(
            segment_batch.q_start, "segment_batch.q_start", (count, N_JOINTS), (jnp.dtype("float64"),)
        )
        q_end = _array(
            segment_batch.q_end, "segment_batch.q_end", (count, N_JOINTS), (jnp.dtype("float64"),)
        )
        time_start = _array(
            segment_batch.time_start_s, "segment_batch.time_start_s", (count,), (jnp.dtype("float64"),)
        )
        time_end = _array(
            segment_batch.time_end_s, "segment_batch.time_end_s", (count,), (jnp.dtype("float64"),)
        )
        _array(
            segment_batch.active_mask, "segment_batch.active_mask", (count,), (jnp.dtype("bool"),)
        )
        if not all(
            bool(np.all(np.isfinite(np.asarray(array))))
            for array in (q_start, q_end, time_start, time_end)
        ):
            raise ValueError("segment_batch numeric fields must be finite")
        if not bool(np.all(np.asarray(time_end) >= np.asarray(time_start))):
            raise ValueError("segment_batch.time_end_s must follow time_start_s")
        started_ns = time.perf_counter_ns()
        scene_status = CollisionStatus(int(np.asarray(prepared_scene.status)))
        status = scene_status if scene_status is not CollisionStatus.OK else CollisionStatus.CERTIFICATE_FAILED
        result_fields = dict(
            valid_mask=jnp.zeros((count,), dtype=jnp.bool_),
            certified_mask=jnp.zeros((count,), dtype=jnp.bool_),
            lower_bound=jnp.zeros((count,), dtype=jnp.float64),
            bisection_depth=jnp.zeros((count,), dtype=jnp.int32),
            failure_interval_s=jnp.zeros((count, 2), dtype=jnp.float64),
        )
        completed_ns = time.perf_counter_ns()
        header = self._header(
            status, prepared_scene.identities, started_ns, completed_ns, self.config.certify_deadline_ns
        )
        return SegmentCertificateBatch(header=header, **result_fields)

    def _validate_prepared_scene(self, prepared_scene: PreparedScene) -> None:
        if not isinstance(prepared_scene, PreparedScene):
            raise TypeError("prepared_scene must be PreparedScene")
        count = self.config.max_support_points
        _array(
            prepared_scene.support_points_m,
            "prepared_scene.support_points_m",
            (count, 3),
            (jnp.dtype("float64"),),
        )
        _array(
            prepared_scene.support_radii_mm,
            "prepared_scene.support_radii_mm",
            (count,),
            (jnp.dtype("float64"),),
        )
        _array(
            prepared_scene.required_clearance_mm,
            "prepared_scene.required_clearance_mm",
            (count,),
            (jnp.dtype("float64"),),
        )
        _array(
            prepared_scene.support_velocity_m_s,
            "prepared_scene.support_velocity_m_s",
            (count, 3),
            (jnp.dtype("float64"),),
        )
        _array(
            prepared_scene.support_ids,
            "prepared_scene.support_ids",
            (count,),
            (jnp.dtype("int32"),),
        )
        _array(
            prepared_scene.support_mask,
            "prepared_scene.support_mask",
            (count,),
            (jnp.dtype("bool"),),
        )
        _array(
            prepared_scene.source_stamp_ns,
            "prepared_scene.source_stamp_ns",
            (),
            (jnp.dtype("int64"),),
        )
        _array(
            prepared_scene.prepared_stamp_ns,
            "prepared_scene.prepared_stamp_ns",
            (),
            (jnp.dtype("int64"),),
        )
        if not isinstance(prepared_scene.identities, CollisionIdentities):
            raise TypeError("prepared_scene.identities must be CollisionIdentities")
        _array(
            prepared_scene.identities.scene_epoch,
            "prepared_scene.identities.scene_epoch",
            (16,),
            (jnp.dtype("uint8"),),
        )
        _array(
            prepared_scene.identities.scene_revision,
            "prepared_scene.identities.scene_revision",
            (),
            (jnp.dtype("int64"),),
        )
        for name in ("geometry_hash", "kernel_version", "collision_policy_hash"):
            _array(
                getattr(prepared_scene.identities, name),
                f"prepared_scene.identities.{name}",
                (32,),
                (jnp.dtype("uint8"),),
            )
        _array(prepared_scene.status, "prepared_scene.status", (), (jnp.dtype("int32"),))
        CollisionStatus(int(np.asarray(prepared_scene.status)))

    @staticmethod
    def _header(
        status: CollisionStatus,
        identities: CollisionIdentities,
        started_ns: int,
        completed_ns: int,
        deadline_ns: int,
    ) -> ResultHeader:
        runtime_ns = completed_ns - started_ns
        missed = runtime_ns > deadline_ns
        return ResultHeader(
            status=jnp.asarray(
                int(CollisionStatus.DEADLINE_MISSED if missed else status), dtype=jnp.int32
            ),
            scene_epoch=identities.scene_epoch,
            scene_revision=identities.scene_revision,
            geometry_hash=identities.geometry_hash,
            kernel_version=identities.kernel_version,
            collision_policy_hash=identities.collision_policy_hash,
            started_ns=jnp.asarray(started_ns, dtype=jnp.int64),
            completed_ns=jnp.asarray(completed_ns, dtype=jnp.int64),
            runtime_ns=jnp.asarray(runtime_ns, dtype=jnp.int64),
            deadline_ns=jnp.asarray(deadline_ns, dtype=jnp.int64),
            deadline_missed=jnp.asarray(missed, dtype=jnp.bool_),
        )
