from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import IntEnum
import hashlib
import io
import json
import time
from typing import NamedTuple

import jax
import jax.numpy as jnp
import numpy as np

from work.collision_parameters import CollisionPolicy


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


class SupportTrackStatus(IntEnum):
    UNTRACKED = 0
    TRACKED = 1


class CollisionIdentities(NamedTuple):
    scene_epoch: jax.Array
    scene_revision: jax.Array
    geometry_hash: jax.Array
    kernel_version: jax.Array
    collision_policy_hash: jax.Array
    fixed_environment_revision: jax.Array
    required_space_hash: jax.Array


@dataclass(frozen=True, slots=True)
class SceneMetadata:
    layout_version: int
    frame_id: str
    point_unit: str
    radius_unit: str
    velocity_unit: str
    time_unit: str
    clock_id: str
    support_count: int
    track_count: int
    required_space_covered: bool
    checksum: bytes


class SceneTracks(NamedTuple):
    ids: jax.Array
    active_mask: jax.Array
    valid_mask: jax.Array
    velocity_m_s: jax.Array
    acceleration_bound_m_s2: jax.Array
    error_bound_mm: jax.Array
    updated_stamp_ns: jax.Array
    valid_until_ns: jax.Array


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
    support_track_status: jax.Array
    support_valid_mask: jax.Array
    support_track_ids: jax.Array
    tracks: SceneTracks
    identities: CollisionIdentities
    coverage_space_hash: jax.Array
    coverage_valid_until_ns: jax.Array
    metadata: SceneMetadata

    def compute_checksum(self) -> bytes:
        # 发布者与读取者共用标准 JSON/NPY 编码，保留原始 dtype 和 shape。
        metadata = asdict(self.metadata)
        del metadata["checksum"]
        digest = hashlib.sha256(json.dumps(
            metadata, sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode("utf-8"))
        for name, value in self._asdict().items():
            if name == "metadata":
                continue
            entries = value._asdict().items() if isinstance(value, tuple) else [("", value)]
            for field, array in entries:
                digest.update(json.dumps([name, field], separators=(",", ":")).encode("utf-8"))
                buffer = io.BytesIO()
                np.save(buffer, np.array(array, copy=True, order="C"), allow_pickle=False)
                digest.update(buffer.getvalue())
        return digest.digest()


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
    support_track_status: jax.Array
    support_valid_mask: jax.Array
    support_track_ids: jax.Array
    tracks: SceneTracks
    valid_until_ns: jax.Array
    coverage_valid_until_ns: jax.Array
    tracking_valid_until_ns: jax.Array


def _identity_specs() -> dict:
    return {
        "scene_epoch": ((16,), "uint8"), "scene_revision": ((), "int64"),
        "geometry_hash": ((32,), "uint8"), "kernel_version": ((32,), "uint8"),
        "collision_policy_hash": ((32,), "uint8"),
        "fixed_environment_revision": ((), "int64"), "required_space_hash": ((32,), "uint8"),
    }


def _track_specs(count: int) -> dict:
    return {
        "ids": ((count,), "int32"), "active_mask": ((count,), "bool"),
        "valid_mask": ((count,), "bool"), "velocity_m_s": ((count, 3), "float"),
        "acceleration_bound_m_s2": ((count,), "float"), "error_bound_mm": ((count,), "float"),
        "updated_stamp_ns": ((count,), "int64"), "valid_until_ns": ((count,), "int64"),
    }


def _support_specs(count: int) -> dict:
    return {
        "support_points_m": ((count, 3), "float"), "support_radii_mm": ((count,), "float"),
        "required_clearance_mm": ((count,), "float"), "support_velocity_m_s": ((count, 3), "float"),
        "support_ids": ((count,), "int32"), "support_mask": ((count,), "bool"),
        "source_stamp_ns": ((), "int64"), "prepared_stamp_ns": ((), "int64"),
        "status": ((), "int32"), "support_track_status": ((count,), "int32"),
        "support_valid_mask": ((count,), "bool"), "support_track_ids": ((count,), "int32"),
    }


def _copy_arrays(value: tuple, specs: dict, prefix: str) -> dict:
    result = {}
    for name, (shape, dtype) in specs.items():
        array = np.array(getattr(value, name), copy=True, order="C")
        if array.shape != shape:
            raise ValueError(f"{prefix}.{name} must have shape {shape}, got {array.shape}")
        allowed = (np.dtype("float32"), np.dtype("float64")) if dtype == "float" else (np.dtype(dtype),)
        if array.dtype not in allowed:
            raise TypeError(f"{prefix}.{name} must have dtype {allowed}, got {array.dtype}")
        result[name] = array
    return result


def _snapshot(scene: CollisionScene, expected: CollisionIdentities, parameters) -> tuple:
    if not isinstance(scene, CollisionScene):
        raise TypeError("scene must be CollisionScene")
    if not isinstance(expected, CollisionIdentities) or not isinstance(scene.identities, CollisionIdentities):
        raise TypeError("identities must be CollisionIdentities")
    if not isinstance(scene.tracks, SceneTracks):
        raise TypeError("scene.tracks must be SceneTracks")
    if not isinstance(scene.metadata, SceneMetadata):
        raise TypeError("scene.metadata must be SceneMetadata")
    for name, value in asdict(scene.metadata).items():
        expected_type = (bytes if name == "checksum" else bool if name == "required_space_covered"
                         else int if name in ("layout_version", "support_count", "track_count") else str)
        if type(value) is not expected_type:
            raise TypeError(f"scene.metadata.{name} must be {expected_type.__name__}")
    values = _copy_arrays(scene, {
        **_support_specs(parameters["max_support_points"]["value"]),
        "coverage_space_hash": ((32,), "uint8"), "coverage_valid_until_ns": ((), "int64"),
    }, "scene")
    values["tracks"] = SceneTracks(**_copy_arrays(
        scene.tracks, _track_specs(parameters["max_tracks"]["value"]), "scene.tracks"))
    values["identities"] = CollisionIdentities(**_copy_arrays(scene.identities, _identity_specs(), "scene.identities"))
    values["metadata"] = scene.metadata
    return CollisionScene(**values), CollisionIdentities(**_copy_arrays(expected, _identity_specs(), "identities"))


def _tracks_valid(scene, now_ns, xp=np):
    tracks = scene.tracks
    active = xp.asarray(tracks.active_mask)
    ids = xp.asarray(tracks.ids)
    support_active = xp.asarray(scene.support_mask)
    tracked = xp.asarray(scene.support_track_status) == int(SupportTrackStatus.TRACKED)
    association = xp.asarray(scene.support_track_ids)
    sorted_ids = xp.sort(xp.where(active, ids, -1))
    matches = active[None, :] & (association[:, None] == ids[None, :])
    selected = xp.argmax(matches, axis=1)
    association_valid = xp.where(
        tracked,
        (xp.sum(matches, axis=1) == 1)
        & xp.all(xp.asarray(scene.support_velocity_m_s) == xp.asarray(tracks.velocity_m_s)[selected], axis=1)
        & (xp.asarray(scene.support_radii_mm) >= xp.asarray(tracks.error_bound_mm)[selected]),
        (association == -1) & xp.all(xp.asarray(scene.support_velocity_m_s) == 0, axis=1),
    )
    return (
        xp.all(~active | xp.asarray(tracks.valid_mask))
        & xp.all(~active | (ids >= 0))
        & ~xp.any((sorted_ids[1:] == sorted_ids[:-1]) & (sorted_ids[1:] >= 0))
        & xp.all(~active | ((xp.asarray(tracks.updated_stamp_ns) > 0)
                            & (xp.asarray(tracks.updated_stamp_ns) <= scene.source_stamp_ns)
                            & (xp.asarray(tracks.valid_until_ns) >= now_ns)))
        & xp.all(xp.isfinite(tracks.velocity_m_s))
        & xp.all(xp.isfinite(tracks.acceleration_bound_m_s2))
        & xp.all(xp.isfinite(tracks.error_bound_mm))
        & xp.all(xp.asarray(tracks.acceleration_bound_m_s2) >= 0)
        & xp.all(xp.asarray(tracks.error_bound_mm) >= 0)
        & xp.all(~support_active | association_valid)
    )


def _admission_status(scene: CollisionScene, expected: CollisionIdentities, policy: CollisionPolicy,
                      now_ns: int, checksum_valid: bool) -> tuple[CollisionStatus, int]:
    metadata = scene.metadata
    parameters = policy.artifact.parameters
    identity = scene.identities
    source, prepared = int(scene.source_stamp_ns), int(scene.prepared_stamp_ns)
    preparation_budget = parameters["transport_delay_ns"]["value"] + parameters["scene_preparation_delay_ns"]["value"]
    valid_until = source + preparation_budget + parameters["control_delay_ns"]["value"]
    active = scene.support_mask
    ids = scene.support_ids[active]
    valid = (
        (metadata.layout_version, metadata.frame_id, metadata.point_unit, metadata.radius_unit,
         metadata.velocity_unit, metadata.time_unit, metadata.clock_id)
        == (1, "base_link", "m", "mm", "m/s", "ns", "monotonic")
        and checksum_valid
        and all(np.array_equal(a, b) for a, b in zip(identity, expected))
        and any(identity.scene_epoch) and int(identity.scene_revision) >= 0
        and int(identity.fixed_environment_revision) >= 0 and any(identity.required_space_hash)
        and all(np.asarray(getattr(identity, name)).tobytes() == getattr(policy, name)
                for name in ("geometry_hash", "kernel_version", "collision_policy_hash"))
        and source > 0 and source <= prepared <= now_ns and prepared - source <= preparation_budget
        and now_ns <= valid_until <= np.iinfo(np.int64).max
        and int(scene.status) in set(CollisionStatus)
        and metadata.support_count >= 0 and metadata.track_count >= 0
        and all(np.all(np.isfinite(value)) for value in (
            scene.support_points_m, scene.support_radii_mm, scene.required_clearance_mm, scene.support_velocity_m_s))
        and np.all(scene.support_radii_mm >= 0) and np.all(scene.required_clearance_mm >= 0)
        and np.all(scene.required_clearance_mm[active] == parameters["environment_clearance_mm"]["value"])
        and np.all(scene.support_radii_mm[active] <= parameters["support_radius_limit_mm"]["value"])
        and np.all(ids >= 0) and len(np.unique(ids)) == len(ids)
        and np.all(~active | scene.support_valid_mask)
        and np.all(np.isin(scene.support_track_status, [int(item) for item in SupportTrackStatus]))
    )
    if not valid:
        return CollisionStatus.INVALID_SCENE, 0
    if (metadata.support_count > parameters["max_support_points"]["value"]
            or metadata.track_count > parameters["max_tracks"]["value"]):
        return CollisionStatus.CAPACITY_OVERFLOW, valid_until
    if (metadata.support_count != int(np.sum(active))
            or metadata.track_count != int(np.sum(scene.tracks.active_mask))
            or not _tracks_valid(scene, now_ns)):
        return CollisionStatus.INVALID_SCENE, valid_until
    if (not metadata.required_space_covered
            or not np.array_equal(scene.coverage_space_hash, identity.required_space_hash)
            or int(scene.coverage_valid_until_ns) < now_ns):
        return CollisionStatus.UNKNOWN_REQUIRED_SPACE, valid_until
    return CollisionStatus(int(scene.status)), valid_until


def _prepare_scene(scene: CollisionScene, identities: CollisionIdentities, policy: CollisionPolicy) -> PreparedScene:
    started = time.monotonic_ns()
    parameters = policy.artifact.parameters
    snapshot, expected = _snapshot(scene, identities, parameters)
    checksum_valid = len(snapshot.metadata.checksum) == 32 and snapshot.metadata.checksum == snapshot.compute_checksum()
    # 数值规则使用与查询相同的 float64 值；checksum 保留发布者原始字节。
    snapshot = snapshot._replace(**{
        name: getattr(snapshot, name).astype(np.float64, copy=False)
        for name, (_, dtype) in _support_specs(parameters["max_support_points"]["value"]).items()
        if dtype == "float"
    }, tracks=snapshot.tracks._replace(**{
        name: getattr(snapshot.tracks, name).astype(np.float64, copy=False)
        for name, (_, dtype) in _track_specs(parameters["max_tracks"]["value"]).items()
        if dtype == "float"
    }))
    status, valid_until = _admission_status(snapshot, expected, policy, time.monotonic_ns(), checksum_valid)
    tracking_until = int(np.min(np.where(snapshot.tracks.active_mask, snapshot.tracks.valid_until_ns,
                                        np.iinfo(np.int64).max)))
    coverage_until = int(snapshot.coverage_valid_until_ns)
    preparation_deadline = started + parameters["scene_preparation_deadline_ns"]["value"]
    transfer_started = time.monotonic_ns()
    transfer_deadline = transfer_started + parameters["device_transfer_deadline_ns"]["value"]

    def device(array):
        return jnp.array(array, copy=True)

    values = {name: device(getattr(snapshot, name)) for name in _support_specs(parameters["max_support_points"]["value"])}
    values["identities"] = CollisionIdentities(*(device(value) for value in snapshot.identities))
    checked_tracks = snapshot.tracks._replace(
        valid_mask=snapshot.tracks.valid_mask & snapshot.tracks.active_mask & (status is CollisionStatus.OK))
    values["tracks"] = SceneTracks(*(device(value) for value in checked_tracks))
    values["support_valid_mask"] = device(snapshot.support_valid_mask & snapshot.support_mask & (status is CollisionStatus.OK))
    values["status"] = jnp.int32(status)
    values["valid_until_ns"] = jnp.int64(valid_until)
    values["coverage_valid_until_ns"] = device(snapshot.coverage_valid_until_ns)
    values["tracking_valid_until_ns"] = jnp.int64(tracking_until)
    result = PreparedScene(**values)
    jax.block_until_ready(result)
    completed = time.monotonic_ns()
    if completed > preparation_deadline or completed > transfer_deadline:
        status = CollisionStatus.DEADLINE_MISSED
    elif status is CollisionStatus.OK:
        if completed > valid_until or completed > tracking_until:
            status = CollisionStatus.INVALID_SCENE
        elif completed > coverage_until:
            status = CollisionStatus.UNKNOWN_REQUIRED_SPACE
    else:
        return result
    if status is CollisionStatus.OK:
        return result
    values["status"] = jnp.int32(status)
    if status is not CollisionStatus.OK:
        values["support_valid_mask"] = device(np.zeros_like(snapshot.support_mask))
        values["tracks"] = values["tracks"]._replace(valid_mask=device(np.zeros_like(snapshot.tracks.active_mask)))
    jax.block_until_ready(values)
    return PreparedScene(**values)
