from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from typing import NamedTuple

import jax
import jax.numpy as jnp
import numpy as np

from work.collision_parameters import CollisionPolicy
from work.fixed_environment import FixedEnvironmentModel


@dataclass(frozen=True)
class OccupiedObservation:
    support_id: int
    point_m: np.ndarray
    geometry_radius_mm: float
    measurement_error_mm: float
    point_stamp_ns: int
    timestamp_error_ns: int
    evidence_hash: bytes

    def __post_init__(self):
        point = np.array(self.point_m, dtype=np.float64, copy=True)
        if (point.shape != (3,) or not np.all(np.isfinite(point))
                or type(self.support_id) is not int or self.support_id < 0
                or type(self.point_stamp_ns) is not int or not 0 < self.point_stamp_ns <= np.iinfo(np.int64).max
                or type(self.timestamp_error_ns) is not int or self.timestamp_error_ns < 0
                or not np.isfinite(self.geometry_radius_mm) or self.geometry_radius_mm < 0
                or not np.isfinite(self.measurement_error_mm) or self.measurement_error_mm < 0
                or not isinstance(self.evidence_hash, bytes) or len(self.evidence_hash) != 32):
            raise ValueError("OccupiedObservation 的几何、时间或来源无效")
        point.setflags(write=False)
        object.__setattr__(self, "point_m", point)


def observation_hash(observations: tuple[OccupiedObservation, ...]) -> bytes:
    records = [{
        "support_id": item.support_id, "point_m": item.point_m.tolist(),
        "geometry_radius_mm": item.geometry_radius_mm, "measurement_error_mm": item.measurement_error_mm,
        "point_stamp_ns": item.point_stamp_ns, "timestamp_error_ns": item.timestamp_error_ns,
        "evidence_hash": item.evidence_hash.hex(),
    } for item in observations]
    return hashlib.sha256(json.dumps(records, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).digest()


@dataclass(frozen=True)
class MotionEvidence:
    source_hash: bytes
    validation_hash: bytes
    method: str
    device: str
    scenario: str

    def __post_init__(self):
        if (any(not isinstance(value, bytes) or len(value) != 32 or not any(value)
                for value in (self.source_hash, self.validation_hash))
                or self.source_hash == self.validation_hash
                or any(not isinstance(value, str) or not value.strip()
                       for value in (self.method, self.device, self.scenario))):
            raise ValueError("tracking 运动范围需要独立来源、验证及适用范围")


@dataclass(frozen=True)
class MotionTrack:
    track_id: int
    support_ids: tuple[int, ...]
    observation_hashes: tuple[bytes, ...]
    velocity_m_s: np.ndarray
    position_error_mm: float
    velocity_error_mm_s: float
    acceleration_bound_m_s2: float
    timestamp_error_ns: int
    updated_stamp_ns: int
    valid_until_ns: int
    valid: bool
    evidence: MotionEvidence

    def __post_init__(self):
        velocity = np.array(self.velocity_m_s, dtype=np.float64, copy=True)
        if (type(self.track_id) is not int or not 0 <= self.track_id <= np.iinfo(np.int32).max
                or not isinstance(self.support_ids, tuple) or not self.support_ids
                or any(type(item) is not int or not 0 <= item <= np.iinfo(np.int32).max for item in self.support_ids)
                or len(set(self.support_ids)) != len(self.support_ids)
                or not isinstance(self.observation_hashes, tuple)
                or len(self.observation_hashes) != len(self.support_ids)
                or any(not isinstance(value, bytes) or len(value) != 32 or not any(value)
                       for value in self.observation_hashes)
                or velocity.shape != (3,) or not np.all(np.isfinite(velocity))
                or any(not np.isfinite(value) or value < 0 for value in (
                    self.position_error_mm, self.velocity_error_mm_s, self.acceleration_bound_m_s2))
                or type(self.timestamp_error_ns) is not int or not 0 <= self.timestamp_error_ns <= np.iinfo(np.int64).max
                or any(type(value) is not int or not 0 < value <= np.iinfo(np.int64).max
                       for value in (self.updated_stamp_ns, self.valid_until_ns))
                or self.valid_until_ns < self.updated_stamp_ns or type(self.valid) is not bool
                or not isinstance(self.evidence, MotionEvidence)):
            raise ValueError("MotionTrack 的关联、时间、速度或误差无效")
        velocity.setflags(write=False)
        object.__setattr__(self, "velocity_m_s", velocity)


@dataclass(frozen=True)
class TrackingFrame:
    scene_epoch: bytes
    required_space_hash: bytes
    policy_hash: bytes
    clock_id: str
    tracks: tuple[MotionTrack, ...]

    def __post_init__(self):
        if (not isinstance(self.scene_epoch, bytes) or len(self.scene_epoch) != 16 or not any(self.scene_epoch)
                or any(not isinstance(value, bytes) or len(value) != 32 or not any(value)
                       for value in (self.required_space_hash, self.policy_hash))
                or self.clock_id != "monotonic" or not isinstance(self.tracks, tuple)
                or any(not isinstance(track, MotionTrack) for track in self.tracks)
                or len({track.track_id for track in self.tracks}) != len(self.tracks)):
            raise ValueError("TrackingFrame 的身份、clock 或 track 无效")
        members = [item for track in self.tracks for item in track.support_ids]
        if len(members) != len(set(members)):
            raise ValueError("同一 support 只能属于一个 track")

    @classmethod
    def from_dict(cls, document):
        if document["schema_version"] != 1 or type(document["schema_version"]) is not int:
            raise ValueError("tracking schema_version 必须为 1")
        if set(document) != {"schema_version", "scene_epoch", "required_space_hash", "policy_hash", "clock_id", "tracks"}:
            raise ValueError("tracking 回放字段无效")
        tracks = []
        for record in document["tracks"]:
            evidence = record["evidence"]
            tracks.append(MotionTrack(**{
                **record, "support_ids": tuple(record["support_ids"]),
                "observation_hashes": tuple(bytes.fromhex(value) for value in record["observation_hashes"]),
                "evidence": MotionEvidence(**{**evidence, "source_hash": bytes.fromhex(evidence["source_hash"]),
                                               "validation_hash": bytes.fromhex(evidence["validation_hash"])}),
            }))
        return cls(bytes.fromhex(document["scene_epoch"]), bytes.fromhex(document["required_space_hash"]),
                   bytes.fromhex(document["policy_hash"]), document["clock_id"], tuple(tracks))

    def to_dict(self):
        tracks = []
        for track in self.tracks:
            tracks.append({
                "track_id": track.track_id, "support_ids": list(track.support_ids),
                "observation_hashes": [value.hex() for value in track.observation_hashes],
                "velocity_m_s": track.velocity_m_s.tolist(), "position_error_mm": track.position_error_mm,
                "velocity_error_mm_s": track.velocity_error_mm_s,
                "acceleration_bound_m_s2": track.acceleration_bound_m_s2,
                "timestamp_error_ns": track.timestamp_error_ns, "updated_stamp_ns": track.updated_stamp_ns,
                "valid_until_ns": track.valid_until_ns, "valid": track.valid,
                "evidence": {"source_hash": track.evidence.source_hash.hex(),
                             "validation_hash": track.evidence.validation_hash.hex(),
                             "method": track.evidence.method, "device": track.evidence.device,
                             "scenario": track.evidence.scenario},
            })
        return {"schema_version": 1, "scene_epoch": self.scene_epoch.hex(),
                "required_space_hash": self.required_space_hash.hex(), "policy_hash": self.policy_hash.hex(),
                "clock_id": self.clock_id, "tracks": tracks}

    @property
    def content_hash(self):
        return hashlib.sha256(json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"),
                                         allow_nan=False).encode()).digest()

    def select(self, identities, policy, observations, now_ns):
        if (self.scene_epoch != np.asarray(identities.scene_epoch).tobytes()
                or self.required_space_hash != np.asarray(identities.required_space_hash).tobytes()
                or self.policy_hash != policy.collision_policy_hash
                or len(self.tracks) > policy.artifact.parameters["max_tracks"]["value"]):
            raise ValueError("tracking 的 scene、空间、policy 身份或容量不一致")
        records: dict[int, list[OccupiedObservation]] = {}
        for record in observations:
            records.setdefault(record.support_id, []).append(record)
        selected = {}
        for track in self.tracks:
            if (track.evidence.device != policy.artifact.device or track.evidence.scenario != policy.artifact.scenario
                    or track.updated_stamp_ns > now_ns):
                raise ValueError("tracking 的适用范围或更新时间无效")
            minimum_speed_mm_s = max(0., np.linalg.norm(track.velocity_m_s) * 1000 - track.velocity_error_mm_s)
            if track.valid and minimum_speed_mm_s > policy.artifact.parameters["untracked_speed_mm_s"]["value"]:
                raise ValueError("tracking 速度与已验证的未跟踪物理上界不相容")
            for support_id, content_hash in zip(track.support_ids, track.observation_hashes):
                if support_id not in records:
                    raise ValueError("tracking 只能关联已存在的 occupied support")
                source = tuple(records[support_id])
                if (track.valid and track.valid_until_ns >= now_ns
                        and track.updated_stamp_ns >= max(item.point_stamp_ns for item in source)
                        and content_hash == observation_hash(source)):
                    selected[support_id] = track
        return selected


class SupportMotionBounds(NamedTuple):
    policy_hash: jax.Array | np.ndarray
    speed_mm_s: jax.Array | np.ndarray
    acceleration_mm_s2: jax.Array | np.ndarray
    timestamp_error_ns: jax.Array | np.ndarray
    base_radius_mm: jax.Array | np.ndarray
    occupied_max_age_ns: jax.Array | np.ndarray

    @classmethod
    def from_policy(cls, policy: CollisionPolicy, xp=np):
        if not isinstance(policy, CollisionPolicy):
            raise TypeError("动态范围需要 CollisionPolicy")
        parameters = policy.artifact.parameters
        base_radius_mm = (np.sqrt(3) * parameters["voxel_size_mm"]["value"] / 2
                          + sum(parameters[name]["value"] for name in (
                              "lidar_range_error_mm", "lidar_incidence_error_mm", "extrinsic_translation_error_mm")))
        return cls(
            xp.asarray(np.frombuffer(policy.collision_policy_hash, dtype=np.uint8)),
            xp.asarray(parameters["untracked_speed_mm_s"]["value"], dtype=xp.float64),
            xp.asarray(parameters["untracked_acceleration_mm_s2"]["value"], dtype=xp.float64),
            xp.asarray(parameters["point_timestamp_error_ns"]["value"], dtype=xp.int64),
            xp.asarray(base_radius_mm, dtype=xp.float64),
            xp.asarray(parameters["occupied_max_age_ns"]["value"], dtype=xp.int64),
        )


class SourceMotion(NamedTuple):
    ids: np.ndarray
    points_m: np.ndarray
    radii_mm: np.ndarray
    velocity_m_s: np.ndarray
    radius_rate_mm_s: np.ndarray
    acceleration_mm_s2: np.ndarray
    stamp_ns: np.ndarray
    valid_until_ns: np.ndarray
    track_ids: np.ndarray
    history_hash: np.ndarray


class SupportTimeModel(NamedTuple):
    reference_stamp_ns: jax.Array | np.ndarray | np.generic
    scene_epoch: jax.Array | np.ndarray
    run_clock_hash: jax.Array | np.ndarray
    policy_hash: jax.Array | np.ndarray
    radius_limit_mm: jax.Array | np.ndarray | np.generic
    radius_rate_mm_s: jax.Array | np.ndarray
    radius_acceleration_mm_s2: jax.Array | np.ndarray
    valid_until_ns: jax.Array | np.ndarray
    source_mask: jax.Array | np.ndarray
    source_ids: jax.Array | np.ndarray
    source_points_m: jax.Array | np.ndarray
    source_radii_mm: jax.Array | np.ndarray
    source_velocity_m_s: jax.Array | np.ndarray
    source_radius_rate_mm_s: jax.Array | np.ndarray
    source_acceleration_mm_s2: jax.Array | np.ndarray
    source_stamp_ns: jax.Array | np.ndarray
    source_valid_until_ns: jax.Array | np.ndarray
    source_track_ids: jax.Array | np.ndarray
    source_history_hash: jax.Array | np.ndarray


def run_clock_hash(scene_epoch, clock_id):
    return hashlib.sha256(np.asarray(scene_epoch, dtype=np.uint8).tobytes() + clock_id.encode()).digest()


def time_model_specs(capacity, members):
    shape = (capacity, members)
    return {
        "reference_stamp_ns": ((), "int64"), "scene_epoch": ((16,), "uint8"),
        "run_clock_hash": ((32,), "uint8"), "policy_hash": ((32,), "uint8"),
        "radius_limit_mm": ((), "float"),
        "radius_rate_mm_s": ((capacity,), "float"),
        "radius_acceleration_mm_s2": ((capacity,), "float"), "valid_until_ns": ((capacity,), "int64"),
        "source_mask": (shape, "bool"), "source_ids": (shape, "int32"),
        "source_points_m": ((*shape, 3), "float"), "source_radii_mm": (shape, "float"),
        "source_velocity_m_s": ((*shape, 3), "float"), "source_radius_rate_mm_s": (shape, "float"),
        "source_acceleration_mm_s2": (shape, "float"), "source_stamp_ns": (shape, "int64"),
        "source_valid_until_ns": (shape, "int64"), "source_track_ids": (shape, "int32"),
        "source_history_hash": ((*shape, 32), "uint8"),
    }


def build_time_model(scene, sources: SourceMotion, groups, maximum_members, radius_limit_mm):
    count = len(scene.support_mask)
    values = {name: np.zeros(shape, dtype=np.float64 if dtype == "float" else dtype)
              for name, (shape, dtype) in time_model_specs(count, maximum_members).items()}
    values.update(reference_stamp_ns=np.int64(scene.prepared_stamp_ns),
                  scene_epoch=np.array(scene.identities.scene_epoch),
                  run_clock_hash=np.frombuffer(run_clock_hash(scene.identities.scene_epoch, scene.metadata.clock_id),
                                               dtype=np.uint8),
                  policy_hash=np.array(scene.identities.collision_policy_hash),
                  radius_limit_mm=np.float64(radius_limit_mm))
    velocities = np.zeros((count, 3))
    for slot, indices in enumerate(groups):
        selected = list(indices)
        length = len(selected)
        member_velocities = sources.velocity_m_s[selected]
        velocities[slot] = (member_velocities[0] if np.all(member_velocities == member_velocities[0])
                            else member_velocities.mean(axis=0))
        rate = float(np.max(
            np.linalg.norm(sources.velocity_m_s[selected] - velocities[slot], axis=1) * 1000
            + sources.radius_rate_mm_s[selected]))
        values["radius_rate_mm_s"][slot] = np.nextafter(rate, np.inf) if rate > 0 else 0.
        values["radius_acceleration_mm_s2"][slot] = sources.acceleration_mm_s2[selected].max()
        values["valid_until_ns"][slot] = sources.valid_until_ns[selected].min()
        values["source_mask"][slot, :length] = True
        for name, array in sources._asdict().items():
            values[f"source_{name}"][slot, :length] = array[selected]
    return SupportTimeModel(**values), velocities


def evaluate_support_time(scene, query_time_ns, xp=jnp):
    model = scene.support_time_model
    times = xp.asarray(query_time_ns)
    if model is None:
        points = xp.broadcast_to(scene.support_points_m, (times.size, *scene.support_points_m.shape))
        radii = xp.broadcast_to(scene.support_radii_mm, (times.size, *scene.support_radii_mm.shape))
        return points, radii, xp.zeros_like(radii)
    elapsed = (times - model.reference_stamp_ns).astype(xp.float64) * 1e-9
    points = scene.support_points_m[None, :, :] + elapsed[:, None, None] * scene.support_velocity_m_s[None, :, :]
    radii = (scene.support_radii_mm[None, :] + elapsed[:, None] * model.radius_rate_mm_s[None, :]
             + .5 * elapsed[:, None] ** 2 * model.radius_acceleration_mm_s2[None, :])
    rates = model.radius_rate_mm_s[None, :] + elapsed[:, None] * model.radius_acceleration_mm_s2[None, :]
    return points, radii, rates


def _fixed_members_enclosed(scene, xp):
    fixed = scene.fixed_environment_model
    if fixed is None:
        return xp.bool_(True)
    required = (xp.linalg.norm(scene.support_points_m[:, None, :]
                              - xp.asarray(fixed.support_points_m)[None, :, :], axis=-1) * 1000
                + xp.asarray(fixed.support_radii_mm)[None, :])
    active = xp.asarray(scene.support_mask) & xp.asarray(scene.support_valid_mask)
    complete = ((len(fixed.support_points_m) == fixed.support_demand)
                & xp.all(xp.any(active[:, None] & (scene.support_radii_mm[:, None] >= required), axis=0)))
    # 容量失败保留原状态码；成功场景必须完整表示模型。
    return (scene.status != 0) | complete


def time_model_valid(scene, xp=np, motion_bounds: SupportMotionBounds | None = None):
    model = scene.support_time_model
    fixed_model = scene.fixed_environment_model
    fixed_scope = (xp.bool_(True) if fixed_model is None else
                   xp.array_equal(xp.asarray(np.frombuffer(fixed_model.policy_hash, dtype=np.uint8)),
                                  scene.identities.collision_policy_hash)
                   & (fixed_model.model_revision == scene.identities.fixed_environment_revision))
    fixed_groups_covered = _fixed_members_enclosed(scene, xp)
    if model is None:
        return fixed_scope & fixed_groups_covered
    if motion_bounds is None:
        return xp.bool_(False)
    mask = xp.asarray(model.source_mask)
    support_mask = xp.asarray(scene.support_mask)
    counts = xp.sum(mask, axis=1)
    ids = xp.sort(xp.where(mask, model.source_ids, -1).ravel())
    enclosure = (xp.linalg.norm(model.source_points_m - scene.support_points_m[:, None, :], axis=-1) * 1000
                 + model.source_radii_mm)
    rate = (xp.linalg.norm(model.source_velocity_m_s - scene.support_velocity_m_s[:, None, :], axis=-1) * 1000
            + model.source_radius_rate_mm_s)
    tracked = model.source_track_ids >= 0
    matches = (xp.asarray(scene.tracks.active_mask)[None, None, :]
               & (model.source_track_ids[:, :, None] == xp.asarray(scene.tracks.ids)[None, None, :]))
    chosen = xp.argmax(matches, axis=-1)
    track_velocity = xp.asarray(scene.tracks.velocity_m_s)[chosen]
    track_acceleration = xp.asarray(scene.tracks.acceleration_bound_m_s2)[chosen] * 1000
    track_error = xp.asarray(scene.tracks.error_bound_mm)[chosen]
    track_until = xp.asarray(scene.tracks.valid_until_ns)[chosen]
    track_stamp = xp.asarray(scene.tracks.updated_stamp_ns)[chosen]
    association_valid = xp.where(tracked,
        (xp.sum(matches, axis=-1) == 1)
        & xp.all(model.source_velocity_m_s == track_velocity, axis=-1)
        & (model.source_acceleration_mm_s2 >= track_acceleration)
        & (model.source_radii_mm >= track_error)
        & (model.source_valid_until_ns <= track_until)
        & (model.source_stamp_ns <= track_stamp),
        (model.source_track_ids == -1) & xp.all(model.source_velocity_m_s == 0, axis=-1))
    first = xp.argmax(mask, axis=1)
    first_track = model.source_track_ids[xp.arange(mask.shape[0]), first]
    same_track = xp.all(~mask | (model.source_track_ids == first_track[:, None]), axis=1)
    any_track = xp.any(mask & tracked, axis=1)
    expected_status = xp.where(any_track, xp.where(same_track, 1, 2), 0)
    expected_association = xp.where(any_track & same_track, first_track, -1)
    finite = xp.bool_(True)
    nonnegative = xp.bool_(True)
    for name in ("radius_limit_mm", "radius_rate_mm_s", "radius_acceleration_mm_s2", "source_points_m", "source_radii_mm",
                 "source_velocity_m_s", "source_radius_rate_mm_s", "source_acceleration_mm_s2"):
        value = xp.asarray(getattr(model, name))
        finite &= xp.all(xp.isfinite(value))
        if name not in ("source_points_m", "source_velocity_m_s"):
            nonnegative &= xp.all(value >= 0)
    if fixed_model is None:
        fixed = xp.zeros_like(mask)
        fixed_sources_covered = xp.bool_(True)
    else:
        matches_fixed = (
            xp.all(model.source_history_hash[:, :, None, :]
                   == xp.asarray(fixed_model.support_history_hashes())[None, None, :, :], axis=-1)
            & xp.all(model.source_points_m[:, :, None, :]
                     == xp.asarray(fixed_model.support_points_m)[None, None, :, :], axis=-1)
            & (model.source_radii_mm[:, :, None] >= xp.asarray(fixed_model.support_radii_mm)[None, None, :]))
        fixed = mask & xp.any(matches_fixed, axis=-1)
        active_sources = mask & support_mask[:, None] & xp.asarray(scene.support_valid_mask)[:, None]
        fixed_sources_covered = ((scene.status != 0)
                                 | xp.all(xp.any(matches_fixed & active_sources[:, :, None], axis=(0, 1))))
    dynamic = mask & ~fixed
    untracked = dynamic & ~tracked
    age = xp.where(dynamic, (model.reference_stamp_ns - model.source_stamp_ns).astype(xp.float64) * 1e-9
                   + motion_bounds.timestamp_error_ns.astype(xp.float64) * 1e-9, 0.)
    minimum_radius = (motion_bounds.base_radius_mm + motion_bounds.speed_mm_s * age
                      + .5 * motion_bounds.acceleration_mm_s2 * age ** 2)
    minimum_rate = motion_bounds.speed_mm_s + motion_bounds.acceleration_mm_s2 * age
    history_age = xp.where(dynamic & tracked, (track_stamp - model.source_stamp_ns).astype(xp.float64) * 1e-9
                           + motion_bounds.timestamp_error_ns.astype(xp.float64) * 1e-9, 0.)
    tracked_minimum_radius = (motion_bounds.base_radius_mm + motion_bounds.speed_mm_s * history_age
                              + .5 * motion_bounds.acceleration_mm_s2 * history_age ** 2 + track_error)
    tracked_minimum_rate = track_acceleration * (model.reference_stamp_ns - track_stamp).astype(xp.float64) * 1e-9
    return (
        finite & nonnegative & fixed_scope & fixed_groups_covered & fixed_sources_covered
        & (model.radius_limit_mm > 0)
        & (model.reference_stamp_ns == scene.prepared_stamp_ns)
        & xp.array_equal(model.scene_epoch, scene.identities.scene_epoch)
        & xp.array_equal(model.policy_hash, scene.identities.collision_policy_hash)
        & xp.array_equal(motion_bounds.policy_hash, scene.identities.collision_policy_hash)
        & xp.all((counts > 0) == support_mask)
        & xp.all(~mask | (model.source_ids >= 0))
        & ~xp.any((ids[1:] == ids[:-1]) & (ids[1:] >= 0))
        & xp.all(~mask | xp.any(model.source_history_hash != 0, axis=-1))
        & xp.all(~mask | ((model.source_stamp_ns >= 0) & (model.source_stamp_ns <= model.reference_stamp_ns)))
        & xp.all(~mask | xp.where(fixed, model.source_stamp_ns == 0, model.source_stamp_ns > 0))
        & xp.all(~mask | (model.source_valid_until_ns >= model.reference_stamp_ns))
        & xp.all(~mask | (model.valid_until_ns[:, None] <= model.source_valid_until_ns))
        & xp.all(~support_mask | (model.valid_until_ns >= model.reference_stamp_ns))
        & xp.all(~mask | (scene.support_radii_mm[:, None] >= enclosure))
        & xp.all(~mask | (model.radius_rate_mm_s[:, None] >= rate))
        & xp.all(~mask | (model.radius_acceleration_mm_s2[:, None] >= model.source_acceleration_mm_s2))
        & xp.all(~mask | association_valid)
        & xp.all(~untracked | ((model.source_radii_mm >= minimum_radius)
                               & (model.source_radius_rate_mm_s >= minimum_rate)
                               & (model.source_acceleration_mm_s2 >= motion_bounds.acceleration_mm_s2)))
        & xp.all(~(dynamic & tracked) | ((model.source_radii_mm >= tracked_minimum_radius)
                                         & (model.source_radius_rate_mm_s >= tracked_minimum_rate)))
        & xp.all(~dynamic | ((model.source_valid_until_ns - model.source_stamp_ns)
                             <= motion_bounds.occupied_max_age_ns))
        & xp.all(~fixed | ((model.source_track_ids == -1) & (model.source_radius_rate_mm_s == 0)
                           & (model.source_acceleration_mm_s2 == 0)))
        & xp.all(~support_mask | (scene.support_track_status == expected_status))
        & xp.all(~support_mask | (scene.support_track_ids == expected_association))
        & xp.all(~xp.asarray(scene.tracks.active_mask) | xp.any(matches & mask[:, :, None], axis=(0, 1)))
    )


class ReachableSupportTube(NamedTuple):
    start_ns: int
    end_ns: int
    start_points_m: np.ndarray
    end_points_m: np.ndarray
    start_radii_mm: np.ndarray
    end_radii_mm: np.ndarray
    radius_rate_end_mm_s: np.ndarray
    enclosing_points_m: np.ndarray
    enclosing_radii_mm: np.ndarray
    valid_mask: np.ndarray


def reachable_support_tube(
        scene, start_ns: int, end_ns: int, *, policy: CollisionPolicy | None = None) -> ReachableSupportTube:
    from work._collision_scene import CollisionScene, PreparedScene
    if not isinstance(scene, (CollisionScene, PreparedScene)):
        raise TypeError("support tube 需要 CollisionScene 或 PreparedScene")
    if scene.fixed_environment_model is not None:
        if not isinstance(scene.fixed_environment_model, FixedEnvironmentModel):
            raise TypeError("scene.fixed_environment_model 必须是 FixedEnvironmentModel")
        scene.fixed_environment_model.verify_identity()
    motion_bounds = None if policy is None else SupportMotionBounds.from_policy(policy)
    if any(type(value) is not int or not 0 < value <= np.iinfo(np.int64).max for value in (start_ns, end_ns)):
        raise ValueError("support tube 时间必须是有效的 int64 ns")
    if end_ns < start_ns:
        raise ValueError("support tube 的结束时间必须晚于开始时间")
    points, radii, rates = evaluate_support_time(scene, np.array([start_ns, end_ns], dtype=np.int64), np)
    model = scene.support_time_model
    valid = np.asarray(scene.support_mask, dtype=bool).copy()
    if model is None:
        valid[:] = False
    else:
        valid &= (start_ns >= int(np.asarray(model.reference_stamp_ns))) & (end_ns <= np.asarray(model.valid_until_ns))
        valid &= time_model_valid(scene, np, motion_bounds)
        if policy is not None:
            valid &= float(np.asarray(model.radius_limit_mm)) == policy.artifact.parameters["support_radius_limit_mm"]["value"]
        valid &= np.asarray(model.run_clock_hash).tobytes() == run_clock_hash(scene.identities.scene_epoch, "monotonic")
        valid &= np.all(radii <= float(np.asarray(model.radius_limit_mm)), axis=0)
    if isinstance(scene, CollisionScene):
        valid &= scene.metadata.checksum == scene.compute_checksum()
        valid &= scene.metadata.required_space_covered
        valid &= scene.metadata.clock_id == "monotonic"
        valid &= np.array_equal(scene.coverage_space_hash, scene.identities.required_space_hash)
    valid &= int(np.asarray(scene.status)) == 0
    valid &= end_ns <= int(np.asarray(scene.coverage_valid_until_ns))
    valid &= np.all(np.isfinite(points), axis=(0, 2)) & np.all(np.isfinite(radii), axis=0)
    center = points.mean(axis=0)
    enclosing = np.nextafter(np.max(np.linalg.norm(points - center, axis=-1) * 1000 + radii, axis=0), np.inf)
    return ReachableSupportTube(start_ns, end_ns, points[0], points[1], radii[0], radii[1], rates[1],
                                center, enclosing, valid)
