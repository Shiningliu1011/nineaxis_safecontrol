from __future__ import annotations

import hashlib
import json
import time
from collections.abc import Mapping
from dataclasses import dataclass, replace
from enum import IntEnum
from types import MappingProxyType

import numpy as np

from work.collision_safety import (
    CollisionIdentities,
    CollisionSafetyConfig,
    CollisionScene,
    CollisionStatus,
    SceneMetadata,
    SceneTracks,
)
from work.dynamic_support import (
    OccupiedObservation,
    SourceMotion,
    TrackingFrame,
    build_time_model,
    observation_hash,
)
from work.fixed_environment import FixedEnvironmentModel


class VoxelState(IntEnum):
    UNKNOWN = 0
    FREE = 1
    OCCUPIED = 2


class RayReturn(IntEnum):
    HIT = 0
    NO_RETURN = 1
    OCCLUDED = 2
    AMBIGUOUS = 3


def _readonly(value, dtype, shape, name):
    original = np.asarray(value)
    target = np.dtype(dtype)
    if target.kind in "iu":
        limits: np.iinfo = np.iinfo(target)
        if (original.dtype.kind not in "iu" or (original.size and (
                original.min().item() < limits.min or original.max().item() > limits.max))):
            raise ValueError(f"{name} 必须是范围有效的整数数组")
    if target.kind == "b" and original.dtype.kind != "b":
        raise ValueError(f"{name} 必须是布尔数组")
    array = np.array(value, dtype=dtype, copy=True)
    if array.shape != shape or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} 的形状或数值无效")
    array.setflags(write=False)
    return array


@dataclass(frozen=True)
class LidarRayBatch:
    origins_m: np.ndarray
    endpoints_m: np.ndarray
    point_stamp_ns: np.ndarray
    return_kind: np.ndarray
    scan_id: int
    frame_id: str = "base_link"
    clock_id: str = "monotonic"
    visible_free_until_m: np.ndarray | None = None
    unknown_start_m: np.ndarray | None = None
    unknown_end_m: np.ndarray | None = None
    unknown_radius_mm: np.ndarray | None = None
    input_valid: bool = True
    filter_identity: str = ""
    invalid_reason: str = "ok"

    def __post_init__(self):
        count = len(self.origins_m)
        for name, dtype, shape in (
            ("origins_m", np.float64, (count, 3)),
            ("endpoints_m", np.float64, (count, 3)),
            ("point_stamp_ns", np.int64, (count,)),
            ("return_kind", np.int32, (count,)),
        ):
            object.__setattr__(self, name, _readonly(getattr(self, name), dtype, shape, name))
        if (type(self.scan_id) is not int or self.scan_id < 0
                or self.frame_id != "base_link" or self.clock_id != "monotonic"
                or np.any(self.point_stamp_ns <= 0)
                or not np.all(np.isin(self.return_kind, list(RayReturn)))
                or np.any(np.linalg.norm(self.endpoints_m - self.origins_m, axis=1) == 0)):
            raise ValueError("LiDAR 射线的身份、坐标、时间或类型无效")
        for name in ("visible_free_until_m", "unknown_start_m", "unknown_end_m", "unknown_radius_mm"):
            value = getattr(self, name)
            if value is not None:
                value = _readonly(value, np.float64, (count,), name)
                if np.any(value < 0):
                    raise ValueError(f"{name} 必须非负")
                object.__setattr__(self, name, value)
        if (type(self.input_valid) is not bool or not self.invalid_reason.strip()
                or (self.filter_identity and len(self.filter_identity) != 64)
                or (self.unknown_start_m is None) != (self.unknown_end_m is None)
                or (self.unknown_radius_mm is not None and self.unknown_start_m is None)
                or (self.visible_free_until_m is not None and not self.filter_identity)
                or (not self.input_valid and self.invalid_reason == "ok")):
            raise ValueError("self-filter 区间、有效状态或证据身份无效")
        if self.unknown_start_m is not None and np.any(self.unknown_end_m < self.unknown_start_m):
            raise ValueError("unknown 区间终点必须大于等于起点")
        if (self.visible_free_until_m is not None
                and np.any(self.visible_free_until_m > np.linalg.norm(self.endpoints_m - self.origins_m, axis=1))):
            raise ValueError("可见 free 前缀不能超过真实返回")


@dataclass(frozen=True)
class OccupancyGrid:
    origin_m: np.ndarray
    shape: tuple[int, int, int]
    voxel_size_mm: float
    required_mask: np.ndarray | None = None

    def __post_init__(self):
        object.__setattr__(self, "origin_m", _readonly(self.origin_m, np.float64, (3,), "origin_m"))
        if (len(self.shape) != 3 or any(type(item) is not int or item <= 0 for item in self.shape)
                or np.prod(self.shape, dtype=object) > np.iinfo(np.int32).max
                or not np.isfinite(self.voxel_size_mm) or self.voxel_size_mm <= 0):
            raise ValueError("占据网格的 shape 或 voxel_size_mm 无效")
        required = np.ones(self.shape, dtype=bool) if self.required_mask is None else self.required_mask
        object.__setattr__(self, "required_mask", _readonly(required, bool, self.shape, "required_mask"))

    @property
    def voxel_size_m(self):
        return self.voxel_size_mm * 1e-3

    @property
    def upper_m(self):
        return self.origin_m + np.array(self.shape) * self.voxel_size_m

    @property
    def required_space_hash(self):
        assert self.required_mask is not None
        return hashlib.sha256(json.dumps({
            "origin_m": self.origin_m.tolist(), "shape": self.shape,
            "voxel_size_mm": self.voxel_size_mm, "required_mask": self.required_mask.tolist(),
        }, sort_keys=True, separators=(",", ":")).encode()).digest()

    def index(self, point):
        index = np.floor((point - self.origin_m) / self.voxel_size_m).astype(np.int64)
        if np.any(index < 0) or np.any(index >= self.shape):
            return None
        return tuple(index)

    def observation_index(self, point, uncertainty_mm):
        index = self.index(point)
        if index is not None:
            return index, uncertainty_mm
        nearest = np.clip(point, self.origin_m, self.upper_m)
        if np.linalg.norm(point - nearest) * 1000 > uncertainty_mm:
            return None, uncertainty_mm
        interior = np.clip(point, self.origin_m, np.nextafter(self.upper_m, self.origin_m))
        index = self.index(interior)
        center = self.origin_m + (np.array(index) + .5) * self.voxel_size_m
        displacement_mm = np.linalg.norm(point - center) * 1000
        extra_mm = max(0., displacement_mm - np.sqrt(3) * self.voxel_size_mm / 2)
        return index, uncertainty_mm + extra_mm

    def overlapping_cells(self, point, uncertainty_mm):
        radius_m = uncertainty_mm * .001
        lower = np.floor(np.nextafter((point - radius_m - self.origin_m) / self.voxel_size_m, -np.inf)).astype(np.int64)
        upper = np.floor(np.nextafter((point + radius_m - self.origin_m) / self.voxel_size_m, np.inf)).astype(np.int64)
        lower, upper = np.maximum(lower, 0), np.minimum(upper, np.array(self.shape) - 1)
        if np.any(lower > upper):
            return
        for offset in np.ndindex(tuple(upper - lower + 1)):
            index = lower + offset
            cell_lower = self.origin_m + index * self.voxel_size_m
            nearest = np.clip(point, cell_lower, cell_lower + self.voxel_size_m)
            if np.linalg.norm(point - nearest) <= np.nextafter(radius_m, np.inf):
                yield tuple(index)

    def ray_cells(self, origin, endpoint):
        direction = endpoint - origin
        start, finish = 0., 1.
        for axis in range(3):
            if direction[axis] == 0:
                if not self.origin_m[axis] < origin[axis] < self.upper_m[axis]:
                    return
                continue
            ends = (np.array([self.origin_m[axis], self.upper_m[axis]]) - origin[axis]) / direction[axis]
            start, finish = max(start, float(ends.min())), min(finish, float(ends.max()))
        if start >= finish:
            return
        point = np.nextafter(origin + start * direction, origin + finish * direction)
        index = np.clip(np.floor((point - self.origin_m) / self.voxel_size_m).astype(int), 0, np.array(self.shape) - 1)
        step = np.sign(direction).astype(int)
        delta, crossing = np.full(3, np.inf), np.full(3, np.inf)
        moving = direction != 0
        delta[moving] = self.voxel_size_m / np.abs(direction[moving])
        boundary = self.origin_m + (index + (step > 0)) * self.voxel_size_m
        crossing[moving] = (boundary[moving] - origin[moving]) / direction[moving]
        while np.all(index >= 0) and np.all(index < self.shape) and start < finish:
            end = min(finish, float(crossing.min()))
            if end > start:
                yield tuple(index), start, end
            axes = crossing <= end
            index[axes] += step[axes]
            crossing[axes] += delta[axes]
            start = end


@dataclass(frozen=True)
class OccupancySnapshot:
    cells: np.ndarray
    last_observed_ns: np.ndarray
    last_hit_ns: np.ndarray
    free_evidence_count: np.ndarray
    free_evidence_stamp_ns: Mapping[tuple, tuple[int, ...]]
    ambiguous_mask: np.ndarray
    ambiguity_stamp_ns: np.ndarray
    fixed_model_hash: bytes
    support_demand: int
    source_support_points_m: np.ndarray
    source_support_radii_mm: np.ndarray
    support_member_indices: tuple[tuple[int, ...], ...]
    support_source_stamp_ns: np.ndarray
    support_valid_until_ns: np.ndarray
    support_motion_bound_mm: np.ndarray
    scene: CollisionScene


class LidarOccupancyScene:
    def __init__(self, config: CollisionSafetyConfig, identities: CollisionIdentities, grid: OccupancyGrid):
        if not isinstance(config, CollisionSafetyConfig) or not isinstance(identities, CollisionIdentities):
            raise TypeError("占据场景需要 CollisionSafetyConfig 与 CollisionIdentities")
        if grid.voxel_size_mm != config.policy.artifact.parameters["voxel_size_mm"]["value"]:
            raise ValueError("占据网格 voxel_size_mm 与参数 artifact 不一致")
        if np.asarray(identities.required_space_hash).tobytes() != grid.required_space_hash:
            raise ValueError("占据网格必要空间身份不一致")
        self.config, self.identities, self.grid = config, identities, grid
        self.parameters = config.policy.artifact.parameters
        self._cells = np.full(grid.shape, VoxelState.UNKNOWN, dtype=np.uint8)
        self._observed_ns = np.zeros(grid.shape, dtype=np.int64)
        self._hit_ns = np.zeros(grid.shape, dtype=np.int64)
        self._hit_evidence: dict[tuple, dict[int, float]] = {}
        self._free_count = np.zeros(grid.shape, dtype=np.int64)
        self._last_free_ns = np.zeros(grid.shape, dtype=np.int64)
        self._free_evidence: dict[tuple, list[int]] = {}
        self._ambiguity_ns = np.zeros(grid.shape, dtype=np.int64)
        self._revision = int(np.asarray(identities.scene_revision).item())
        self._last_scan_id = -1
        self._last_source_ns = 0
        self._last_now_ns = 0
        self._fixed_environment: FixedEnvironmentModel | None = None
        self._filter_identity = ""
        self._identity_invalidated = False
        self._input_valid = True
        self.invalid_reason = "ok"
        self._tracking_hash = b""

    def observations(self) -> tuple[OccupiedObservation, ...]:
        evidence = hashlib.sha256(np.asarray(self.identities.scene_epoch).tobytes()
                                  + self.grid.required_space_hash + self.config.collision_policy_hash).digest()
        records = []
        for index in map(tuple, np.argwhere(self._cells == VoxelState.OCCUPIED)):
            support_id = int(np.ravel_multi_index(index, self.grid.shape))
            point = self.grid.origin_m + (np.array(index) + .5) * self.grid.voxel_size_m
            for stamp, error in sorted(self._hit_evidence[index].items()):
                records.append(OccupiedObservation(
                    support_id, point, np.sqrt(3) * self.grid.voxel_size_mm / 2, error,
                    stamp, self._value("point_timestamp_error_ns"), evidence))
        return tuple(records)

    def set_fixed_environment(self, model: FixedEnvironmentModel):
        if not isinstance(model, FixedEnvironmentModel):
            raise TypeError("固定环境需要 FixedEnvironmentModel")
        model.verify_identity()
        if model.policy_hash != self.config.collision_policy_hash:
            raise ValueError("固定模型的 collision policy 身份不一致")
        if model.model_revision < int(np.asarray(self.identities.fixed_environment_revision).item()):
            raise ValueError("固定模型的 model_revision 不能递减")
        previous = self._fixed_environment
        if previous is not None and model.model_hash == previous.model_hash and model.model_revision == previous.model_revision:
            return
        if previous is not None and model.model_revision <= previous.model_revision:
            raise ValueError("固定模型发生变化时必须增加 model_revision")
        self._fixed_environment = replace(model)
        self.identities = self.identities._replace(fixed_environment_revision=np.int64(model.model_revision))
        self._revision += 1

    def _fixed_known(self):
        result = np.zeros(self.grid.shape, dtype=bool)
        if self._fixed_environment is None:
            return result
        indices = np.indices(self.grid.shape).reshape(3, -1).T
        centers = self.grid.origin_m + (indices + .5) * self.grid.voxel_size_m
        half_diagonal_mm = np.sqrt(3) * self.grid.voxel_size_mm / 2
        flat = result.reshape(-1)
        for center, radius in zip(self._fixed_environment.support_points_m,
                                  self._fixed_environment.support_radii_mm):
            flat |= np.nextafter(np.linalg.norm(centers - center, axis=1) * 1e3 + half_diagonal_mm, np.inf) <= radius
        return result

    def _merge_supports(self, points, radii, deadline_ns):
        capacity = self.config.max_support_points
        maximum_members = self._value("support_merge_max_points")
        if (len(points) > capacity * maximum_members
                or np.any(radii > self._value("support_radius_limit_mm"))):
            return None
        groups: list[tuple[tuple[int, ...], np.ndarray, float]] = [
            ((index,), point.copy(), float(radius)) for index, (point, radius) in enumerate(zip(points, radii))]
        limit = self._value("support_merge_radius_limit_mm")
        while len(groups) > capacity:
            selected = None
            for first in range(len(groups)):
                if time.monotonic_ns() > deadline_ns:
                    return None
                for second in range(first + 1, len(groups)):
                    members = groups[first][0] + groups[second][0]
                    if len(members) > maximum_members:
                        continue
                    indices = list(members)
                    center = points[indices].mean(axis=0)
                    radius = float(np.nextafter(np.max(
                        np.linalg.norm(points[indices] - center, axis=1) * 1e3 + radii[indices],
                    ), np.inf))
                    if radius <= limit and (selected is None or radius < selected[0]):
                        selected = radius, first, second, members, center
            if selected is None:
                return None
            radius, first, second, members, center = selected
            groups[first] = members, center, radius
            del groups[second]
        if time.monotonic_ns() > deadline_ns:
            return None
        return groups

    def _value(self, name):
        return self.parameters[name]["value"]

    def _measurement_radius_mm(self, distance_m):
        angle = self._value("lidar_angular_error_rad") + self._value("extrinsic_rotation_error_rad")
        return (self._value("lidar_range_error_mm") + self._value("lidar_incidence_error_mm")
                + self._value("extrinsic_translation_error_mm") + distance_m * 1e3 * angle)

    def _expire(self, now_ns):
        age = now_ns - self._observed_ns
        expired = (((self._cells == VoxelState.OCCUPIED) & (age > self._value("occupied_max_age_ns")))
                   | ((self._cells == VoxelState.FREE) & (age > self._value("free_max_age_ns"))))
        if np.any(expired):
            self._cells[expired] = VoxelState.UNKNOWN
            self._free_count[expired] = 0
            self._last_free_ns[expired] = 0
            self._revision += 1
        for index in list(self._free_evidence):
            retained_free = [stamp for stamp in self._free_evidence[index]
                             if now_ns - stamp <= self._value("free_max_age_ns") and not expired[index]]
            if len(retained_free) != len(self._free_evidence[index]):
                self._revision += 1
            if retained_free:
                self._free_evidence[index] = retained_free
            else:
                del self._free_evidence[index]
            self._free_count[index] = len(retained_free)
            self._last_free_ns[index] = retained_free[-1] if retained_free else 0
        for index in list(self._hit_evidence):
            retained = {stamp: error for stamp, error in self._hit_evidence[index].items()
                        if now_ns - stamp <= self._value("occupied_max_age_ns")}
            if retained:
                if len(retained) != len(self._hit_evidence[index]):
                    self._revision += 1
                self._hit_evidence[index] = retained
            else:
                del self._hit_evidence[index]

    def _revoke_free(self):
        self._cells[self._cells == VoxelState.FREE] = VoxelState.UNKNOWN
        self._free_evidence.clear()
        self._free_count.fill(0)
        self._last_free_ns.fill(0)

    def invalidate(self, *, now_ns: int, reason: str):
        if type(now_ns) is not int or now_ns <= 0 or now_ns < self._last_now_ns or not reason.strip():
            raise ValueError("占据输入失效时间或原因无效")
        self._revoke_free()
        self._input_valid, self.invalid_reason = False, reason
        self._revision += 1
        self._last_now_ns = now_ns

    def _unknown_cells(self, origin, endpoint, start_m, end_m, radius_mm):
        if end_m <= start_m:
            return
        direction = endpoint - origin
        direction /= np.linalg.norm(direction)
        start, finish = origin + start_m * direction, origin + end_m * direction
        radius = radius_mm * .001
        lower = np.floor(np.nextafter((np.minimum(start, finish) - radius - self.grid.origin_m) / self.grid.voxel_size_m, -np.inf)).astype(int)
        upper = np.floor(np.nextafter((np.maximum(start, finish) + radius - self.grid.origin_m) / self.grid.voxel_size_m, np.inf)).astype(int)
        lower, upper = np.maximum(lower, 0), np.minimum(upper, np.array(self.grid.shape) - 1)
        if np.any(lower > upper):
            return
        half_diagonal = np.sqrt(3) * self.grid.voxel_size_m / 2
        for offset in np.ndindex(tuple(upper - lower + 1)):
            index = lower + offset
            center = self.grid.origin_m + (index + .5) * self.grid.voxel_size_m
            projected = np.clip(np.dot(center - start, direction), 0., end_m - start_m)
            if np.linalg.norm(center - start - projected * direction) <= np.nextafter(radius + half_diagonal, np.inf):
                yield tuple(index)

    def update(self, batch: LidarRayBatch, *, now_ns: int):
        if not isinstance(batch, LidarRayBatch):
            raise TypeError("update 需要 LidarRayBatch")
        if type(now_ns) is not int or now_ns <= 0 or now_ns < self._last_now_ns or batch.scan_id <= self._last_scan_id:
            raise ValueError("LiDAR scan 身份或采集时间无效")
        if not batch.input_valid:
            self.invalidate(now_ns=now_ns, reason=batch.invalid_reason)
            self._last_scan_id = batch.scan_id
            return
        if self._identity_invalidated or (self._filter_identity and batch.filter_identity != self._filter_identity):
            self._identity_invalidated = True
            self.invalidate(now_ns=now_ns, reason="filter_identity_changed_new_scene_required")
            self._last_scan_id = batch.scan_id
            return
        if (type(now_ns) is not int or now_ns <= 0 or now_ns < self._last_now_ns
                or batch.scan_id <= self._last_scan_id
                or np.any(batch.point_stamp_ns > now_ns)
                or np.any(batch.point_stamp_ns < self._last_source_ns)
                or np.any(now_ns - batch.point_stamp_ns > self._value("transport_delay_ns"))):
            raise ValueError("LiDAR scan 身份或采集时间无效")
        self._expire(now_ns)
        if batch.filter_identity:
            if not self._filter_identity:
                self._revoke_free()
            self._filter_identity = batch.filter_identity
        self._input_valid, self.invalid_reason = True, "ok"
        hits: dict[tuple, dict[int, float]] = {}
        free: dict[tuple, int] = {}
        ambiguous: dict[tuple, int] = {}
        for slot, (origin, endpoint, stamp, kind) in enumerate(zip(
                batch.origins_m, batch.endpoints_m, batch.point_stamp_ns, batch.return_kind)):
            explicit_intervals = batch.visible_free_until_m is not None
            if batch.unknown_start_m is not None and batch.unknown_end_m is not None:
                radius = 0. if batch.unknown_radius_mm is None else batch.unknown_radius_mm[slot]
                for index in self._unknown_cells(origin, endpoint, batch.unknown_start_m[slot], batch.unknown_end_m[slot], radius):
                    ambiguous[index] = max(ambiguous.get(index, 0), int(stamp))
            if kind not in (RayReturn.HIT, RayReturn.AMBIGUOUS) and not explicit_intervals:
                continue
            distance = float(np.linalg.norm(endpoint - origin))
            error_mm = self._measurement_radius_mm(distance)
            if kind == RayReturn.AMBIGUOUS and not explicit_intervals:
                for index in self.grid.overlapping_cells(endpoint, error_mm):
                    ambiguous[index] = max(ambiguous.get(index, 0), int(stamp))
                continue
            hit, support_error_mm = self.grid.observation_index(endpoint, error_mm) if kind == RayReturn.HIT else (None, error_mm)
            if hit is not None:
                observations = hits.setdefault(hit, {})
                observations[int(stamp)] = max(observations.get(int(stamp), 0.), support_error_mm)
            limit = distance - error_mm * 1e-3
            if batch.visible_free_until_m is not None:
                limit = min(limit, batch.visible_free_until_m[slot])
            for index, _, end in self.grid.ray_cells(origin, endpoint):
                if index != hit and end * distance <= limit:
                    free[index] = max(free.get(index, 0), int(stamp))
        continued = np.zeros(self.grid.shape, dtype=bool)
        for index, stamp in free.items():
            if (index in hits or index in ambiguous
                    or now_ns - stamp > self._value("free_max_age_ns")):
                continue
            continued[index] = True
            previous = self._last_free_ns[index]
            if previous == 0 or stamp - previous > self._value("free_evidence_max_gap_ns"):
                self._free_evidence[index] = [stamp]
            elif stamp - previous >= self._value("free_evidence_min_interval_ns"):
                self._free_evidence[index].append(stamp)
            self._free_count[index] = len(self._free_evidence[index])
            self._last_free_ns[index] = self._free_evidence[index][-1]
            if (self._cells[index] != VoxelState.OCCUPIED
                    or self._free_count[index] >= self._value("free_evidence_required_count")):
                self._cells[index], self._observed_ns[index] = VoxelState.FREE, stamp
                self._hit_evidence.pop(index, None)
            if stamp >= self._ambiguity_ns[index]:
                self._ambiguity_ns[index] = 0
        self._free_count[~continued] = 0
        self._last_free_ns[~continued] = 0
        for index in list(self._free_evidence):
            if not continued[index]:
                del self._free_evidence[index]
        for index, observations in hits.items():
            stamp = max(observations)
            self._cells[index], self._observed_ns[index] = VoxelState.OCCUPIED, stamp
            self._hit_ns[index] = stamp
            if index not in ambiguous and stamp >= self._ambiguity_ns[index]:
                self._ambiguity_ns[index] = 0
            retained = self._hit_evidence.setdefault(index, {})
            for observed, error_mm in observations.items():
                retained[observed] = max(retained.get(observed, 0.), error_mm)
        for index, stamp in ambiguous.items():
            self._ambiguity_ns[index] = max(stamp, self._ambiguity_ns[index])
            if self._cells[index] != VoxelState.OCCUPIED:
                self._cells[index] = VoxelState.UNKNOWN
        self._last_scan_id = batch.scan_id
        if len(batch.point_stamp_ns):
            self._last_source_ns = int(batch.point_stamp_ns.max())
        self._revision += 1
        self._last_now_ns = now_ns

    def snapshot(self, *, now_ns: int, tracking: TrackingFrame | None = None) -> OccupancySnapshot:
        preparation_started = time.monotonic_ns()
        if type(now_ns) is not int or now_ns < self._last_now_ns or now_ns <= 0:
            raise ValueError("场景快照时间无效")
        if tracking is not None and not isinstance(tracking, TrackingFrame):
            raise TypeError("tracking 必须是 TrackingFrame")
        source_records = tuple(record for record in self.observations()
                               if now_ns - record.point_stamp_ns <= self._value("occupied_max_age_ns"))
        associated = {} if tracking is None else tracking.select(
            self.identities, self.config.policy, source_records, now_ns)
        tracking_hash = b"" if tracking is None else tracking.content_hash
        self._expire(now_ns)
        if (now_ns > self._last_now_ns and np.any(self._cells == VoxelState.OCCUPIED)
                and (self._value("untracked_speed_mm_s") > 0
                     or self._value("untracked_acceleration_mm_s2") > 0)):
            self._revision += 1
        self._last_now_ns = now_ns
        occupied = np.argwhere(self._cells == VoxelState.OCCUPIED)
        dynamic_count = len(occupied)
        fixed = self._fixed_environment
        fixed_count = 0 if fixed is None else len(fixed.support_points_m)
        raw_count, capacity = dynamic_count + fixed_count, self.config.max_support_points
        raw_points, raw_radii = np.zeros((raw_count, 3)), np.zeros(raw_count)
        raw_stamps = np.zeros(raw_count, dtype=np.int64)
        raw_deadlines = np.full(raw_count, np.iinfo(np.int64).max, dtype=np.int64)
        raw_motion = np.zeros(raw_count)
        raw_ids = np.empty(raw_count, dtype=np.int32)
        raw_rates, raw_acceleration = np.zeros(raw_count), np.zeros(raw_count)
        raw_hashes = np.zeros((raw_count, 32), dtype=np.uint8)
        if tracking_hash != self._tracking_hash:
            self._tracking_hash = tracking_hash
            self._revision += 1
        raw_velocity = np.zeros((raw_count, 3))
        raw_track_ids = np.full(raw_count, -1, dtype=np.int32)
        if dynamic_count:
            raw_points[:dynamic_count] = self.grid.origin_m + (occupied + .5) * self.grid.voxel_size_m
            for slot, index in enumerate(occupied):
                observations = self._hit_evidence[tuple(index)]
                stamps = np.array(list(observations), dtype=np.int64)
                errors = np.array(list(observations.values()))
                age_s = (now_ns - stamps) * 1e-9 + self._value("point_timestamp_error_ns") * 1e-9
                motion = (self._value("untracked_speed_mm_s") * age_s
                          + .5 * self._value("untracked_acceleration_mm_s2") * age_s ** 2)
                raw_motion[slot] = motion.max()
                raw_rates[slot] = (self._value("untracked_speed_mm_s")
                                   + self._value("untracked_acceleration_mm_s2") * age_s.max())
                raw_acceleration[slot] = self._value("untracked_acceleration_mm_s2")
                raw_radii[slot] = np.nextafter(np.sqrt(3) * self.grid.voxel_size_mm / 2
                                               + np.max(errors + motion), np.inf)
                raw_stamps[slot] = stamps.min()
                raw_deadlines[slot] = stamps.min() + self._value("occupied_max_age_ns")
                support_id = int(np.ravel_multi_index(tuple(index), self.grid.shape))
                track = associated.get(support_id)
                if track is not None:
                    tau = track.timestamp_error_ns * 1e-9
                    history_age = ((track.updated_stamp_ns - stamps) * 1e-9
                                   + self._value("point_timestamp_error_ns") * 1e-9 + tau)
                    history = (self._value("untracked_speed_mm_s") * history_age
                               + .5 * self._value("untracked_acceleration_mm_s2") * history_age ** 2)
                    elapsed = (now_ns - track.updated_stamp_ns) * 1e-9
                    acceleration = track.acceleration_bound_m_s2 * 1000
                    uncertainty = (track.position_error_mm + np.linalg.norm(track.velocity_m_s) * 1000 * tau
                                   + track.velocity_error_mm_s * (elapsed + tau)
                                   + .5 * acceleration * (elapsed + tau) ** 2)
                    raw_points[slot] += track.velocity_m_s * elapsed
                    raw_radii[slot] = np.nextafter(np.sqrt(3) * self.grid.voxel_size_mm / 2
                                                  + np.max(errors + history) + uncertainty, np.inf)
                    raw_motion[slot] = history.max() + uncertainty
                    raw_rates[slot] = track.velocity_error_mm_s + acceleration * (elapsed + tau)
                    raw_acceleration[slot] = acceleration
                    raw_velocity[slot] = track.velocity_m_s
                    raw_track_ids[slot] = track.track_id
                    raw_deadlines[slot] = min(raw_deadlines[slot], track.valid_until_ns)
            raw_ids[:dynamic_count] = np.ravel_multi_index(tuple(occupied.T), self.grid.shape)
            for slot, support_id in enumerate(raw_ids[:dynamic_count]):
                raw_hashes[slot] = np.frombuffer(observation_hash(tuple(
                    item for item in source_records if item.support_id == support_id)), dtype=np.uint8)
                if support_id in associated:
                    raw_hashes[slot] = np.frombuffer(hashlib.sha256(raw_hashes[slot].tobytes()
                                                                  + tracking_hash).digest(), dtype=np.uint8)
        if fixed is not None and fixed_count:
            raw_points[dynamic_count:] = fixed.support_points_m
            raw_radii[dynamic_count:] = fixed.support_radii_mm
            raw_ids[dynamic_count:] = int(np.prod(self.grid.shape)) + np.arange(fixed_count)
            raw_hashes[dynamic_count:] = fixed.support_history_hashes()
        demand = dynamic_count + (0 if fixed is None else fixed.support_demand)
        deadline = preparation_started + self._value("scene_preparation_deadline_ns")
        groups = self._merge_supports(raw_points, raw_radii, deadline) if demand == raw_count else None
        status = CollisionStatus.OK if groups is not None else CollisionStatus.CAPACITY_OVERFLOW
        if not self._input_valid:
            status = CollisionStatus.INVALID_SCENE
        groups = [] if groups is None else groups
        count = len(groups)
        points, radii = np.zeros((capacity, 3)), np.zeros(capacity)
        support_ids = np.zeros(capacity, dtype=np.int32)
        source_stamps, valid_until = np.zeros(capacity, dtype=np.int64), np.zeros(capacity, dtype=np.int64)
        motion_bounds = np.zeros(capacity)
        for slot, (members, center, radius) in enumerate(groups):
            indices = list(members)
            points[slot], radii[slot] = center, radius
            support_ids[slot] = raw_ids[indices].min()
            stamps = raw_stamps[indices]
            source_stamps[slot] = int(stamps[stamps > 0].min()) if np.any(stamps > 0) else 0
            valid_until[slot] = raw_deadlines[indices].min()
            motion_bounds[slot] = raw_motion[indices].max()
        mask = np.arange(capacity) < count
        fixed_known = self._fixed_known()
        cells = self._cells.copy()
        cells[fixed_known] = VoxelState.OCCUPIED
        known = (cells != VoxelState.UNKNOWN) & (self._ambiguity_ns == 0)
        covered = self._input_valid and bool(np.all(known[self.grid.required_mask]))
        needed = (self._cells == VoxelState.OCCUPIED) | (self.grid.required_mask & ~fixed_known
                                                        & (self._cells != VoxelState.UNKNOWN))
        stamps = np.r_[self._observed_ns[needed], raw_stamps[raw_stamps > 0]]
        source = int(stamps.min()) if len(stamps) else (self._last_source_ns or now_ns)
        budget = sum(self._value(name) for name in ("transport_delay_ns", "scene_preparation_delay_ns", "control_delay_ns"))
        tracks = self._value("max_tracks")
        identities = self.identities._replace(scene_revision=np.int64(self._revision))
        needed_coverage = self.grid.required_mask & ~fixed_known & (self._cells != VoxelState.UNKNOWN)
        required_stamps = self._observed_ns[needed_coverage]
        required_states = self._cells[needed_coverage]
        deadlines = required_stamps + np.where(
            required_states == VoxelState.OCCUPIED,
            self._value("occupied_max_age_ns"), self._value("free_max_age_ns"),
        )
        coverage_until = min(source + budget, int(deadlines.min())) if len(deadlines) else source + budget
        if raw_count:
            coverage_until = min(coverage_until, int(raw_deadlines.min()))
        if time.monotonic_ns() > deadline:
            status = CollisionStatus.CAPACITY_OVERFLOW
        scene = CollisionScene(
            support_points_m=points, support_radii_mm=radii,
            required_clearance_mm=np.full(capacity, self._value("environment_clearance_mm")),
            support_velocity_m_s=np.zeros((capacity, 3)), support_ids=support_ids,
            support_mask=mask, source_stamp_ns=np.int64(source), prepared_stamp_ns=np.int64(now_ns),
            status=np.int32(status), support_track_status=np.zeros(capacity, dtype=np.int32),
            support_valid_mask=mask.copy(), support_track_ids=np.full(capacity, -1, dtype=np.int32),
            tracks=SceneTracks(
                np.full(tracks, -1, dtype=np.int32), np.zeros(tracks, dtype=bool), np.zeros(tracks, dtype=bool),
                np.zeros((tracks, 3)), np.zeros(tracks), np.zeros(tracks),
                np.zeros(tracks, dtype=np.int64), np.zeros(tracks, dtype=np.int64),
            ), identities=identities, coverage_space_hash=np.array(identities.required_space_hash),
            coverage_valid_until_ns=np.int64(coverage_until),
            metadata=SceneMetadata(1, "base_link", "m", "mm", "m/s", "ns", "monotonic", count, 0, covered, b""),
            fixed_environment_model=fixed,
        )
        sources = SourceMotion(raw_ids, raw_points, raw_radii, raw_velocity, raw_rates,
                               raw_acceleration, raw_stamps, raw_deadlines, raw_track_ids, raw_hashes)
        model, velocities = build_time_model(scene, sources, tuple(group[0] for group in groups),
                                             self._value("support_merge_max_points"),
                                             self._value("support_radius_limit_mm"))
        group_tracks = np.full(capacity, -1, dtype=np.int32)
        group_status = np.zeros(capacity, dtype=np.int32)
        used_tracks = {}
        for slot, (members, _, _) in enumerate(groups):
            member_ids = raw_track_ids[list(members)]
            unique = np.unique(member_ids)
            if len(unique) == 1 and unique[0] >= 0:
                group_tracks[slot], group_status[slot] = unique[0], 1
            elif np.any(member_ids >= 0):
                group_status[slot] = 2
            for member in members:
                if raw_track_ids[member] >= 0:
                    used_tracks[raw_track_ids[member]] = associated[int(raw_ids[member])]
        track_values = {name: np.array(value, copy=True) for name, value in scene.tracks._asdict().items()}
        for slot, track in enumerate(used_tracks.values()):
            for name, value in (("ids", track.track_id), ("active_mask", True), ("valid_mask", True),
                                ("velocity_m_s", track.velocity_m_s),
                                ("acceleration_bound_m_s2", track.acceleration_bound_m_s2),
                                ("error_bound_mm", track.position_error_mm),
                                ("updated_stamp_ns", track.updated_stamp_ns), ("valid_until_ns", track.valid_until_ns)):
                track_values[name][slot] = value
        scene = scene._replace(support_time_model=model, support_velocity_m_s=velocities,
                               support_track_status=group_status, support_track_ids=group_tracks,
                               tracks=SceneTracks(**track_values),
                               metadata=replace(scene.metadata, track_count=len(used_tracks)))
        scene = scene._replace(metadata=replace(scene.metadata, checksum=scene.compute_checksum()))
        return OccupancySnapshot(
            cells, self._observed_ns.copy(), self._hit_ns.copy(), self._free_count.copy(),
            MappingProxyType({index: tuple(stamps) for index, stamps in self._free_evidence.items()}),
            self._ambiguity_ns != 0, self._ambiguity_ns.copy(),
            b"" if fixed is None else fixed.model_hash, demand, raw_points, raw_radii,
            tuple(group[0] for group in groups), source_stamps, valid_until, motion_bounds, scene,
        )
