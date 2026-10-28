from __future__ import annotations

import time
from collections import deque
from copy import deepcopy
from dataclasses import dataclass

import numpy as np

from work.collision_safety import CollisionIdentities, CollisionStatus
from work.dynamic_support import TrackingFrame
from work.kinematics_data import JOINT_POSITION_LOWER, JOINT_POSITION_UPPER
from work.lidar_occupancy import LidarOccupancyScene, OccupancyGrid, OccupancySnapshot
from work.lidar_self_filter import ClockMapping, MeshRaySelfFilter, SelfFilterResult
from work.shared_collision_scene import ScenePublicationError, SceneRevision, SharedSceneWriter

from robot_safecontrol_moveit.lidar_self_filter_adapter import (
    history_from_joint_states, joint_state_sample, scan_from_pointcloud,
)


@dataclass(frozen=True)
class ScenePipelineResult:
    revision: SceneRevision
    snapshot: OccupancySnapshot
    filter_result: SelfFilterResult | None


class CollisionScenePipeline:
    def __init__(self, writer: SharedSceneWriter, filter_module: MeshRaySelfFilter,
                 grid: OccupancyGrid, mapping: ClockMapping, *, feedback_source: str,
                 zero_calibration_id: str, history_capacity: int, max_range_m: float,
                 joint_mapping: ClockMapping | None = None):
        if (not isinstance(writer, SharedSceneWriter) or not isinstance(filter_module, MeshRaySelfFilter)
                or not isinstance(grid, OccupancyGrid) or not isinstance(mapping, ClockMapping)):
            raise TypeError("场景管线需要唯一 writer、真实 mesh self-filter、占据网格和 ClockMapping")
        if (feedback_source not in ("simulation", "encoder") or not zero_calibration_id.strip()
                or type(history_capacity) is not int or history_capacity < 2
                or not np.isfinite(max_range_m) or max_range_m <= 0):
            raise ValueError("场景反馈来源、历史容量、零位身份或 LiDAR 范围无效")
        self.writer, self.filter_module, self.mapping = writer, filter_module, mapping
        self.joint_mapping = mapping if joint_mapping is None else joint_mapping
        if (not isinstance(self.joint_mapping, ClockMapping)
                or self.joint_mapping.run_id != mapping.run_id
                or self.joint_mapping.target_clock_id != mapping.target_clock_id):
            raise ValueError("LiDAR 与反馈映射必须使用共同 run 和目标时钟")
        self.feedback_source, self.zero_calibration_id = feedback_source, zero_calibration_id
        self.max_range_m = float(max_range_m)
        config = writer.safety.config
        identities = CollisionIdentities(
            np.frombuffer(writer.epoch, "u1"), np.int64(0),
            np.frombuffer(config.geometry_hash, "u1"), np.frombuffer(config.kernel_version, "u1"),
            np.frombuffer(config.collision_policy_hash, "u1"), np.int64(0),
            np.frombuffer(grid.required_space_hash, "u1"))
        self.occupancy = LidarOccupancyScene(config, identities, grid)
        self._feedback = deque(maxlen=history_capacity)
        self._last_feedback_ns = None
        self._scan_id = 0
        self._tracking = None
        self._last_now_ns = 0

    def invalidate(self, reason, *, now_ns=None):
        now = time.monotonic_ns() if now_ns is None else now_ns
        self.occupancy.invalidate(now_ns=now, reason=reason)
        self.writer.invalidate(CollisionStatus.INVALID_SCENE)
        self._last_now_ns = now

    def add_joint_state(self, message):
        try:
            stamp, positions = joint_state_sample(message)
            mapped, errors = self.joint_mapping.map(np.array([stamp], dtype=np.int64))
            if (self._last_feedback_ns is not None and stamp <= self._last_feedback_ns
                    or int(mapped[0]) + int(errors[0]) > time.monotonic_ns()
                    or np.any(positions < np.asarray(JOINT_POSITION_LOWER))
                    or np.any(positions > np.asarray(JOINT_POSITION_UPPER))):
                raise ValueError("反馈历史时间必须递增，关节坐标必须满足模型范围")
            self._feedback.append(deepcopy(message))
            self._last_feedback_ns = stamp
        except BaseException:
            self.invalidate("joint_feedback_invalid")
            raise

    @property
    def history(self):
        return history_from_joint_states(
            list(self._feedback), mapping=self.joint_mapping, feedback_source=self.feedback_source,
            model_hash=self.filter_module.geometry.geometry_hash, zero_calibration_id=self.zero_calibration_id)

    def set_fixed_environment(self, model):
        try:
            self.occupancy.set_fixed_environment(model)
            self.writer.invalidate(CollisionStatus.REVISION_PENDING)
        except BaseException:
            self.invalidate("fixed_environment_invalid")
            raise

    def update_tracking(self, frame):
        try:
            if not isinstance(frame, TrackingFrame):
                raise TypeError("tracking 输入必须是 TrackingFrame")
            frame.select(self.occupancy.identities, self.writer.safety.config.policy,
                         self.occupancy.observations(), time.monotonic_ns())
            self._tracking = frame
        except BaseException:
            self.invalidate("tracking_invalid")
            raise

    def observations(self):
        return self.occupancy.observations()

    def ingest_pointcloud(self, message, *, now_ns=None, received_monotonic_ns=None):
        received = time.monotonic_ns() if received_monotonic_ns is None else received_monotonic_ns
        try:
            self._scan_id += 1
            scan = scan_from_pointcloud(message, mapping=self.mapping, scan_id=self._scan_id,
                                       received_monotonic_ns=received, max_range_m=self.max_range_m)
            filtered = self.filter_module.classify(scan, self.history)
            now = time.monotonic_ns() if now_ns is None else now_ns
            self.occupancy.update(filtered.to_ray_batch(), now_ns=now)
            self._last_now_ns = now
            result = self.publish(now_ns=now)
            return ScenePipelineResult(result.revision, result.snapshot, filtered)
        except ScenePublicationError:
            raise
        except BaseException:
            self.invalidate("pointcloud_processing_invalid")
            raise

    def publish(self, *, now_ns=None):
        try:
            now = time.monotonic_ns() if now_ns is None else now_ns
            snapshot = self.occupancy.snapshot(now_ns=now, tracking=self._tracking)
            self._last_now_ns = now
            revision = self.writer.publish(snapshot.scene)
            return ScenePipelineResult(revision, snapshot, None)
        except ScenePublicationError:
            raise
        except BaseException:
            self.writer.invalidate(CollisionStatus.INVALID_SCENE)
            raise
