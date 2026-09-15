"""Perception bridge: dual-sensor fusion -> ESDF + dynamic tracks + CollisionObjects.

Subscribes to LiDAR (``/livox/lidar``) and Camera (``/camera/depth_registered/points``)
point clouds.  Sensor callbacks (ReentrantCallbackGroup) only decode -> apply the
startup-validated calibration transform -> ROI -> source voxel -> push to deque.
A 20 Hz fusion timer (MutuallyExclusiveCallbackGroup)
performs timestamp pairing, self-filtering, fusion voxel downsampling, three-layer
classification, ESDF construction, dynamic clustering, and publishes:

* ``/perception/cloud_world``        — fused voxel cloud (sensor_msgs/PointCloud2)
* ``/perception/esdf``               — fixed-shape distance field (Float32MultiArray)
* ``/perception/esdf_meta``          — [origin_x, origin_y, origin_z, voxel_size, valid]
* ``/perception/tracks``             — 8 fixed dynamic-obstacle slots (Float32MultiArray)
* ``/collision_object``              — SPHERE primitives for MoveIt planning scene
* ``/perception/instant_occupancy``  — instant safety channel (PointCloud2)
* ``/perception/status``             — 10-element health status (Float32MultiArray)
* ``/perception/calibration_status`` — calibration record identity + admission
                                      (diagnostic_msgs/DiagnosticArray)

Extrinsics come from the calibration record (``config/sensor_extrinsics.yaml`` via the
ament share path) and **only** from there — ADR 0004 single authority, ADR 0009 schema.
Startup is fail-closed: an unreadable record, a non-SE(3) matrix, a frame binding that
does not match the configured frames, an enabled source that has not passed the
calibration gate without a per-source debug exemption, or a leftover legacy extrinsics
parameter all refuse to start (exit code 3) rather than fusing on unknown geometry.
There is no identity fallback and no TF input path.

When ``source_topic_lidar`` is empty, LiDAR is not subscribed and the node behaves
identically to the original single-camera version.

Usage::

    ros2 run robot_safecontrol_moveit perception_bridge
"""

from __future__ import annotations

import os
import sys
import time
from collections import deque
from pathlib import Path
from threading import Lock

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point, Quaternion
from robot_safecontrol_moveit.calibration_record import (
    DIAGNOSTIC_LEVELS,
    LEVEL_ERROR,
    CalibrationRecordError,
    CalibrationStartupRefused,
    StartupDecision,
    calibration_status_entries,
    decide_startup,
    load_calibration_record,
)
from robot_safecontrol_moveit.ros_conventions import (
    JOINT_STATE_TOPIC,
    PERCEPTION_TRACKS_TOPIC,
)

# --- portable_oscbf on path (pure-python calculation core) --------------------
_REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_REPO_ROOT / "portable_oscbf"))
sys.path.insert(0, str(_REPO_ROOT / "portable_oscbf" / "work"))

from work.fusion_engine import FusionEngine  # noqa: E402
from work.perception_config import (  # noqa: E402
    PointCloudCollisionConfig,
    load_point_cloud_collision,
    spec_of,
)
from work.safety_snapshot import (  # noqa: E402
    MAX_DYNAMIC_TRACKS,
    preprocess_points,
)

# Each /perception/tracks slot carries 10 floats.
_TRACK_SLOT_FLOATS = 10  # px,py,pz, r, vx,vy,vz, enabled, d_safe, alpha


PERCEPTION_CALIBRATION_STATUS_TOPIC = "/perception/calibration_status"

# 已废弃的外参参数：声明它们**只为探测**，不再有任何运行时语义。
# 值不等于内置默认 = 启动命令还在传旧配置 → 拒绝启动（两套外参并存正是 ADR 0004
# 要消灭的 split-brain）。只删声明会让旧参数静默失效：使用者以为它生效了，实际用的是
# 记录里的另一套几何（本机实测，`.scratch/off01-extrinsics-source/README.md`）。
_LEGACY_EXTRINSICS_DEFAULTS: dict[str, object] = {
    "use_tf": False,
    "camera_to_world_static": [
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    "lidar_to_world_static": [
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
}
def _points_xyz(msg: PointCloud2) -> np.ndarray:
    """Extract x/y/z as float32 regardless of mixed field datatypes.

    sensor_msgs_py.read_points_numpy requires all fields to share one dtype,
    which Livox PointCloud2 (float32 xyz + uint8 tag/line) violates.
    """
    try:
        arr = pc2.read_points_numpy(msg, field_names=["x", "y", "z"])
        return np.asarray(arr, dtype=np.float32).reshape(-1, 3)
    except AssertionError:
        # Fallback: decode the raw byte buffer by field offset. Valid only for
        # the common un-packed little-endian layout (count-1 FLOAT32 x/y/z,
        # row_step*height == len(data)); anything else is rejected loudly
        # rather than silently misread.
        if msg.is_bigendian:
            raise ValueError("big-endian PointCloud2 is not supported")
        offsets: dict[str, int] = {}
        for name in ("x", "y", "z"):
            field = next((f for f in msg.fields if f.name == name), None)
            if field is None:
                raise ValueError(f"PointCloud2 missing '{name}' field")
            if field.datatype != PointField.FLOAT32 or field.count != 1:
                raise ValueError(
                    f"'{name}' must be a count-1 FLOAT32 field, "
                    f"got datatype={field.datatype} count={field.count}")
            if field.offset + 4 > msg.point_step:
                raise ValueError(f"'{name}' offset {field.offset} exceeds point_step")
            offsets[name] = field.offset
        if msg.row_step * msg.height != len(msg.data):
            raise ValueError("row_step*height does not match data size")
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
        if raw.shape[0] * msg.point_step != len(msg.data):
            raise ValueError("point_step does not match data size")
        cols = np.stack([
            raw[:, offsets[name]:offsets[name] + 4].view(np.float32)[:, 0]
            for name in ("x", "y", "z")
        ], axis=1)
        if cols.shape != (raw.shape[0], 3):
            raise ValueError("decoded x/y/z column layout mismatch")
        return cols


class PerceptionBridge(Node):
    """Dual-sensor point cloud -> world-frame ESDF + dynamic tracks + MoveIt collision objects."""

    def __init__(self) -> None:
        super().__init__("perception_bridge")
        self._base_cfg = load_point_cloud_collision()
        self._declare_parameters(self._base_cfg)

        # --- Calibration record (SSOT) gate ---------------------------------
        # Fail closed *before* anything is subscribed and before any timer
        # exists: a refusal here means the node never enters the perception
        # chain at all (ADR 0004 single authority, ADR 0009 grading table).
        self._world_frame = str(self.get_parameter("world_frame").value)
        self._node_identity = f"{self.get_name()}#{os.getpid()}"
        self._decision = self._load_calibration_gate()
        self._record_extrinsics = self._record_matrices()

        cfg = self._load_cfg()
        self._cfg = cfg
        self._spec: SafetyGridSpec = spec_of(cfg)

        self._max_points = int(self.get_parameter("max_points").value)
        self._safety_margin = float(self.get_parameter("safety_margin").value)
        self._sdf_far = float(self.get_parameter("sdf_far_distance").value)
        self._cloud_rate = float(self.get_parameter("publish_cloud_rate_hz").value)
        self._esdf_rate = float(self.get_parameter("publish_esdf_rate_hz").value)
        self._collision_rate = float(
            self.get_parameter("publish_collision_rate_hz").value)

        # Sensor parameters.
        self._input_frame_camera = str(self.get_parameter("input_frame").value)
        self._input_frame_lidar = str(self.get_parameter("input_frame_lidar").value)
        self._source_voxel_camera = float(
            self.get_parameter("source_voxel_camera_m").value)
        self._source_voxel_lidar = float(
            self.get_parameter("source_voxel_lidar_m").value)
        self._fusion_voxel = float(self.get_parameter("fusion_voxel_m").value)
        self._camera_max_age = float(
            self.get_parameter("camera_max_age_s").value)
        self._lidar_max_age = float(self.get_parameter("lidar_max_age_s").value)
        self._max_inter_sensor_dt = float(
            self.get_parameter("max_inter_sensor_dt_s").value)
        self._perception_timeout = float(
            self.get_parameter("perception_timeout_s").value)

        self._engine = FusionEngine(
            spec=self._spec,
            max_inter_sensor_dt_s=self._max_inter_sensor_dt,
            camera_max_age_s=self._camera_max_age,
            lidar_max_age_s=self._lidar_max_age,
            perception_timeout_s=self._perception_timeout,
            fusion_voxel_m=self._fusion_voxel,
            safety_margin=self._safety_margin,
            sdf_far=self._sdf_far,
            occupancy_timeout_s=float(
                self.get_parameter("occupancy_timeout_s").value),
            static_confirm_s=float(
                self.get_parameter("static_confirm_s").value),
            cluster_max_tracks=int(
                self.get_parameter("cluster_max_tracks").value),
            cluster_min_points=int(
                self.get_parameter("cluster_min_points").value),
            cluster_association_max_dist_m=float(
                self.get_parameter("cluster_association_max_dist_m").value),
            camera_buffer_maxlen=6,
            lidar_buffer_maxlen=3,
        )

        # Callback groups: sensor callbacks are reentrant (parallel),
        # fusion timer is mutually exclusive (serial, no competition).
        self._sensor_cbg = ReentrantCallbackGroup()
        self._fusion_cbg = MutuallyExclusiveCallbackGroup()

        # Runtime frame-mismatch reporting (throttled): a message whose
        # header.frame_id does not match the configured source frame is dropped
        # instead of being transformed with another source's matrix.  There is
        # no fallback matrix (T7 handoff 27-calibration-ssot.md:74).
        self._throttle_t: dict[str, float] = {}

        # Short-history buffers: (points, stamp_s, sensor_to_world) + Lock.
        self._camera_buffer: deque = deque(maxlen=6)
        self._camera_lock = Lock()
        self._lidar_buffer: deque = deque(maxlen=3)
        self._lidar_lock = Lock()

        # Sensor subscriptions (ReentrantCallbackGroup).
        source_topic = str(self.get_parameter("source_topic").value)
        self._camera_sub = None
        if source_topic:
            self._camera_sub = self.create_subscription(
                PointCloud2, source_topic, self._camera_callback,
                qos_profile_sensor_data, callback_group=self._sensor_cbg)

        source_topic_lidar = str(self.get_parameter("source_topic_lidar").value)
        self._lidar_sub = None
        if source_topic_lidar:
            self._lidar_sub = self.create_subscription(
                PointCloud2, source_topic_lidar, self._lidar_callback,
                qos_profile_sensor_data, callback_group=self._sensor_cbg)

        # JointState subscription for self-filtering (latest-only, no history buffer).
        # TODO(self-filter): 当前仅缓存, 未消费。需实现 FK→碰撞球体管线:
        #   JointState → pin.forward_kinematics → 碰撞球体 → robot_spheres 参数
        #   传入 engine.feed_camera(feed_lidar)。届时 _fusion_callback 需读取
        #   _latest_joint_state 并计算 robot_spheres 列表。
        self._latest_joint_state = None
        self._joint_lock = Lock()
        self._joint_sub = self.create_subscription(
            JointState, JOINT_STATE_TOPIC, self._joint_state_callback,
            qos_profile_sensor_data, callback_group=self._sensor_cbg)

        # Fusion timer: 20 Hz, MutuallyExclusiveCallbackGroup.
        self._fusion_timer = self.create_timer(
            0.05, self._fusion_callback, callback_group=self._fusion_cbg)

        # --- publishers ---
        self._cloud_pub = self.create_publisher(
            PointCloud2, "/perception/cloud_world", qos_profile_sensor_data)
        self._esdf_pub = self.create_publisher(
            Float32MultiArray, "/perception/esdf", qos_profile_sensor_data)
        self._esdf_meta_pub = self.create_publisher(
            Float32MultiArray, "/perception/esdf_meta", qos_profile_sensor_data)
        self._tracks_pub = self.create_publisher(
            Float32MultiArray, PERCEPTION_TRACKS_TOPIC, qos_profile_sensor_data)
        self._collision_pub = self.create_publisher(
            CollisionObject, "/collision_object", 10)
        self._instant_pub = self.create_publisher(
            PointCloud2, "/perception/instant_occupancy", qos_profile_sensor_data)
        self._status_pub = self.create_publisher(
            Float32MultiArray, "/perception/status", qos_profile_sensor_data)
        # Calibration diagnostics (ADR 0004 状态接口补充决议): report the identity
        # of the record this node actually loaded, per source, plus the gates.
        # Volatile QoS on purpose — a latched report would hand a late-joining
        # checker the identity of an instance that may already be gone, so
        # "received" would stop implying "just sent".  Consumers judge freshness
        # with their own monotonic clock (ROS time can pause, jump back, or
        # read zero).  Heartbeat 1 Hz + one immediate report at startup.
        self._calibration_pub = self.create_publisher(
            DiagnosticArray, PERCEPTION_CALIBRATION_STATUS_TOPIC, 10)

        self._last_cloud_t = 0.0
        self._last_esdf_t = 0.0
        self._last_collision_t = 0.0
        self._prev_collision_active: set[int] = set()
        self.get_logger().info(
            f"PERCEPTION_BRIDGE_STARTED camera={source_topic} "
            f"lidar={source_topic_lidar or '(none)'} "
            f"world={self._world_frame} fusion_voxel={self._fusion_voxel} "
            f"spec_shape={self._spec.shape}")
        identities = ", ".join(
            f"{name}={source.calibration_id}"
            for name, source in sorted(self._record.sources.items())
            if source.calibration_id)
        self.get_logger().info(
            f"CALIBRATION_RECORD_LOADED lookup={self._record.lookup_path} "
            f"resolved={self._record.resolved_path} "
            f"sha256={self._record.file_sha256} ids=[{identities}] "
            f"level={self._decision.level}")
        self._publish_calibration_status()
        self._calibration_timer = self.create_timer(
            1.0, self._publish_calibration_status)

    # ------------------------------------------------------------------
    # Parameter declaration
    # ------------------------------------------------------------------
    def _declare_parameters(self, base: PointCloudCollisionConfig) -> None:
        for cf in PointCloudCollisionConfig.config_fields():
            default = cf.default if cf.default is not None else getattr(base, cf.name)
            self.declare_parameter(cf.name, default)
        # workspace (ndarray -> list).
        self.declare_parameter("workspace_min", base.workspace_min.tolist())
        self.declare_parameter("workspace_max", base.workspace_max.tolist())
        # Removed extrinsics parameters: declared only so that a launch command
        # still passing them is detected instead of silently ignored.
        self._legacy_extrinsics_overrides: dict[str, object] = {}
        for name, default in _LEGACY_EXTRINSICS_DEFAULTS.items():
            try:
                self.declare_parameter(name, default)
            except Exception as exc:
                # e.g. an integer array overriding a double-array default: the
                # declaration itself fails, which is still "a removed parameter
                # was passed", so record it and let the gate refuse.
                self._legacy_extrinsics_overrides[name] = f"declaration rejected: {exc}"
                continue
            if not self._parameter_matches_default(name, default):
                self._legacy_extrinsics_overrides[name] = self.get_parameter(name).value
        # Calibration record (SSOT) input — ADR 0004 / ADR 0009.
        self.declare_parameter("calibration_record_path", "")
        # 默认值必须是非空字符串数组：rclpy 会把空列表默认值推断成 BYTE_ARRAY，之后
        # params 文件里的 ['camera']（STRING_ARRAY）就会在声明期抛
        # InvalidParameterTypeException（本机实测）。空串由 _read_string_array 过滤掉，
        # 所以 [""] 就是「没有任何豁免」。
        self.declare_parameter("allow_uncalibrated_debug", [""])
        self.declare_parameter("source_time_domain_camera", "")
        self.declare_parameter("source_time_domain_lidar", "")
        self.declare_parameter("publish_cloud_rate_hz", 5.0)
        self.declare_parameter("publish_esdf_rate_hz", 10.0)
        self.declare_parameter("publish_collision_rate_hz", 5.0)
        self.declare_parameter("collision_object_prefix", "camera_cluster")

    def _parameter_matches_default(self, name: str, default: object) -> bool:
        """探测已废弃参数是否被覆盖（值 != 内置默认即视为覆盖）。"""
        try:
            value = self.get_parameter(name).value
        except Exception:
            return False
        if isinstance(default, list):
            try:
                return [float(v) for v in value] == [float(v) for v in default]
            except (TypeError, ValueError):
                return value == default
        return value == default

    def _read_string_array(self, name: str) -> list[str]:
        """读取字符串数组参数，忽略空串。

        params 文件里的空 YAML 列表会让参数变成「未初始化」（读 ``.value`` 抛
        ``ParameterUninitializedException``，本机实测，见
        ``.scratch/off01-extrinsics-source/probe_bridge_params.py``），语义上等价于
        「没有声明任何一项」，所以按空列表处理并告警。
        """
        try:
            value = self.get_parameter(name).value
        except ParameterUninitializedException:
            self.get_logger().warn(
                f"{name} is declared but uninitialised (an empty YAML list in the"
                f" params file does that) — treated as empty")
            return []
        return [str(item) for item in (value or []) if str(item).strip()]

    # ------------------------------------------------------------------
    # Calibration record gate (ADR 0004 / ADR 0009)
    # ------------------------------------------------------------------
    def _load_calibration_gate(self) -> StartupDecision:
        """加载记录并做启动分级；拒绝时抛 :class:`CalibrationStartupRefused`。"""
        record_path = str(self.get_parameter("calibration_record_path").value or "")
        try:
            record = load_calibration_record(explicit=record_path or None)
        except CalibrationRecordError as exc:
            self._refuse_calibration(StartupDecision(
                start=False, level=LEVEL_ERROR,
                errors=(f"{exc.code}: {exc.message}",),
                warnings=(), notes=()))

        enabled = [
            name for name, topic in (
                ("camera", str(self.get_parameter("source_topic").value)),
                ("lidar", str(self.get_parameter("source_topic_lidar").value)),
            ) if topic
        ]
        frames = {
            "camera": (
                str(self.get_parameter("input_frame").value), self._world_frame),
            "lidar": (
                str(self.get_parameter("input_frame_lidar").value), self._world_frame),
        }
        decision = decide_startup(
            record,
            enabled_sources=enabled,
            exempt_sources=self._read_string_array("allow_uncalibrated_debug"),
            frame_expectations=frames,
            legacy_overrides=dict(self._legacy_extrinsics_overrides),
        )
        self._record = record
        for warning in decision.warnings:
            self.get_logger().warn(f"CALIBRATION_WARN {warning}")
        for note in decision.notes:
            self.get_logger().info(f"CALIBRATION_NOTE {note}")
        if not decision.start:
            self._refuse_calibration(decision)
        return decision

    def _refuse_calibration(self, decision: StartupDecision) -> None:
        for error in decision.errors:
            self.get_logger().error(f"CALIBRATION_REFUSED {error}")
        raise CalibrationStartupRefused(decision)

    def _record_matrices(self) -> dict[str, np.ndarray]:
        """逐源取记录里的 4x4（记录已在启动期校验，运行期不再回退）。"""
        matrices: dict[str, np.ndarray] = {}
        for name, source in self._record.sources.items():
            if source.matrix is None:
                continue
            if name in self._decision.sources and self._decision.sources[name].enabled:
                matrices[name] = source.matrix
        return matrices

    def _load_cfg(self) -> PointCloudCollisionConfig:
        """obstacle_params.yaml as truth source, ROS parameters as override layer."""
        base = load_point_cloud_collision()
        kwargs = {}
        for cf in PointCloudCollisionConfig.config_fields():
            raw = self.get_parameter(cf.name).value
            kwargs[cf.name] = cf.type_constructor(raw)
        kwargs["workspace_min"] = np.asarray(
            self.get_parameter("workspace_min").value, dtype=np.float64)
        kwargs["workspace_max"] = np.asarray(
            self.get_parameter("workspace_max").value, dtype=np.float64)
        kwargs["enabled"] = base.enabled
        return PointCloudCollisionConfig(**kwargs)

    # ------------------------------------------------------------------
    # Sensor callbacks (ReentrantCallbackGroup — decode + buffer only)
    # ------------------------------------------------------------------
    def _sensor_callback(
        self, msg: PointCloud2, source: str,
        input_frame: str,
        buffer: deque, lock: Lock,
        source_voxel_m: float,
    ) -> None:
        """Shared decode -> frame check -> preprocess -> buffer for both sensors.

        The transform is the one validated at startup, looked up by source; a
        message whose ``header.frame_id`` is not the configured frame is dropped
        instead of being transformed with another source's matrix (T7 handoff
        `27-calibration-ssot.md:74`).
        """
        if input_frame and msg.header.frame_id != input_frame:
            self._warn_throttled(
                f"frame-mismatch:{source}",
                f"{source}: dropping cloud whose header.frame_id="
                f"{msg.header.frame_id!r} is not the configured {input_frame!r}")
            return
        s2w = self._record_extrinsics.get(source)
        if s2w is None:
            self._warn_throttled(
                f"no-extrinsics:{source}",
                f"{source}: no calibration matrix for this source; cloud dropped")
            return
        try:
            sensor_pts = _points_xyz(msg)
        except Exception as exc:
            self.get_logger().warn(f"{source} decode failed: {exc}")
            return
        if sensor_pts.shape[0] == 0:
            return
        if sensor_pts.shape[0] > self._max_points:
            keep = np.random.choice(
                sensor_pts.shape[0], self._max_points, replace=False)
            sensor_pts = sensor_pts[keep]

        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        world = self._preprocess(sensor_pts, s2w, voxel_size=source_voxel_m)
        if world.shape[0] == 0:
            return
        with lock:
            buffer.append((world, stamp_s, s2w))

    def _lidar_callback(self, msg: PointCloud2) -> None:
        self._sensor_callback(
            msg, "lidar", self._input_frame_lidar,
            self._lidar_buffer, self._lidar_lock, self._source_voxel_lidar)

    def _camera_callback(self, msg: PointCloud2) -> None:
        self._sensor_callback(
            msg, "camera", self._input_frame_camera,
            self._camera_buffer, self._camera_lock, self._source_voxel_camera)

    def _warn_throttled(self, key: str, message: str, *, period_s: float = 1.0) -> None:
        """Rate-limit a per-message warning (monotonic clock, not ROS time)."""
        now = time.monotonic()
        last = self._throttle_t.get(key)
        if last is not None and now - last < period_s:
            return
        self._throttle_t[key] = now
        self.get_logger().warn(message)

    def _joint_state_callback(self, msg) -> None:
        with self._joint_lock:
            self._latest_joint_state = msg

    # ------------------------------------------------------------------
    # Preprocess
    # ------------------------------------------------------------------
    def _preprocess(
        self, sensor_pts: np.ndarray, sensor_to_world: np.ndarray,
        *, voxel_size: float | None = None,
    ) -> np.ndarray:
        """Transform -> workspace crop -> voxel downsample."""
        from work.safety_snapshot import preprocess_points
        return preprocess_points(sensor_pts, sensor_to_world, self._spec,
                                 voxel_size=voxel_size)

    # ------------------------------------------------------------------
    # Fusion timer (MutuallyExclusiveCallbackGroup, 20 Hz)
    # ------------------------------------------------------------------
    def _fusion_callback(self) -> None:
        # 1. Snapshot bridge buffers under lock -> feed into engine.
        with self._lidar_lock:
            lidar_snapshot = list(self._lidar_buffer)
        with self._camera_lock:
            camera_snapshot = list(self._camera_buffer)

        now_s = self.get_clock().now().nanoseconds * 1e-9

        self._engine.clear_buffers()
        for pts, stamp, _s2w in lidar_snapshot:
            self._engine.feed_lidar(pts, stamp)
        for pts, stamp, _s2w in camera_snapshot:
            self._engine.feed_camera(pts, stamp)

        result = self._engine.fuse(now_s)

        # 2. Publish result (even when empty — status still reports health).
        st = result.status
        try:
            self._publish_status(st, now_s)
            if st["source_count"] > 0.0:
                source_stamp = _stamp_to_msg(st["fusion_stamp"])
                self._maybe_publish_cloud(result.merged_points, now_s, source_stamp)
                self._maybe_publish_esdf(result.distance_field, now_s)
                self._publish_tracks(result.tracks, now_s)
                self._maybe_publish_collision(result.tracks, now_s)
                self._publish_instant(result.instant_points, source_stamp)
        except Exception as exc:
            self.get_logger().warn(f"perception publish failed: {exc}")

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------
    def _publish_status(self, st: dict, now_s: float) -> None:
        """Publish /perception/status from a FusionEngine status dict."""
        msg = Float32MultiArray()
        msg.data = [
            st["camera_alive"],
            st["lidar_alive"],
            st["camera_age"],
            st["lidar_age"],
            st["camera_used"],
            st["lidar_used"],
            st["fusion_stamp"],
            st["fusion_age"],
            st["source_count"],
            st["perception_valid"],
        ]
        self._status_pub.publish(msg)

    def _publish_calibration_status(self) -> None:
        """Publish ``/perception/calibration_status`` (startup report + 1 Hz heartbeat).

        Each entry carries the node instance identity plus the record identity
        this node actually loaded (lookup/resolved path, file sha256, per-source
        ``calibration_id``, gate results, declared time domain).  A consumer must
        judge freshness with its own monotonic clock: ROS time can pause, jump
        back or read zero, so the message timestamp is not a freshness proof.
        """
        time_domains = {
            "camera": str(self.get_parameter("source_time_domain_camera").value or ""),
            "lidar": str(self.get_parameter("source_time_domain_lidar").value or ""),
        }
        entries = calibration_status_entries(
            self._record, self._decision,
            node_identity=self._node_identity, time_domains=time_domains)
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        for entry in entries:
            status = DiagnosticStatus()
            status.name = str(entry["name"])
            # rclpy 把 DiagnosticStatus.level 的 `byte` 字段绑成「长度 1 的 bytes」而不是
            # int：直接赋 0/1/2 会在 assert 上崩（本机实测）。这里是唯一需要 bytes() 的地方，
            # 常量仍是 int，因为纯模块不依赖 diagnostic_msgs。
            status.level = bytes([DIAGNOSTIC_LEVELS[str(entry["level"])]])
            status.message = str(entry["message"])
            status.hardware_id = self._node_identity
            status.values = [
                KeyValue(key=str(key), value=str(value))
                for key, value in entry["values"].items()
            ]
            msg.status.append(status)
        self._calibration_pub.publish(msg)

    def _publish_instant(self, instant_pts: np.ndarray, stamp) -> None:
        if len(instant_pts) == 0:
            return
        header = self._make_header(self._world_frame, stamp)
        try:
            msg = pc2.create_cloud_xyz32(header, instant_pts.astype(np.float32))
            self._instant_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f"instant_occupancy publish failed: {exc}")

    def _maybe_publish_cloud(self, world: np.ndarray, now_s: float,
                             source_stamp) -> None:
        if now_s - self._last_cloud_t < 1.0 / max(self._cloud_rate, 0.1):
            return
        self._last_cloud_t = now_s
        header = self._make_header(self._world_frame, source_stamp)
        try:
            msg = pc2.create_cloud_xyz32(header, world.astype(np.float32))
            self._cloud_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f"cloud publish failed: {exc}")

    def _maybe_publish_esdf(self, sdf: np.ndarray, now_s: float) -> None:
        if now_s - self._last_esdf_t < 1.0 / max(self._esdf_rate, 0.1):
            return
        self._last_esdf_t = now_s
        msg = Float32MultiArray()
        shape = self._spec.shape
        strides = [shape[2] * shape[1], shape[2], 1]
        msg.layout = MultiArrayLayout(
            dim=[
                MultiArrayDimension(label="x", size=int(shape[0]), stride=int(strides[0])),
                MultiArrayDimension(label="y", size=int(shape[1]), stride=int(strides[1])),
                MultiArrayDimension(label="z", size=int(shape[2]), stride=int(strides[2])),
            ],
            data_offset=0,
        )
        msg.data = np.asarray(sdf, dtype=np.float32).ravel().tolist()
        self._esdf_pub.publish(msg)

        meta = Float32MultiArray()
        meta.data = [
            float(self._spec.workspace_min[0]),
            float(self._spec.workspace_min[1]),
            float(self._spec.workspace_min[2]),
            float(self._spec.voxel_size),
            1.0,  # valid
        ]
        self._esdf_meta_pub.publish(meta)

    def _publish_tracks(self, tracks: TrackState, now_s: float) -> None:
        data = np.zeros(MAX_DYNAMIC_TRACKS * _TRACK_SLOT_FLOATS, dtype=np.float32)
        d_safe = float(self._safety_margin)
        for i in range(MAX_DYNAMIC_TRACKS):
            slot = i * _TRACK_SLOT_FLOATS
            data[slot:slot + 3] = tracks.pos[i]
            data[slot + 3] = tracks.radii[i]
            data[slot + 4:slot + 7] = tracks.vel[i]
            data[slot + 7] = tracks.enabled[i]
            data[slot + 8] = d_safe
            data[slot + 9] = 1.5  # alpha
        msg = Float32MultiArray()
        msg.data = data.tolist()
        self._tracks_pub.publish(msg)

    def _maybe_publish_collision(self, tracks: TrackState, now_s: float) -> None:
        if now_s - self._last_collision_t < 1.0 / max(self._collision_rate, 0.1):
            return
        self._last_collision_t = now_s
        prefix = str(self.get_parameter("collision_object_prefix").value)
        header = self._make_header(self._world_frame, None)
        published = set()
        for i in range(MAX_DYNAMIC_TRACKS):
            if tracks.enabled[i] <= 0.0:
                continue
            published.add(i)
            obj = CollisionObject()
            obj.header = header
            obj.id = f"{prefix}_{i}"
            obj.operation = CollisionObject.ADD
            p = Pose()
            p.position = Point(
                x=float(tracks.pos[i, 0]),
                y=float(tracks.pos[i, 1]),
                z=float(tracks.pos[i, 2]))
            p.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            obj.pose = p
            sphere = SolidPrimitive()
            sphere.type = SolidPrimitive.SPHERE
            sphere.dimensions = [
                float(tracks.radii[i]) + self._safety_margin]
            obj.primitives = [sphere]
            obj.primitive_poses = [Pose()]
            self._collision_pub.publish(obj)
        # Remove slots that were active last frame but are now gone.
        for i in self._prev_collision_active - published:
            obj = CollisionObject()
            obj.header = header
            obj.id = f"{prefix}_{i}"
            obj.operation = CollisionObject.REMOVE
            self._collision_pub.publish(obj)
        self._prev_collision_active = published

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _make_header(self, frame_id: str, source_stamp):
        from std_msgs.msg import Header
        header = Header()
        header.frame_id = frame_id
        header.stamp = (source_stamp if source_stamp is not None
                        else self.get_clock().now().to_msg())
        return header


def _stamp_to_msg(stamp_s: float):
    """Convert seconds (float) to builtin_interfaces/Time message."""
    from builtin_interfaces.msg import Time
    sec = int(stamp_s)
    nanosec = int((stamp_s - sec) * 1e9)
    return Time(sec=sec, nanosec=nanosec)


REFUSAL_EXIT_CODE = 3


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = PerceptionBridge()
    except CalibrationStartupRefused as exc:
        # Fail closed: no subscriptions, no timers, no perception output.  The
        # nonzero exit code and the absence of a current-instance report on
        # /perception/calibration_status are the observable signals.
        print("PERCEPTION_BRIDGE_CALIBRATION_REFUSED", file=sys.stderr)
        for error in exc.decision.errors:
            print(f"  {error}", file=sys.stderr)
        rclpy.logging.get_logger("perception_bridge").fatal(
            f"calibration gate refused startup: {exc}")
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(REFUSAL_EXIT_CODE)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
