"""Perception bridge: dual-sensor fusion -> ESDF + dynamic tracks + CollisionObjects.

Subscribes to LiDAR (``/livox/lidar``) and Camera (``/camera/depth_registered/points``)
point clouds.  Sensor callbacks (ReentrantCallbackGroup) only decode -> TF -> ROI ->
source voxel -> push to deque.  A 20 Hz fusion timer (MutuallyExclusiveCallbackGroup)
performs timestamp pairing, self-filtering, fusion voxel downsampling, three-layer
classification, ESDF construction, dynamic clustering, and publishes:

* ``/perception/cloud_world``        — fused voxel cloud (sensor_msgs/PointCloud2)
* ``/perception/esdf``               — fixed-shape distance field (Float32MultiArray)
* ``/perception/esdf_meta``          — [origin_x, origin_y, origin_z, voxel_size, valid]
* ``/perception/tracks``             — 8 fixed dynamic-obstacle slots (Float32MultiArray)
* ``/collision_object``              — SPHERE primitives for MoveIt planning scene
* ``/perception/instant_occupancy``  — instant safety channel (PointCloud2)
* ``/perception/status``             — 10-element health status (Float32MultiArray)

When ``source_topic_lidar`` is empty, LiDAR is not subscribed and the node behaves
identically to the original single-camera version.

Usage::

    ros2 run robot_safecontrol_moveit perception_bridge
"""

from __future__ import annotations

import sys
from collections import deque
from pathlib import Path
from threading import Lock

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point, Quaternion
from robot_safecontrol_moveit.ros_conventions import JOINT_STATE_TOPIC

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


def _identity_extrinsics() -> np.ndarray:
    return np.eye(4, dtype=np.float64)




class PerceptionBridge(Node):
    """Dual-sensor point cloud -> world-frame ESDF + dynamic tracks + MoveIt collision objects."""

    def __init__(self) -> None:
        super().__init__("perception_bridge")
        self._base_cfg = load_point_cloud_collision()
        self._declare_parameters(self._base_cfg)

        cfg = self._load_cfg()
        self._cfg = cfg
        self._spec: SafetyGridSpec = spec_of(cfg)

        self._world_frame = str(self.get_parameter("world_frame").value)
        self._use_tf = bool(self.get_parameter("use_tf").value)
        self._max_points = int(self.get_parameter("max_points").value)
        self._safety_margin = float(self.get_parameter("safety_margin").value)
        self._sdf_far = float(self.get_parameter("sdf_far_distance").value)
        self._cloud_rate = float(self.get_parameter("publish_cloud_rate_hz").value)
        self._esdf_rate = float(self.get_parameter("publish_esdf_rate_hz").value)
        self._collision_rate = float(
            self.get_parameter("publish_collision_rate_hz").value)

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

        # TF listener (shared across sensors).
        self._tf_buffer = None
        self._tf_listener = None
        if self._use_tf:
            from tf2_ros import Buffer, TransformListener
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)

        # Callback groups: sensor callbacks are reentrant (parallel),
        # fusion timer is mutually exclusive (serial, no competition).
        self._sensor_cbg = ReentrantCallbackGroup()
        self._fusion_cbg = MutuallyExclusiveCallbackGroup()

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

        # Short-history buffers: (points, stamp_s, sensor_to_world) + Lock.
        self._camera_buffer: deque = deque(maxlen=6)
        self._camera_lock = Lock()
        self._lidar_buffer: deque = deque(maxlen=3)
        self._lidar_lock = Lock()

        # Sensor subscriptions (ReentrantCallbackGroup).
        source_topic = str(self.get_parameter("source_topic").value)
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
            Float32MultiArray, "/perception/tracks", qos_profile_sensor_data)
        self._collision_pub = self.create_publisher(
            CollisionObject, "/collision_object", 10)
        self._instant_pub = self.create_publisher(
            PointCloud2, "/perception/instant_occupancy", qos_profile_sensor_data)
        self._status_pub = self.create_publisher(
            Float32MultiArray, "/perception/status", qos_profile_sensor_data)

        self._last_cloud_t = 0.0
        self._last_esdf_t = 0.0
        self._last_collision_t = 0.0
        self._prev_collision_active: set[int] = set()
        self.get_logger().info(
            f"PERCEPTION_BRIDGE_STARTED camera={source_topic} "
            f"lidar={source_topic_lidar or '(none)'} "
            f"world={self._world_frame} fusion_voxel={self._fusion_voxel} "
            f"spec_shape={self._spec.shape}")

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
        # Bridge-only parameters (not in PointCloudCollisionConfig).
        self.declare_parameter("use_tf", False)
        self.declare_parameter(
            "camera_to_world_static",
            [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter(
            "lidar_to_world_static",
            [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("publish_cloud_rate_hz", 5.0)
        self.declare_parameter("publish_esdf_rate_hz", 10.0)
        self.declare_parameter("publish_collision_rate_hz", 5.0)
        self.declare_parameter("collision_object_prefix", "camera_cluster")

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
    # Transform lookup
    # ------------------------------------------------------------------
    def _sensor_to_world(
        self, sensor_frame: str, stamp, static_param_name: str,
    ) -> np.ndarray:
        """Look up sensor -> world transform at the given message stamp.

        Tries TF first (using the message stamp for motion compensation),
        falls back to the latest available TF, then to the static 4x4 parameter.
        """
        if self._use_tf and self._tf_buffer is not None \
                and self._world_frame != sensor_frame:
            from tf2_ros import ConnectivityException, ExtrapolationException, \
                LookupException
            try:
                t = self._tf_buffer.lookup_transform(
                    self._world_frame, sensor_frame, stamp)
                return _tf_to_matrix(t)
            except (LookupException, ConnectivityException,
                    ExtrapolationException) as exc:
                self.get_logger().warn(
                    f"TF {sensor_frame}->{self._world_frame} at stamp: {exc}")
            try:
                t = self._tf_buffer.lookup_transform(
                    self._world_frame, sensor_frame, Time())
                return _tf_to_matrix(t)
            except (LookupException, ConnectivityException,
                    ExtrapolationException) as exc:
                self.get_logger().warn(
                    f"TF {sensor_frame}->{self._world_frame} latest: {exc}")
        static = self.get_parameter(static_param_name).value
        if static:
            return np.asarray(static, dtype=np.float64).reshape(4, 4)
        return _identity_extrinsics()

    # ------------------------------------------------------------------
    # Sensor callbacks (ReentrantCallbackGroup — decode + buffer only)
    # ------------------------------------------------------------------
    def _sensor_callback(
        self, msg: PointCloud2, label: str,
        input_frame: str, static_param: str,
        buffer: deque, lock: Lock,
        source_voxel_m: float,
    ) -> None:
        """Shared decode -> TF -> preprocess -> buffer for both sensors."""
        try:
            arr = pc2.read_points_numpy(msg, field_names=["x", "y", "z"])
            sensor_pts = np.asarray(arr, dtype=np.float32).reshape(-1, 3)
        except Exception as exc:
            self.get_logger().warn(f"{label} decode failed: {exc}")
            return
        if sensor_pts.shape[0] == 0:
            return
        if sensor_pts.shape[0] > self._max_points:
            keep = np.random.choice(
                sensor_pts.shape[0], self._max_points, replace=False)
            sensor_pts = sensor_pts[keep]

        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        s2w = self._sensor_to_world(input_frame, msg.header.stamp, static_param)
        world = self._preprocess(sensor_pts, s2w, voxel_size=source_voxel_m)
        if world.shape[0] == 0:
            return
        with lock:
            buffer.append((world, stamp_s, s2w))

    def _lidar_callback(self, msg: PointCloud2) -> None:
        self._sensor_callback(
            msg, "LiDAR", self._input_frame_lidar,
            "lidar_to_world_static", self._lidar_buffer, self._lidar_lock,
            self._source_voxel_lidar)

    def _camera_callback(self, msg: PointCloud2) -> None:
        self._sensor_callback(
            msg, "Camera", self._input_frame_camera,
            "camera_to_world_static", self._camera_buffer, self._camera_lock,
            self._source_voxel_camera)

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


def _tf_to_matrix(transform_stamped) -> np.ndarray:
    """geometry_msgs/TransformStamped -> 4x4 homogeneous float64."""
    t = transform_stamped.transform.translation
    q = transform_stamped.transform.rotation
    norm2 = float(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm2 <= 0.0:
        return np.eye(4, dtype=np.float64)
    x, y, z, w = (q.x, q.y, q.z, q.w) if norm2 == 1.0 else (
        q.x / norm2**0.5, q.y / norm2**0.5, q.z / norm2**0.5, q.w / norm2**0.5)
    mat = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), t.x],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), t.y],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), t.z],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=np.float64)
    return mat


def _stamp_to_msg(stamp_s: float):
    """Convert seconds (float) to builtin_interfaces/Time message."""
    from builtin_interfaces.msg import Time
    sec = int(stamp_s)
    nanosec = int((stamp_s - sec) * 1e9)
    return Time(sec=sec, nanosec=nanosec)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionBridge()
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
