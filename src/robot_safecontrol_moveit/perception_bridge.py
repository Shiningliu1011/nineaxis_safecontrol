"""Perception bridge: live Orbbec depth cloud -> ESDF + dynamic tracks + CollisionObjects.

Subscribes to the camera's registered point cloud (``/camera/depth_registered/points``),
transforms it into a world frame (default: the camera optical frame as a placeholder —
set ``world_frame`` to ``base_link`` once a real TF is available), separates persistent
(static) from transient (dynamic) points, and publishes:

* ``/perception/cloud_world``   — processed voxel cloud (sensor_msgs/PointCloud2)
* ``/perception/esdf``          — fixed-shape distance field (Float32MultiArray)
* ``/perception/esdf_meta``     — [origin_x, origin_y, origin_z, voxel_size, valid]
* ``/perception/tracks``        — 8 fixed dynamic-obstacle slots (Float32MultiArray)
* ``/collision_object``         — SPHERE primitives for MoveIt planning scene

All perception topics use best-effort QoS (``qos_profile_sensor_data``) — subscribers
must match.  The ``work/`` modules (portable_oscbf) stay pure Python; all ROS code
lives here.

Usage::

    ros2 run robot_safecontrol_moveit perception_bridge
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point, Quaternion

# --- portable_oscbf on path (pure-python calculation core) --------------------
_REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_REPO_ROOT / "portable_oscbf"))
sys.path.insert(0, str(_REPO_ROOT / "portable_oscbf" / "work"))

from work.perception_config import (  # noqa: E402
    PointCloudCollisionConfig,
    load_point_cloud_collision,
    spec_of,
)
from work.perception_config import ConfigField  # noqa: E402
from work.static_occupancy import OccupancyTracker  # noqa: E402
from work.dynamic_clustering import (  # noqa: E402
    TrackState,
    cluster_into_tracks,
    empty_track_state,
)
from work.safety_snapshot import (  # noqa: E402
    MAX_DYNAMIC_TRACKS,
    SafetyGridSpec,
    build_distance_field,
)

# Each /perception/tracks slot carries 10 floats.
_TRACK_SLOT_FLOATS = 10  # px,py,pz, r, vx,vy,vz, enabled, d_safe, alpha


def _static_identity() -> np.ndarray:
    return np.eye(4, dtype=np.float64)


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


class PerceptionBridge(Node):
    """Point cloud -> world-frame ESDF + dynamic tracks + MoveIt collision objects."""

    def __init__(self) -> None:
        super().__init__("perception_bridge")
        # 先读 obstacle_params.yaml, 用它作参数默认值 → 与 demo 的 spec 严格一致,
        # 保证 sdf_shape 相同、JAX 不触发重编译。
        self._base_cfg = load_point_cloud_collision()
        self._declare_parameters(self._base_cfg)

        cfg = self._load_cfg()
        self._cfg = cfg
        self._spec: SafetyGridSpec = spec_of(cfg)

        self._input_frame = str(self.get_parameter("input_frame").value)
        self._world_frame = str(self.get_parameter("world_frame").value)
        self._use_tf = bool(self.get_parameter("use_tf").value)
        self._max_points = int(self.get_parameter("max_points").value)
        self._safety_margin = float(self.get_parameter("safety_margin").value)
        self._sdf_far = float(self.get_parameter("sdf_far_distance").value)
        self._cloud_rate = float(self.get_parameter("publish_cloud_rate_hz").value)
        self._esdf_rate = float(self.get_parameter("publish_esdf_rate_hz").value)
        self._collision_rate = float(self.get_parameter(
            "publish_collision_rate_hz").value)

        self._occupancy_tracker = OccupancyTracker(
            self._spec,
            occupancy_timeout_s=float(self.get_parameter(
                "occupancy_timeout_s").value),
            static_confirm_s=float(self.get_parameter(
                "static_confirm_s").value))
        self._prev_tracks: TrackState = empty_track_state(MAX_DYNAMIC_TRACKS)
        self._next_track_id = 1
        self._frame_seq = 0

        # TF (first use in the repo): only needed when world_frame != input_frame.
        self._tf_buffer = None
        self._tf_listener = None
        if self._use_tf and self._world_frame != self._input_frame:
            from tf2_ros import Buffer, TransformListener
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)

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

        # --- camera subscription ---
        source_topic = str(self.get_parameter("source_topic").value)
        self._sub = self.create_subscription(
            PointCloud2, source_topic, self._cloud_callback,
            qos_profile_sensor_data, callback_group=ReentrantCallbackGroup())

        self._last_cloud_t = 0.0
        self._last_esdf_t = 0.0
        self._last_collision_t = 0.0
        self._prev_collision_active: set[int] = set()
        self.get_logger().info(
            f"PERCEPTION_BRIDGE_STARTED source={source_topic} "
            f"input={self._input_frame} world={self._world_frame} "
            f"voxel={cfg.voxel_size} spec_shape={self._spec.shape}")

    # ------------------------------------------------------------------
    # Parameter declaration
    # ------------------------------------------------------------------
    def _declare_parameters(self, base: PointCloudCollisionConfig) -> None:
        # 从 dataclass config_fields() 自动声明所有配置参数。
        # 无默认值的字段用 base 中加载的值作为默认。
        for cf in PointCloudCollisionConfig.config_fields():
            default = cf.default if cf.default is not None else getattr(base, cf.name)
            self.declare_parameter(cf.name, default)
        # workspace 特殊处理 (ndarray → list)。
        self.declare_parameter("workspace_min", base.workspace_min.tolist())
        self.declare_parameter("workspace_max", base.workspace_max.tolist())
        # 桥接节点专属参数 (不在 PointCloudCollisionConfig 中)。
        self.declare_parameter("use_tf", False)
        # 4x4 单位阵 (camera==world), 避免 rclpy 把空列表推断成 BYTE_ARRAY。
        self.declare_parameter(
            "camera_to_world_static",
            [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("publish_cloud_rate_hz", 5.0)
        self.declare_parameter("publish_esdf_rate_hz", 10.0)
        self.declare_parameter("publish_collision_rate_hz", 5.0)
        self.declare_parameter("collision_object_prefix", "camera_cluster")

    def _load_cfg(self) -> PointCloudCollisionConfig:
        """obstacle_params.yaml 为真源, ROS 参数作为覆盖层。"""
        base = load_point_cloud_collision()
        # 从 config_fields() 自动读取所有配置参数。
        kwargs = {}
        for cf in PointCloudCollisionConfig.config_fields():
            raw = self.get_parameter(cf.name).value
            kwargs[cf.name] = cf.type_constructor(raw)
        # workspace 特殊处理 (list → ndarray)。
        kwargs["workspace_min"] = np.asarray(
            self.get_parameter("workspace_min").value, dtype=np.float64)
        kwargs["workspace_max"] = np.asarray(
            self.get_parameter("workspace_max").value, dtype=np.float64)
        kwargs["enabled"] = base.enabled
        return PointCloudCollisionConfig(**kwargs)

    # ------------------------------------------------------------------
    # Transform
    # ------------------------------------------------------------------
    def _sensor_to_world(self) -> np.ndarray:
        if self._use_tf and self._tf_buffer is not None and \
                self._world_frame != self._input_frame:
            try:
                t = self._tf_buffer.lookup_transform(
                    self._world_frame, self._input_frame, Time())
                return _tf_to_matrix(t)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    f"TF lookup {self._input_frame}->{self._world_frame} failed "
                    f"({exc}); falling back to identity")
        static = self.get_parameter("camera_to_world_static").value
        if static:
            arr = np.asarray(static, dtype=np.float64).reshape(4, 4)
            return arr
        return _static_identity()

    # ------------------------------------------------------------------
    # Main callback
    # ------------------------------------------------------------------
    def _cloud_callback(self, msg: PointCloud2) -> None:
        self._frame_seq += 1
        try:
            arr = pc2.read_points_numpy(msg, field_names=["x", "y", "z"])
            sensor_pts = np.asarray(arr, dtype=np.float32).reshape(-1, 3)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"PointCloud2 decode failed: {exc}")
            return
        if sensor_pts.shape[0] == 0:
            return

        # 大点云均匀抽稀, 控制变换开销。
        if sensor_pts.shape[0] > self._max_points:
            keep = np.random.choice(
                sensor_pts.shape[0], self._max_points, replace=False)
            sensor_pts = sensor_pts[keep]

        sensor_to_world = self._sensor_to_world()

        # 变换 + 裁剪 + 体素降采样 + (可选) 机器人球剔除。
        world = self._preprocess(sensor_pts, sensor_to_world)
        if world.shape[0] == 0:
            return

        # 三层占据分离。
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        static_pts, unconfirmed_pts, instant_pts = self._occupancy_tracker.update(
            world, stamp_s)

        # ESDF (静态环境)。
        sdf = build_distance_field(
            static_pts, self._spec, far_distance=self._sdf_far)

        # 动态聚类 → 8 槽。
        new_tracks, self._next_track_id = cluster_into_tracks(
            unconfirmed_pts, self._prev_tracks, self._spec,
            max_tracks=int(self.get_parameter("cluster_max_tracks").value),
            min_points=int(self.get_parameter("cluster_min_points").value),
            asso_max_dist_m=float(
                self.get_parameter("cluster_association_max_dist_m").value),
            next_id=self._next_track_id,
        )
        self._prev_tracks = new_tracks

        now = self.get_clock().now().nanoseconds * 1e-9
        try:
            self._maybe_publish_cloud(world, now, msg.header.stamp)
            self._maybe_publish_esdf(sdf, now)
            self._publish_tracks(new_tracks, now)
            self._maybe_publish_collision(new_tracks, now)
        except Exception as exc:  # noqa: BLE001 — 感知节点不因单个错误崩掉
            self.get_logger().warn(f"perception publish failed: {exc}")

    def _preprocess(self, sensor_pts: np.ndarray, sensor_to_world: np.ndarray):
        from work.safety_snapshot import preprocess_points
        return preprocess_points(sensor_pts, sensor_to_world, self._spec)

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------
    def _maybe_publish_cloud(self, world: np.ndarray, now_s: float,
                             source_stamp) -> None:
        if now_s - self._last_cloud_t < 1.0 / max(self._cloud_rate, 0.1):
            return
        self._last_cloud_t = now_s
        header = self._make_header(self._world_frame, source_stamp)
        try:
            msg = pc2.create_cloud_xyz32(header, world.astype(np.float32))
            self._cloud_pub.publish(msg)
        except Exception as exc:  # noqa: BLE001
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
        # 上一帧激活、本帧消失的槽 → REMOVE (清理过期对象)。
        if self._prev_collision_active is not None:
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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionBridge()
    executor = MultiThreadedExecutor(num_threads=2)
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
