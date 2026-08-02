"""Display the project URDF in MuJoCo and mirror ROS 2 joint feedback.

This is deliberately a visualisation bridge, not a second planner, IK solver,
or controller.  MoveIt 2 remains responsible for planning and collision
checking; this node loads ``ninezzhou.urdf`` and its STL meshes from this
repository, then writes the received ``/joint_states`` values to MuJoCo's
``qpos`` array for inspection.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from html import escape
from math import isfinite, sqrt
from pathlib import Path
from tempfile import TemporaryDirectory
from time import monotonic, sleep
from typing import Iterable, Sequence

import mujoco
import numpy as np
import mujoco.viewer
import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from moveit_msgs.msg import CollisionObject
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from .motion_planning import CollisionObjectSpec
from .plan_transition import (
    DEFAULT_JOINT_NAMES,
    load_collision_objects,
    load_mat_trajectory,
)


# The project URDF is authored in the legacy Y-up convention.  The wrapper is
# display-only: it rotates both robot and annotations for MuJoCo's Z-up view,
# while joint qpos values retain their URDF/MoveIt units and signs unchanged.
Y_UP_TO_Z_UP_EULER = "1.5707963267948966 0 0"
# Quaternion (wxyz) for the Y-up->Z-up rotation about X by +90 deg.  Free-joint
# obstacle slots live at worldbody level (MuJoCo requires free joints at the
# top level), so base_link poses are rotated by this before writing qpos.
_Y_UP_TO_Z_UP_QUAT_WXYZ = (0.7071067811865476, 0.7071067811865476, 0.0, 0.0)

# Pre-allocated free-body "slots" used to render live dynamic obstacles
# (from /collision_object MOVE/ADD) without recompiling the MuJoCo model.
# Order matters: sphere, box, cylinder.
DEFAULT_OBSTACLE_SLOT_COUNTS = (16, 16, 8)
STATIC_COLLISION_OBJECT_TOPIC = "/static_collision_object"
DYNAMIC_COLLISION_OBJECT_TOPIC = "/collision_object"


def static_collision_qos(depth: int = 100) -> QoSProfile:
    """QoS contract for retained static collision objects."""
    return QoSProfile(
        depth=max(1, int(depth)),
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


def dynamic_collision_qos(depth: int = 100) -> QoSProfile:
    """QoS contract for live dynamic collision-object updates."""
    return QoSProfile(
        depth=max(1, int(depth)),
        durability=DurabilityPolicy.VOLATILE,
        reliability=ReliabilityPolicy.RELIABLE,
    )

# Default geom size per slot type (m): sphere radius; box half-extents; cylinder (radius, half-height).
_DEFAULT_SLOT_SIZES = {
    "sphere": (0.05, 0.0, 0.0),
    "box": (0.05, 0.05, 0.05),
    "cylinder": (0.05, 0.05, 0.0),
}
# Shapes names must match CollisionObject primitive types for size parsing.
_SLOT_SHAPE_BY_PRIMITIVE = {
    SolidPrimitive.BOX: "box",
    SolidPrimitive.SPHERE: "sphere",
    SolidPrimitive.CYLINDER: "cylinder",
}


@dataclass(frozen=True)
class TrackingCylinderSpec:
    """Display-only cylinder expressed in the MoveIt ``base_link`` frame."""

    center: tuple[float, float, float]
    axis_direction: tuple[float, float, float]
    radius: float
    height: float
    radial_rms_error: float
    radial_max_error: float


class MuJoCoJointStateViewer(Node):
    """A passive MuJoCo view of the project arm driven by ``/joint_states``."""

    def __init__(self) -> None:
        super().__init__("mujoco_joint_state_viewer")
        self._declare_parameters()

        joint_names = tuple(str(name) for name in self.get_parameter("joint_names").value)
        if not joint_names:
            raise ValueError("joint_names must not be empty")
        self._joint_names = joint_names

        urdf_path = self._file_parameter("urdf_path", self._default_urdf_path())
        mesh_directory = self._directory_parameter(
            "mesh_directory", self._default_mesh_directory()
        )
        show_target_path = bool(self.get_parameter("show_target_path").value)
        target_path: list[tuple[float, float, float]] = []
        full_path: list[tuple[float, float, float]] = []
        if show_target_path:
            trajectory_mat = self._file_parameter(
                "trajectory_mat", self._default_trajectory_mat()
            )
            trajectory_offset_m = self._float_tuple("trajectory_offset_m", 3)
            # Load the *full* trajectory once: the cylinder must be fitted on
            # every sample.  A short (possibly stationary) selection of points
            # makes the least-squares circle fit degenerate and the normal
            # direction meaningless.
            full_path, _ = load_mat_trajectory(
                trajectory_mat, trajectory_offset_m, max_points=0, point_stride=1
            )
            target_path = self._sample_display_path(
                full_path, int(self.get_parameter("path_max_points").value)
            )

        tracking_cylinder: TrackingCylinderSpec | None = None
        if (
            full_path
            and bool(self.get_parameter("show_tracking_cylinder").value)
        ):
            tracking_cylinder = self._fit_tracking_cylinder(
                full_path,
                self._float_tuple("tracking_cylinder_axis_direction", 3),
                float(self.get_parameter("tracking_cylinder_height_margin_m").value),
                extend_to_ground=bool(
                    self.get_parameter("tracking_cylinder_extend_to_ground").value
                ),
            )
            if bool(self.get_parameter("project_display_path_to_cylinder").value):
                target_path = self._project_path_to_cylinder(
                    target_path,
                    tracking_cylinder,
                    float(self.get_parameter("path_surface_offset_m").value),
                )
            self.get_logger().info(
                "Tracking cylinder fitted on full trajectory in base_link: "
                f"center={tracking_cylinder.center}, "
                f"axis={tracking_cylinder.axis_direction}, "
                f"radius={tracking_cylinder.radius:.6f} m, "
                f"height={tracking_cylinder.height:.6f} m, "
                f"radial RMS={tracking_cylinder.radial_rms_error * 1000.0:.3f} mm, "
                f"max={tracking_cylinder.radial_max_error * 1000.0:.3f} mm."
            )

        raw_mjcf = self._urdf_to_mjcf(urdf_path, mesh_directory)
        slot_counts = tuple(
            int(value)
            for value in self.get_parameter("obstacle_slot_counts").value
        )
        if len(slot_counts) != 3:
            raise ValueError("obstacle_slot_counts must contain exactly three values")
        if any(count < 0 for count in slot_counts):
            raise ValueError("obstacle_slot_counts values must all be non-negative")
        # Both retained static objects and live dynamic objects are displayed
        # through pre-allocated slots. Do NOT inject a second obstacle copy
        # into MJCF, otherwise the Viewer can diverge from MoveIt.
        scene_mjcf = self._inject_display_scene(
            raw_mjcf, target_path, (), tracking_cylinder, slot_counts,
            joint_names=self._joint_names,
        )
        self._model = mujoco.MjModel.from_xml_string(scene_mjcf)
        self._model.opt.gravity[:] = 0.0
        self._data = mujoco.MjData(self._model)
        self._qpos_addresses = self._joint_qpos_addresses(joint_names)
        self._joint_limits = self._read_joint_limits(joint_names)
        self._latest_positions: dict[str, float] = {}
        self._received_joint_state = False
        self._manual_mode = False
        self.selected_joint: int = -1  # -1 = none selected

        self._dynamic_color = [
            float(value) for value in self.get_parameter("dynamic_obstacle_color").value
        ]
        if len(self._dynamic_color) != 4:
            raise ValueError("dynamic_obstacle_color must contain four values")
        self._setup_obstacle_slots(slot_counts)

        # Create the manual-joint-state publisher at init time (never lazily).
        manual_topic = str(self.get_parameter("manual_joint_state_topic").value)
        self._manual_pose_pub = self.create_publisher(
            JointState, manual_topic, qos_profile_sensor_data
        )
        self._manual_publish_rate_hz = float(
            self.get_parameter("manual_joint_state_publish_rate_hz").value
        )
        if self._manual_publish_rate_hz <= 0.0:
            raise ValueError(
                "manual_joint_state_publish_rate_hz must be positive"
            )
        self._manual_publish_timer = None  # created when entering manual mode

        topic = str(self.get_parameter("joint_state_topic").value)
        self.create_subscription(
            JointState,
            topic,
            self._joint_state_callback,
            qos_profile_sensor_data,
        )
        if bool(self.get_parameter("show_obstacles").value):
            self.create_subscription(
                CollisionObject,
                str(self.get_parameter("static_collision_object_topic").value),
                self._collision_object_callback,
                static_collision_qos(),
            )
        if bool(self.get_parameter("show_dynamic_obstacles").value):
            self.create_subscription(
                CollisionObject,
                str(self.get_parameter("collision_object_topic").value),
                self._collision_object_callback,
                dynamic_collision_qos(),
            )

        # Viewer mode-control service so plan_transition can switch the
        # Viewer to ROS-tracking mode before replay.
        from std_srvs.srv import SetBool

        self._mode_service = self.create_service(
            SetBool, "set_mujoco_manual_mode", self._mode_service_callback
        )

        # Transition planning client (T key triggers /plan_transition_once).
        from std_srvs.srv import Trigger

        self._transition_client = self.create_client(
            Trigger, "/plan_transition_once"
        )
        self._transition_future = None
        self._transition_status: str = ""

        self.get_logger().info(
            f"MuJoCo loaded project model {urdf_path} ({self._model.njnt} joints, "
            f"{self._model.ngeom} geoms); mirroring {topic}."
        )

    @property
    def model(self) -> mujoco.MjModel:
        return self._model

    @property
    def data(self) -> mujoco.MjData:
        return self._data

    def viewer_rate_hz(self) -> float:
        rate_hz = float(self.get_parameter("viewer_rate_hz").value)
        if rate_hz <= 0.0:
            raise ValueError("viewer_rate_hz must be positive")
        return rate_hz

    def apply_latest_joint_state(self) -> bool:
        """Copy the latest complete joint state to MuJoCo and run forward
        kinematics.  Skipped when manual mode is active so the user's
        slider adjustments are not overwritten by ROS."""
        if not self._received_joint_state:
            return False
        if self._manual_mode:
            return False
        for name, qpos_address in self._qpos_addresses.items():
            self._data.qpos[qpos_address] = self._latest_positions[name]
        mujoco.mj_forward(self._model, self._data)
        return True

    @property
    def manual_mode(self) -> bool:
        return self._manual_mode

    def toggle_manual_mode(self) -> bool:
        """Toggle between ROS-tracking and manual-joint-adjustment mode.

        When switching to manual mode the current ROS joint state is applied
        once as a starting point.  Returns the new mode (True = manual).
        """
        self._manual_mode = not self._manual_mode
        if self._manual_mode:
            # Seed the arm at the last known ROS pose.
            if self._received_joint_state:
                for name, qpos_address in self._qpos_addresses.items():
                    self._data.qpos[qpos_address] = self._latest_positions[name]
                mujoco.mj_forward(self._model, self._data)
            # Start continuous manual joint state publishing.
            period_s = 1.0 / self._manual_publish_rate_hz
            self._manual_publish_timer = self.create_timer(
                period_s, self._manual_publish_timer_callback
            )
            self.get_logger().info(
                f"Manual mode ON — publishing to "
                f"{self.get_parameter('manual_joint_state_topic').value} "
                f"at {self._manual_publish_rate_hz:.1f} Hz; "
                f"drag joint sliders, press P to publish pose"
            )
        else:
            if self._manual_publish_timer is not None:
                self.destroy_timer(self._manual_publish_timer)
                self._manual_publish_timer = None
            self.get_logger().info("Manual mode OFF — tracking /joint_states")
        return self._manual_mode

    def _mode_service_callback(self, request, response):
        """Service callback: set manual mode via ROS (for replay閉環)."""
        desired = request.data  # True = manual, False = ROS tracking
        if desired and not self._manual_mode:
            self.toggle_manual_mode()
            response.success = True
            response.message = "Switched to manual mode"
        elif not desired and self._manual_mode:
            self.toggle_manual_mode()
            response.success = True
            response.message = "Switched to ROS tracking mode"
        else:
            response.success = True
            response.message = f"Already in {'manual' if desired else 'ROS tracking'} mode"
        return response

    def request_transition_plan(self) -> None:
        """Send an async planning request to /plan_transition_once.

        Called from the T key handler.  Non-blocking — the result is polled
        each frame in ``process_transition_result()``.
        """
        if not self._manual_mode:
            self._transition_status = "Enter manual mode before planning"
            self.get_logger().warning(self._transition_status)
            return

        if self._transition_future is not None and not self._transition_future.done():
            self._transition_status = "Planning already running"
            self.get_logger().warning(self._transition_status)
            return

        # Force-publish latest pose so the server reads fresh data.
        self.publish_current_qpos()

        if not self._transition_client.service_is_ready():
            self._transition_status = "Transition planning service unavailable"
            self.get_logger().error(self._transition_status)
            return

        from std_srvs.srv import Trigger
        request = Trigger.Request()
        self._transition_future = self._transition_client.call_async(request)
        self._transition_status = "TRANSITION_REQUEST_SENT"
        self.get_logger().info(self._transition_status)

    def process_transition_result(self) -> None:
        """Check async planning future and update status overlay.

        Called each frame from the main loop (inside viewer.lock).
        """
        if self._transition_future is None:
            return
        if not self._transition_future.done():
            return

        result = self._transition_future.result()
        self._transition_future = None
        if result is None:
            self._transition_status = "Planning failed: no response"
            self.get_logger().error(self._transition_status)
            return

        if result.success:
            # Parse error_code from the pipe-delimited message.
            msg = result.message
            parts = dict(p.split("=", 1) for p in msg.split("|") if "=" in p)
            code = parts.get("error_code", "UNKNOWN")
            pts = parts.get("trajectory_points", "?")
            t = parts.get("planning_time", "?")
            self._transition_status = f"{code} ({pts} pts, {t}s)"
            self.get_logger().info(
                f"TRANSITION_PLANNED: code={code}, points={pts}, time={t}s"
            )
        else:
            msg = result.message
            code = msg.split("|")[0].split("=", 1)[-1] if "|" in msg else msg
            self._transition_status = f"Planning failed: {code}"
            self.get_logger().error(f"TRANSITION_FAILED: {msg}")

    def publish_current_qpos(self) -> None:
        """Read the current MuJoCo qpos and force-publish it immediately.

        Call this from the main thread (inside viewer.lock) to snapshot the
        user's manually-adjusted pose so that ``plan_transition`` /
        ``use_current_state:=true`` can pick it up.  The continuous timer
        publishes the same data at the configured rate; this method forces one
        immediate publish with log output.
        """
        self._publish_manual_joint_state(log_values=True)

    def _publish_manual_joint_state(self, *, log_values: bool = False) -> None:
        """Publish current MuJoCo qpos to the manual joint state topic."""
        values: list[float] = []
        for name in self._joint_names:
            adr = self._qpos_addresses[name]
            values.append(float(self._data.qpos[adr]))
        if not all(isfinite(v) for v in values):
            self.get_logger().warning(
                "Manual joint state contains non-finite values; skipping publish"
            )
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        msg.position = values
        self._manual_pose_pub.publish(msg)
        self._latest_positions = dict(zip(self._joint_names, values))
        self._received_joint_state = True
        if log_values:
            self.get_logger().info(
                f"Published manual pose: {[f'{v:.3f}' for v in values]}"
            )

    def _manual_publish_timer_callback(self) -> None:
        """Timer callback: continuously publish current MuJoCo qpos."""
        if not self._manual_mode:
            return
        self._publish_manual_joint_state(log_values=False)

    # ------------------------------------------------------------------
    #  Live dynamic obstacle slots (/collision_object → MuJoCo)
    # ------------------------------------------------------------------

    def _setup_obstacle_slots(self, counts: tuple[int, int, int]) -> None:
        """Resolve the pre-allocated free-body slots and initialise bookkeeping."""
        self._obstacle_slots: list[dict] = []
        self._slot_by_object_id: dict[str, int] = {}
        self._obstacle_targets: dict[int, dict] = {}
        for shape, count in zip(("sphere", "box", "cylinder"), counts):
            for index in range(count):
                slot_name = f"dyn_slot_{shape}_{index}"
                joint_id = mujoco.mj_name2id(
                    self._model, mujoco.mjtObj.mjOBJ_JOINT, f"{slot_name}_joint"
                )
                geom_id = mujoco.mj_name2id(
                    self._model, mujoco.mjtObj.mjOBJ_GEOM, f"{slot_name}_geom"
                )
                if joint_id < 0 or geom_id < 0:
                    raise ValueError(
                        f"MuJoCo model is missing dynamic obstacle slot {slot_name}"
                    )
                default_size = _DEFAULT_SLOT_SIZES[shape]
                self._obstacle_slots.append(
                    {
                        "name": slot_name,
                        "joint_id": joint_id,
                        "geom_id": geom_id,
                        "qpos_adr": int(self._model.jnt_qposadr[joint_id]),
                        "shape": shape,
                        "claimed": False,
                        "object_id": None,
                        "size": default_size,
                    }
                )
        # The last arm joint comes before the first free joint in qpos.
        # We need this boundary so manual-mode mj_step can save/restore
        # obstacle positions while letting arm actuators respond to sliders.
        if self._obstacle_slots:
            self._arm_qpos_count = min(
                slot["qpos_adr"] for slot in self._obstacle_slots
            )
        else:
            self._arm_qpos_count = self._model.nq
        self.get_logger().info(
            f"Pre-allocated {len(self._obstacle_slots)} dynamic obstacle slot(s) "
            "for /collision_object rendering."
        )

    def _collision_object_callback(self, message: CollisionObject) -> None:
        """Track ADD/APPEND/MOVE/REMOVE of collision objects.

        This callback only mutates plain-dict bookkeeping (``_obstacle_targets``);
        it never touches ``self._model`` / ``self._data`` because it runs outside
        the viewer lock.  ``apply_latest_obstacles`` writes the pending targets
        into MuJoCo under ``viewer.lock()`` each frame.

        Issue #11: strict input validation — illegal messages are rejected.
        """
        # Validate common fields first.
        if not message.id:
            self.get_logger().error("CollisionObject rejected: empty ID")
            return
        if not message.header.frame_id:
            self.get_logger().error(
                f"CollisionObject {message.id} rejected: empty frame_id"
            )
            return

        # message.operation is uint8; rclpy may deserialize as int (0) or
        # bytes (b'\\x00') depending on the subscriber context.  Normalise to
        # an int for robust comparison.
        op_raw = message.operation
        if isinstance(op_raw, bytes):
            op_val = int.from_bytes(op_raw, 'little')
        else:
            op_val = int(op_raw)
        object_id = message.id

        if op_val == 0 or op_val == 2:  # ADD or APPEND
            if not message.primitives:
                self.get_logger().warning(
                    f"ADD/APPEND for {object_id} has no primitives; ignored"
                )
                return
            # Validate pose.
            if not self._validate_pose(message, object_id):
                return
            # Validate primitive.
            if not self._validate_primitive(message.primitives[0], object_id):
                return
            # SolidPrimitive.type is uint8 — may arrive as int or bytes.
            prim_type = message.primitives[0].type
            if isinstance(prim_type, bytes):
                prim_type = int.from_bytes(prim_type, 'little')
            else:
                prim_type = int(prim_type)
            shape_name = _SLOT_SHAPE_BY_PRIMITIVE.get(prim_type)
            if shape_name is None:
                self.get_logger().warning(
                    f"Ignoring {object_id}: unsupported primitive type {prim_type}"
                )
                return
            self.get_logger().info(
                f"ADD {object_id} shape={shape_name} -> claiming slot"
            )
            slot_idx = self._slot_by_object_id.get(object_id)
            if slot_idx is not None:
                existing_slot = self._obstacle_slots[slot_idx]
                if existing_slot["shape"] != shape_name:
                    # Issue #11.3: Same ID, different shape — release old, claim new.
                    self.get_logger().info(
                        f"Object {object_id} changed shape: "
                        f"{existing_slot['shape']} -> {shape_name}"
                    )
                    self._release_slot(slot_idx, object_id)
                    slot_idx = None
            if slot_idx is None:
                slot_idx = self._claim_slot(shape_name, object_id)
                if slot_idx is None:
                    self.get_logger().warning(
                        f"No free {shape_name} obstacle slot for {object_id}; dropped"
                    )
                    return
            self._set_slot_target(slot_idx, message)

        elif op_val == 3:  # MOVE
            slot_idx = self._slot_by_object_id.get(object_id)
            if slot_idx is None:
                self.get_logger().warning(
                    f"MOVE for unknown object {object_id} ignored"
                )
                return
            # Validate pose for MOVE.
            if not self._validate_pose(message, object_id):
                return
            # If MOVE includes a primitive, validate it too.
            if message.primitives:
                if not self._validate_primitive(message.primitives[0], object_id):
                    return
            self._set_slot_target(slot_idx, message)

        elif op_val == 1:  # REMOVE
            slot_idx = self._slot_by_object_id.get(object_id)
            if slot_idx is None:
                return
            self._release_slot(slot_idx, object_id)

    def _validate_pose(self, msg: CollisionObject, object_id: str) -> bool:
        """Issue #11: validate position and quaternion are finite and normalised."""
        px, py, pz = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        if not (isfinite(px) and isfinite(py) and isfinite(pz)):
            self.get_logger().error(
                f"CollisionObject {object_id} rejected: non-finite position"
            )
            return False
        qx, qy, qz, qw = (
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w,
        )
        if not (isfinite(qx) and isfinite(qy) and isfinite(qz) and isfinite(qw)):
            self.get_logger().error(
                f"CollisionObject {object_id} rejected: non-finite quaternion"
            )
            return False
        norm = sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 1e-12:
            self.get_logger().error(
                f"CollisionObject {object_id} rejected: zero-norm quaternion"
            )
            return False
        # Normalise in-place for downstream use.
        inv = 1.0 / norm
        msg.pose.orientation.x = qx * inv
        msg.pose.orientation.y = qy * inv
        msg.pose.orientation.z = qz * inv
        msg.pose.orientation.w = qw * inv
        return True

    def _validate_primitive(self, prim, object_id: str) -> bool:
        """Issue #11: validate primitive type and dimensions."""
        EXPECTED_DIMS = {SolidPrimitive.BOX: 3, SolidPrimitive.SPHERE: 1, SolidPrimitive.CYLINDER: 2}
        prim_type = prim.type
        if isinstance(prim_type, bytes):
            prim_type = int.from_bytes(prim_type, 'little')
        expected = EXPECTED_DIMS.get(int(prim_type))
        if expected is None:
            self.get_logger().error(
                f"CollisionObject {object_id} rejected: unsupported primitive type"
            )
            return False
        if len(prim.dimensions) != expected:
            self.get_logger().error(
                f"CollisionObject {object_id} rejected: expected {expected} "
                f"dimensions, got {len(prim.dimensions)}"
            )
            return False
        for d in prim.dimensions:
            if not isfinite(d) or d <= 0.0:
                self.get_logger().error(
                    f"CollisionObject {object_id} rejected: non-finite or "
                    f"non-positive dimension {d}"
                )
                return False
        return True

    def _claim_slot(self, shape_name: str, object_id: str) -> int | None:
        for index, slot in enumerate(self._obstacle_slots):
            if not slot["claimed"] and slot["shape"] == shape_name:
                slot["claimed"] = True
                slot["object_id"] = object_id
                self._slot_by_object_id[object_id] = index
                return index
        return None

    def _release_slot(self, slot_idx: int, object_id: str) -> None:
        """Release a slot and hide its geom (used on REMOVE or shape change)."""
        slot = self._obstacle_slots[slot_idx]
        slot["claimed"] = False
        slot["object_id"] = None
        slot["size"] = _DEFAULT_SLOT_SIZES[slot["shape"]]
        self._slot_by_object_id.pop(object_id, None)
        self._obstacle_targets[slot_idx] = {"visible": False}

    @staticmethod
    def _yup_to_zup_world(
        position: tuple[float, float, float],
        quat_wxyz: tuple[float, float, float, float],
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        """Rotate a base_link pose into world coords for a free-joint slot.

        Free-joint slots live at worldbody level (MuJoCo restriction), so the
        Y-up→Z-up rotation that ``display_frame`` applies to static annotations
        must be applied here manually: position by R, quaternion by composition
        with the rotation quaternion.
        """
        x, y, z = position
        rotated_position = (x, -z, y)
        rw, rx, ry, rz = _Y_UP_TO_Z_UP_QUAT_WXYZ
        qw, qx, qy, qz = quat_wxyz
        # Hamilton product R ⊗ q
        w = rw * qw - rx * qx - ry * qy - rz * qz
        i = rw * qx + rx * qw + ry * qz - rz * qy
        j = rw * qy - rx * qz + ry * qw + rz * qx
        k = rw * qz + rx * qy - ry * qx + rz * qw
        return rotated_position, (w, i, j, k)

    @staticmethod
    def _clamp_min_size(
        size: tuple[float, float, float], shape: str
    ) -> tuple[float, float, float]:
        """Ensure minimum positive dimensions to avoid MuJoCo errors."""
        MIN_DIM = 1e-6
        if shape == "sphere":
            return (max(size[0], MIN_DIM), 0.0, 0.0)
        elif shape == "box":
            return (
                max(size[0], MIN_DIM),
                max(size[1], MIN_DIM),
                max(size[2], MIN_DIM),
            )
        elif shape == "cylinder":
            return (max(size[0], MIN_DIM), max(size[1], MIN_DIM), 0.0)
        return size

    def _set_slot_target(self, slot_idx: int, message: CollisionObject) -> None:
        slot = self._obstacle_slots[slot_idx]
        position = (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        )
        # ROS xyzw → MuJoCo wxyz.
        orientation = message.pose.orientation
        quat_wxyz = (
            orientation.w,
            orientation.x,
            orientation.y,
            orientation.z,
        )
        # Default to slot's saved size (preserved across MOVE without primitive).
        size = slot.get("size", _DEFAULT_SLOT_SIZES[slot["shape"]])
        if message.primitives:
            primitive = message.primitives[0]
            if slot["shape"] == "sphere" and len(primitive.dimensions) >= 1:
                size = (float(primitive.dimensions[0]), 0.0, 0.0)
            elif slot["shape"] == "box" and len(primitive.dimensions) >= 3:
                size = (
                    float(primitive.dimensions[0]) / 2.0,
                    float(primitive.dimensions[1]) / 2.0,
                    float(primitive.dimensions[2]) / 2.0,
                )
            elif slot["shape"] == "cylinder" and len(primitive.dimensions) >= 2:
                # SolidPrimitive cylinder dimensions = (height, radius).
                size = (
                    float(primitive.dimensions[1]),
                    float(primitive.dimensions[0]) / 2.0,
                    0.0,
                )
        # Clamp to minimum size and persist in slot for MOVE-without-primitive.
        size = MuJoCoJointStateViewer._clamp_min_size(size, slot["shape"])
        slot["size"] = size
        if message.header.frame_id not in ("", "base_link"):
            self.get_logger().error(
                f"Obstacle {message.id} rejected: unsupported frame "
                f"'{message.header.frame_id}' (only base_link is supported)"
            )
            return
        world_position, world_quat = MuJoCoJointStateViewer._yup_to_zup_world(
            position, quat_wxyz
        )
        self._obstacle_targets[slot_idx] = {
            "position": world_position,
            "quat_wxyz": world_quat,
            "size": size,
            "visible": True,
        }

    def _write_obstacle_targets(self) -> bool:
        """Copy pending ROS obstacle targets into MuJoCo qpos, geom_size, and
        geom_rgba (no physics step).

        Returns True if any targets were written.
        """
        if not self._obstacle_targets:
            return False
        for slot_idx, target in self._obstacle_targets.items():
            slot = self._obstacle_slots[slot_idx]
            geom_id = slot["geom_id"]
            adr = slot["qpos_adr"]
            if target.get("visible", True):
                # Write free-joint qpos (x, y, z, qw, qx, qy, qz).
                self._data.qpos[adr : adr + 7] = [
                    target["position"][0],
                    target["position"][1],
                    target["position"][2],
                    target["quat_wxyz"][0],
                    target["quat_wxyz"][1],
                    target["quat_wxyz"][2],
                    target["quat_wxyz"][3],
                ]
                # Write geom_size (full array; unused elements ignored by MuJoCo).
                size = target.get("size")
                if size is not None:
                    if slot["shape"] == "sphere":
                        self._model.geom_size[geom_id, 0] = size[0]
                    elif slot["shape"] == "box":
                        self._model.geom_size[geom_id, :3] = size[:3]
                    elif slot["shape"] == "cylinder":
                        self._model.geom_size[geom_id, :2] = size[:2]
                # Write geom_rgba with configured dynamic obstacle color.
                self._model.geom_rgba[geom_id] = self._dynamic_color
            else:
                # REMOVE: park below floor, set alpha to 0.
                self._data.qpos[adr : adr + 7] = [
                    0.0, 0.0, -5.0, 1.0, 0.0, 0.0, 0.0
                ]
                self._model.geom_rgba[geom_id, 3] = 0.0
        self._obstacle_targets.clear()
        return True

    def apply_latest_obstacles(self) -> None:
        """Write ROS obstacle targets and (in ROS mode) resolve contacts.

        In manual mode only obstacle targets are written and ``mj_forward``
        is called — no ``mj_step``, because keyboard joint control directly
        modifies ``qpos`` and we don't want position actuators to fight it.
        """
        had_targets = self._write_obstacle_targets()
        arm_count = self._arm_qpos_count

        if self._manual_mode:
            mujoco.mj_forward(self._model, self._data)
        elif had_targets:
            saved_arm_qpos = self._data.qpos[:arm_count].copy()
            saved_arm_qvel = self._data.qvel[:arm_count].copy()
            mujoco.mj_step(self._model, self._data)
            self._data.qpos[:arm_count] = saved_arm_qpos
            self._data.qvel[:arm_count] = saved_arm_qvel
            mujoco.mj_forward(self._model, self._data)

    def _declare_parameters(self) -> None:
        self.declare_parameter("joint_names", list(DEFAULT_JOINT_NAMES))
        self.declare_parameter("joint_state_topic", "/mujoco_joint_states")
        self.declare_parameter("viewer_rate_hz", 60.0)
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("mesh_directory", "")
        self.declare_parameter("show_target_path", True)
        self.declare_parameter("trajectory_mat", "")
        self.declare_parameter("trajectory_offset_m", [0.0, 0.343, 1.587])
        self.declare_parameter("path_max_points", 300)
        # The trajectory data uses the project URDF/MoveIt Y-up coordinates.
        # Its fitted cylinder axis is therefore +Y even though MuJoCo displays
        # that direction vertically after applying Y_UP_TO_Z_UP_EULER.
        self.declare_parameter("show_tracking_cylinder", True)
        self.declare_parameter("tracking_cylinder_axis_direction", [0.0, 1.0, 0.0])
        self.declare_parameter("tracking_cylinder_height_margin_m", 0.04)
        self.declare_parameter("project_display_path_to_cylinder", True)
        # When True the tracking-cylinder bottom extends to the origin plane
        # (Y=0 in base_link, i.e. the floor) so the cylinder visually stands
        # on the ground instead of hovering at the trajectory's lowest extent.
        self.declare_parameter("tracking_cylinder_extend_to_ground", True)
        # Lift the red display line very slightly above the transparent surface
        # to avoid z-fighting. This changes visualization only, not IK input.
        self.declare_parameter("path_surface_offset_m", 0.002)
        self.declare_parameter("show_obstacles", True)
        self.declare_parameter("obstacles_file", "")
        self.declare_parameter(
            "static_collision_object_topic", STATIC_COLLISION_OBJECT_TOPIC
        )
        # Live dynamic obstacles (from /collision_object) rendered via a
        # pre-allocated free-body slot pool.  Purely additive: static
        # obstacles.yaml objects are unaffected.
        self.declare_parameter("show_dynamic_obstacles", True)
        self.declare_parameter(
            "collision_object_topic", DYNAMIC_COLLISION_OBJECT_TOPIC
        )
        self.declare_parameter("dynamic_obstacle_color", [1.0, 0.5, 0.1, 1.0])
        self.declare_parameter("obstacle_slot_counts", list(DEFAULT_OBSTACLE_SLOT_COUNTS))
        self.declare_parameter("manual_joint_state_publish_rate_hz", 20.0)
        self.declare_parameter("manual_joint_state_topic", "/mujoco_joint_states")
        # Issue #5: t_key_execute_transition / t_key_replay_transition removed.
        # Planning behaviour is controlled by transition_result_mode on the server.

    def _joint_state_callback(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        if not all(name in positions for name in self._joint_names):
            return
        values = {name: float(positions[name]) for name in self._joint_names}
        if not all(isfinite(value) for value in values.values()):
            self.get_logger().warning("Ignoring /joint_states message with non-finite positions")
            return
        self._latest_positions = values
        self._received_joint_state = True

    def _joint_qpos_addresses(self, joint_names: Sequence[str]) -> dict[str, int]:
        addresses: dict[str, int] = {}
        for name in joint_names:
            joint_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise ValueError(
                    f"Project MuJoCo model has no joint named {name}; "
                    "check joint_names and the project URDF"
                )
            addresses[name] = int(self._model.jnt_qposadr[joint_id])
        return addresses

    def _read_joint_limits(
        self, joint_names: Sequence[str]
    ) -> dict[str, tuple[float, float]]:
        """Read per-joint position limits from the MuJoCo model ``jnt_range``.

        Only joints that are BOTH scalar (hinge/slide) AND explicitly limited
        (``jnt_limited[joint_id]`` is True) get finite bounds.  Unlimited
        joints (e.g. continuous hinges) correctly receive (-inf, +inf) so they
        are never wrongly clamped to zero.
        """
        limits: dict[str, tuple[float, float]] = {}
        for name in joint_names:
            joint_id = mujoco.mj_name2id(
                self._model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            if joint_id < 0:
                raise ValueError(f"Unknown joint in MuJoCo model: {name}")

            joint_type = self._model.jnt_type[joint_id]
            is_scalar = joint_type in (
                mujoco.mjtJoint.mjJNT_HINGE,
                mujoco.mjtJoint.mjJNT_SLIDE,
            )
            is_limited = bool(self._model.jnt_limited[joint_id])

            if is_scalar and is_limited:
                low = float(self._model.jnt_range[joint_id, 0])
                high = float(self._model.jnt_range[joint_id, 1])
                if not isfinite(low) or not isfinite(high):
                    raise ValueError(
                        f"Joint {name} has non-finite limits [{low}, {high}]"
                    )
                if low > high:
                    raise ValueError(
                        f"Joint {name} has invalid range [{low}, {high}]"
                    )
                limits[name] = (low, high)
            else:
                limits[name] = (-float("inf"), float("inf"))
        return limits

    def _clamp_joint(self, name: str, value: float) -> float:
        """Clamp a joint value to its MuJoCo/URDF limits.

        Logs a message when clamping occurs; never crashes.
        """
        low, high = self._joint_limits.get(name, (-float("inf"), float("inf")))
        clamped = max(low, min(high, value))
        if not isfinite(clamped):
            self.get_logger().warning(
                f"Joint {name}: value {value} clamped to non-finite; using 0.0"
            )
            return 0.0
        if clamped != value:
            self.get_logger().info(
                f"Joint {name}: {value:.3f} clamped to [{low:.3f}, {high:.3f}]"
            )
        return clamped

    @staticmethod
    def _sample_display_path(
        points: Sequence[Sequence[float]],
        maximum_points: int,
    ) -> list[tuple[float, float, float]]:
        """Sub-sample a full trajectory for display (capsule budget)."""
        if maximum_points < 2:
            raise ValueError("path_max_points must be at least two")
        values = list(points)
        if len(values) <= maximum_points:
            return [
                (float(x), float(y), float(z)) for x, y, z in values
            ]
        last = len(values) - 1
        indices = [
            round(index * last / (maximum_points - 1))
            for index in range(maximum_points)
        ]
        return [
            (float(values[index][0]), float(values[index][1]), float(values[index][2]))
            for index in indices
        ]

    @staticmethod
    def _urdf_to_mjcf(urdf_path: Path, mesh_directory: Path) -> str:
        """Let MuJoCo import the authoritative project URDF and STL meshes."""

        urdf_xml = urdf_path.read_text(encoding="utf-8")
        urdf_xml = urdf_xml.replace(
            "package://ninezzhou/meshes/", f"{mesh_directory}/"
        )
        with TemporaryDirectory(prefix="robot_safecontrol_mujoco_") as temporary_directory:
            temporary_root = Path(temporary_directory)
            temporary_urdf = temporary_root / "ninezzhou.urdf"
            temporary_mjcf = temporary_root / "ninezzhou_converted.xml"
            temporary_urdf.write_text(urdf_xml, encoding="utf-8")
            model = mujoco.MjModel.from_xml_path(str(temporary_urdf))
            mujoco.mj_saveLastXML(str(temporary_mjcf), model)
            return temporary_mjcf.read_text(encoding="utf-8")

    @staticmethod
    def _inject_display_scene(
        mjcf_xml: str,
        target_path: Iterable[tuple[float, float, float]],
        obstacles: Iterable[CollisionObjectSpec],
        tracking_cylinder: TrackingCylinderSpec | None = None,
        obstacle_slot_counts: tuple[int, int, int] = DEFAULT_OBSTACLE_SLOT_COUNTS,
        joint_names: tuple[str, ...] = (),
    ) -> str:
        """Add display-only floor, lighting, path, obstacles, frame axes,
        cylinder surface normals, and position actuators for every joint so the
        right-side joint-slider panel is populated."""

        if "<worldbody>" not in mjcf_xml or "</worldbody>" not in mjcf_xml:
            raise ValueError("MuJoCo conversion did not produce a worldbody")

        # Inject position actuators (before <worldbody>) so the right-side
        # joint-slider panel is populated and the user can manually pose the arm.
        actuator_xml = MuJoCoJointStateViewer._actuator_xml(joint_names)
        if actuator_xml:
            mjcf_xml = mjcf_xml.replace("<worldbody>", actuator_xml + "\n<worldbody>", 1)

        world_prefix = f"""
    <geom name=\"floor\" type=\"plane\" size=\"3 3 0.1\" pos=\"0 0 -0.058\"
          rgba=\"0.22 0.22 0.22 1\" contype=\"0\" conaffinity=\"0\"/>
    <light pos=\"0 0 4\" dir=\"0 0 -1\" diffuse=\"0.8 0.8 0.8\" specular=\"0.3 0.3 0.3\"/>
    <light pos=\"2 2 3\" dir=\"-2 -2 -3\" diffuse=\"0.45 0.45 0.45\"/>
    <body name=\"display_frame\" euler=\"{Y_UP_TO_Z_UP_EULER}\">
"""
        annotations = MuJoCoJointStateViewer._tracking_cylinder_geom(
            tracking_cylinder
        )
        annotations += MuJoCoJointStateViewer._path_geoms(target_path)
        annotations += MuJoCoJointStateViewer._obstacle_geoms(obstacles)
        result = mjcf_xml.replace("<worldbody>", "<worldbody>" + world_prefix, 1)
        result = result.replace("</worldbody>", annotations + "    </body>\n  </worldbody>", 1)
        # Free joints can only live at the top level (children of worldbody), so
        # the dynamic-obstacle slots are inserted as siblings of display_frame.
        # apply_latest_obstacles applies the same Y-up->Z-up rotation manually.
        slot_geoms = MuJoCoJointStateViewer._obstacle_slot_geoms(obstacle_slot_counts)
        if slot_geoms:
            result = result.replace("</worldbody>", slot_geoms + "  </worldbody>", 1)

        # Inject coordinate-frame axis geoms into base_link and tool0.
        result = MuJoCoJointStateViewer._inject_frame_axes(result)

        # Inject surface-normal indicators along the cylinder surface.
        if tracking_cylinder is not None:
            points = list(target_path)
            geoms = MuJoCoJointStateViewer._surface_normal_geoms(
                points, tracking_cylinder
            )
            if geoms:
                # Insert before the last </body> that precedes </worldbody>.
                wb_pos = result.rfind("</worldbody>")
                body_pos = result.rfind("</body>", 0, wb_pos)
                if body_pos != -1:
                    result = result[:body_pos] + geoms + result[body_pos:]

        return result

    @staticmethod
    def _fit_tracking_cylinder(
        points: Sequence[tuple[float, float, float]],
        axis_direction: Sequence[float],
        height_margin_m: float,
        *,
        extend_to_ground: bool = False,
    ) -> TrackingCylinderSpec:
        """Fit a circle in the plane normal to a configured cylinder axis.

        When *extend_to_ground* is True the cylinder bottom is extended to the
        origin plane (Y=0 in base_link coordinates, i.e. the floor) so the
        cylinder appears to stand on the ground.
        """

        if len(points) < 3:
            raise ValueError("At least three path points are required to fit a cylinder")
        if height_margin_m < 0.0:
            raise ValueError("tracking_cylinder_height_margin_m must be non-negative")

        values = np.asarray(points, dtype=float)
        axis = np.asarray(axis_direction, dtype=float)
        axis_length = float(np.linalg.norm(axis))
        if axis.shape != (3,) or axis_length < 1e-12:
            raise ValueError("tracking_cylinder_axis_direction must be a non-zero 3-vector")
        axis /= axis_length

        # Build an orthonormal basis (u, v) for the plane perpendicular to axis.
        helper = np.array([1.0, 0.0, 0.0])
        if abs(float(np.dot(helper, axis))) > 0.9:
            helper = np.array([0.0, 0.0, 1.0])
        u = np.cross(axis, helper)
        u /= np.linalg.norm(u)
        v = np.cross(axis, u)
        v /= np.linalg.norm(v)

        plane_x = values @ u
        plane_y = values @ v
        matrix = np.column_stack((plane_x, plane_y, np.ones(len(values))))
        target = -(plane_x * plane_x + plane_y * plane_y)
        coefficient, *_ = np.linalg.lstsq(matrix, target, rcond=None)
        d_value, e_value, f_value = coefficient
        center_x = -0.5 * d_value
        center_y = -0.5 * e_value
        radius_squared = center_x * center_x + center_y * center_y - f_value
        if radius_squared <= 0.0:
            raise ValueError("Cylinder fit produced a non-positive radius")
        radius = sqrt(float(radius_squared))

        axial_values = values @ axis
        axial_top = float(axial_values.max()) + height_margin_m
        if extend_to_ground:
            # The floor plane is at z=-0.058 in world coords, which
            # corresponds to Y=-0.058 in base_link (Y-up).  The origin
            # projects to 0 for any axis direction, so the floor's
            # projection equals axis[1] * (-0.058) for axis=[0,1,0].
            axial_bottom = float(np.dot(np.array([0.0, -0.058, 0.0]), axis))
            height = axial_top - axial_bottom
            axial_center = 0.5 * (axial_bottom + axial_top)
        else:
            axial_bottom = float(axial_values.min()) - height_margin_m
            height = axial_top - axial_bottom
            axial_center = 0.5 * (axial_bottom + axial_top)
        center = center_x * u + center_y * v + axial_center * axis

        radial_distance = np.sqrt(
            (plane_x - center_x) ** 2 + (plane_y - center_y) ** 2
        )
        radial_error = radial_distance - radius
        rms_error = float(np.sqrt(np.mean(radial_error * radial_error)))
        max_error = float(np.max(np.abs(radial_error)))

        return TrackingCylinderSpec(
            center=tuple(float(value) for value in center),
            axis_direction=tuple(float(value) for value in axis),
            radius=radius,
            height=height,
            radial_rms_error=rms_error,
            radial_max_error=max_error,
        )

    @staticmethod
    def _project_path_to_cylinder(
        points: Sequence[tuple[float, float, float]],
        cylinder: TrackingCylinderSpec,
        surface_offset_m: float,
    ) -> list[tuple[float, float, float]]:
        """Project display points radially onto the fitted cylinder surface."""

        if surface_offset_m < 0.0:
            raise ValueError("path_surface_offset_m must be non-negative")
        center = np.asarray(cylinder.center, dtype=float)
        axis = np.asarray(cylinder.axis_direction, dtype=float)
        display_radius = cylinder.radius + surface_offset_m
        result: list[tuple[float, float, float]] = []

        for raw_point in points:
            point = np.asarray(raw_point, dtype=float)
            relative = point - center
            axial = axis * float(np.dot(relative, axis))
            radial = relative - axial
            radial_length = float(np.linalg.norm(radial))
            if radial_length < 1e-12:
                raise ValueError("A trajectory point lies on the fitted cylinder axis")
            projected = center + axial + radial * (display_radius / radial_length)
            result.append(tuple(float(value) for value in projected))
        return result

    # ------------------------------------------------------------------
    #  Coordinate-frame axes (display-only, group 2)
    #  Red = X, Green = Y, Blue = Z.  Each frame is a child body with
    #  three capsule geoms.
    # ------------------------------------------------------------------

    _FRAME_AXIS_LENGTH = 0.06  # m
    _FRAME_AXIS_RADIUS = 0.0025  # m
    _FRAME_AXIS_ALPHA = 0.85

    @staticmethod
    def _frame_axis_geoms(
        frame_name: str,
        axis_length: float | None = None,
        axis_radius: float | None = None,
        alpha: float | None = None,
        *,
        raw: bool = False,
    ) -> str:
        """Return MuJoCo XML for three axis capsule geoms.

        When *raw* is False the geoms are wrapped in a child ``body`` element
        with identity transform.  When *raw* is True only the ``geom`` lines
        are returned (indented for nesting inside a caller-provided body).
        """

        length = axis_length if axis_length is not None else MuJoCoJointStateViewer._FRAME_AXIS_LENGTH
        radius = axis_radius if axis_radius is not None else MuJoCoJointStateViewer._FRAME_AXIS_RADIUS
        a = alpha if alpha is not None else MuJoCoJointStateViewer._FRAME_AXIS_ALPHA

        indent = "    " if raw else "          "
        geoms = (
            f'{indent}<geom name="{frame_name}_axis_x" type="capsule" '
            f'fromto="0 0 0 {length:.3f} 0 0" size="{radius:.4f}" '
            f'rgba="1 0.08 0.08 {a:.2f}" contype="0" conaffinity="0" group="2"/>\n'
            f'{indent}<geom name="{frame_name}_axis_y" type="capsule" '
            f'fromto="0 0 0 0 {length:.3f} 0" size="{radius:.4f}" '
            f'rgba="0.08 1 0.08 {a:.2f}" contype="0" conaffinity="0" group="2"/>\n'
            f'{indent}<geom name="{frame_name}_axis_z" type="capsule" '
            f'fromto="0 0 0 0 0 {length:.3f}" size="{radius:.4f}" '
            f'rgba="0.08 0.25 1 {a:.2f}" contype="0" conaffinity="0" group="2"/>\n'
        )
        if raw:
            return geoms
        return (
            f'        <body name="{frame_name}_frame_axes" pos="0 0 0" quat="1 0 0 0">\n'
            f'{geoms}'
            f'        </body>\n'
        )

    @staticmethod
    def _inject_frame_axes(mjcf_xml: str) -> str:
        """Add axis geoms for the base (world origin) and tool0 (Link9+offset)
        coordinate frames."""

        # Base frame: insert a body right after the display_frame opening tag.
        # display_frame wraps the robot for Y-up → Z-up conversion, so its
        # origin is the world origin in URDF coordinates.
        base_viz = MuJoCoJointStateViewer._frame_axis_geoms(
            "base_link",
            MuJoCoJointStateViewer._FRAME_AXIS_LENGTH * 1.5,
            MuJoCoJointStateViewer._FRAME_AXIS_RADIUS * 1.2,
        )
        # Find the display_frame opening tag and insert base frame after it.
        display_pattern = re.compile(
            r'(<body\s[^>]*\bname="display_frame"[^>]*>)'
        )
        mjcf_xml = display_pattern.sub(
            lambda m: m.group(0) + "\n" + base_viz, mjcf_xml, count=1
        )

        # Tool0 frame: insert as a child of Link9 at position (0.235, 0, 0).
        # This matches the URDF tool0_fixed joint origin xyz="0.235 0 0".
        tool0_viz = (
            f'        <body name="tool0_frame_axes" pos="0.235 0 0" quat="1 0 0 0">\n'
        )
        tool0_viz += MuJoCoJointStateViewer._frame_axis_geoms(
            "tool0",
            MuJoCoJointStateViewer._FRAME_AXIS_LENGTH,
            MuJoCoJointStateViewer._FRAME_AXIS_RADIUS,
            raw=True,
        )
        tool0_viz += "        </body>\n"
        link9_pattern = re.compile(
            r'(<body\s[^>]*\bname="Link9"[^>]*>)'
        )
        mjcf_xml = link9_pattern.sub(
            lambda m: m.group(0) + "\n" + tool0_viz, mjcf_xml, count=1
        )

        return mjcf_xml

    @staticmethod
    def _surface_normal_geoms(
        points: Sequence[tuple[float, float, float]],
        cylinder: TrackingCylinderSpec,
        normal_length: float = 0.04,
        step: int = 12,
    ) -> str:
        """Add small capsule geoms showing the cylinder radial direction at
        sampled path points.  The line points *toward the cylinder axis*
        (inward radial), matching the tool0 X-axis orientation used by IK.

        ``step`` controls down-sampling so the display is not cluttered.
        """

        if cylinder is None or len(points) < 2:
            return ""
        center = np.asarray(cylinder.center, dtype=float)
        axis = np.asarray(cylinder.axis_direction, dtype=float)
        display_radius = cylinder.radius  # geometric cylinder surface

        geoms_parts: list[str] = []
        for idx in range(0, len(points), step):
            point = np.asarray(points[idx], dtype=float)
            rel = point - center
            axial = axis * float(np.dot(rel, axis))
            radial = rel - axial
            radial_len = float(np.linalg.norm(radial))
            if radial_len < 1e-12:
                continue
            inward = -radial / radial_len
            end = point + inward * normal_length
            geoms_parts.append(
                f'      <geom name="surface_normal_{idx}" type="capsule" '
                f'fromto="{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} '
                f'{end[0]:.6f} {end[1]:.6f} {end[2]:.6f}" '
                f'size="0.0018" rgba="0.15 0.85 0.15 0.85" '
                f'contype="0" conaffinity="0" group="2"/>\n'
            )
        return "".join(geoms_parts)

    @staticmethod
    def _tracking_cylinder_geom(cylinder: TrackingCylinderSpec | None) -> str:
        """Create a transparent display-only MuJoCo cylinder."""

        if cylinder is None:
            return ""
        axis = np.asarray(cylinder.axis_direction, dtype=float)
        source = np.array([0.0, 0.0, 1.0])
        dot_value = float(np.clip(np.dot(source, axis), -1.0, 1.0))
        if dot_value < -0.999999:
            quat_wxyz = (0.0, 1.0, 0.0, 0.0)
        else:
            cross_value = np.cross(source, axis)
            quat_xyzw = np.array(
                [cross_value[0], cross_value[1], cross_value[2], 1.0 + dot_value],
                dtype=float,
            )
            quat_xyzw /= np.linalg.norm(quat_xyzw)
            quat_wxyz = (
                float(quat_xyzw[3]),
                float(quat_xyzw[0]),
                float(quat_xyzw[1]),
                float(quat_xyzw[2]),
            )

        x, y, z = cylinder.center
        qw, qx, qy, qz = quat_wxyz
        return (
            "      <geom name=\"tracking_cylinder\" type=\"cylinder\" "
            f"pos=\"{x:.6f} {y:.6f} {z:.6f}\" "
            f"quat=\"{qw:.9f} {qx:.9f} {qy:.9f} {qz:.9f}\" "
            f"size=\"{cylinder.radius:.6f} {cylinder.height / 2.0:.6f}\" "
            "rgba=\"0.25 0.55 0.82 0.18\" "
            "contype=\"0\" conaffinity=\"0\" group=\"2\"/>\n"
        )

    @staticmethod
    def _path_geoms(points: Iterable[tuple[float, float, float]]) -> str:
        values = list(points)
        geoms = []
        for index, (first, second) in enumerate(zip(values, values[1:])):
            geoms.append(
                "      <geom name=\"target_path_%d\" type=\"capsule\" "
                "fromto=\"%.6f %.6f %.6f %.6f %.6f %.6f\" size=\"0.001\" "
                "rgba=\"0.95 0.1 0.1 0.9\" contype=\"0\" conaffinity=\"0\"/>\n"
                % (index, *first, *second)
            )
        return "".join(geoms)

    @staticmethod
    def _obstacle_geoms(obstacles: Iterable[CollisionObjectSpec]) -> str:
        geoms = []
        for obstacle in obstacles:
            if obstacle.shape == "box":
                geom_type = "box"
                size = tuple(value / 2.0 for value in obstacle.dimensions)
            elif obstacle.shape == "sphere":
                geom_type = "sphere"
                size = obstacle.dimensions
            elif obstacle.shape == "cylinder":
                geom_type = "cylinder"
                height, radius = obstacle.dimensions
                size = (radius, height / 2.0)
            else:
                raise ValueError(f"Unsupported MuJoCo obstacle shape: {obstacle.shape}")
            x, y, z = obstacle.position
            qx, qy, qz, qw = obstacle.quaternion_xyzw
            geoms.append(
                "      <geom name=\"%s\" type=\"%s\" pos=\"%.6f %.6f %.6f\" "
                "quat=\"%.6f %.6f %.6f %.6f\" size=\"%s\" "
                "rgba=\"0.2 0.65 0.95 0.55\" contype=\"0\" conaffinity=\"0\"/>\n"
                % (
                    escape(obstacle.object_id, quote=True),
                    geom_type,
                    x,
                    y,
                    z,
                    qw,
                    qx,
                    qy,
                    qz,
                    " ".join(f"{value:.6f}" for value in size),
                )
            )
        return "".join(geoms)

    @staticmethod
    def _actuator_xml(joint_names: tuple[str, ...]) -> str:
        """Return MJCF ``<actuator>`` section with a position servo per joint.

        Without actuators MuJoCo's right-side UI panel shows no joint sliders.
        A modest ``kp`` gives responsive drag behaviour without instability.
        """
        if not joint_names:
            return ""
        lines = ["  <actuator>"]
        for name in joint_names:
            lines.append(
                f'    <position name="act_{name}" joint="{name}" kp="60"/>'
            )
        lines.append("  </actuator>")
        return "\n".join(lines)

    @staticmethod
    def _obstacle_slot_geoms(counts: tuple[int, int, int]) -> str:
        """Pre-allocate free-body obstacle slots for live /collision_object updates.

        Each slot is a free body with a single display geom parked far below the
        floor (invisible, alpha=0) until claimed by a dynamic collision object.
        Runtime updates only write the slot's freejoint qpos and geom size/rgba,
        so the MuJoCo model never needs recompiling.
        """
        geoms: list[str] = []
        shapes = ("sphere", "box", "cylinder")
        for shape, count in zip(shapes, counts):
            size = _DEFAULT_SLOT_SIZES[shape]
            for index in range(count):
                slot_name = f"dyn_slot_{shape}_{index}"
                size_text = " ".join(f"{value:.6f}" for value in size)
                geoms.append(
                    f'      <body name="{slot_name}" pos="0 0 -5" quat="1 0 0 0">\n'
                    f'        <freejoint name="{slot_name}_joint"/>\n'
                    f'        <inertial pos="0 0 0" mass="0.01" '
                    f'diaginertia="1e-4 1e-4 1e-4"/>\n'
                    f'        <geom name="{slot_name}_geom" type="{shape}" '
                    f'size="{size_text}" rgba="1.0 0.5 0.1 1.0" '
                    f'contype="1" conaffinity="1" solimp="0.9 0.95 0.001" '
                    f'margin="0.001" gap="0.0"/>\n'
                    f"      </body>\n"
                )
        return "".join(geoms)

    @staticmethod
    def _share_file(package_name: str, relative_path: str) -> Path:
        try:
            return Path(get_package_share_directory(package_name)) / relative_path
        except PackageNotFoundError as error:
            raise FileNotFoundError(
                f"ROS package '{package_name}' is not available; source install/setup.bash first"
            ) from error

    def _default_urdf_path(self) -> Path:
        return self._share_file(
            "robot_safecontrol_moveit", "models/ninezzhou/urdf/ninezzhou.urdf"
        )

    def _default_mesh_directory(self) -> Path:
        return self._share_file(
            "robot_safecontrol_moveit", "models/ninezzhou/meshes"
        )

    def _default_trajectory_mat(self) -> Path:
        return self._share_file("robot_safecontrol_moveit", "data/nurbs/ik_input.mat")

    def _default_obstacles_file(self) -> Path:
        return self._share_file("robot_safecontrol_moveit", "config/obstacles.yaml")

    def _file_parameter(self, name: str, default: Path) -> Path:
        value = str(self.get_parameter(name).value).strip()
        path = Path(value).expanduser() if value else default
        if not path.is_file():
            raise FileNotFoundError(f"{name} does not exist: {path}")
        return path

    def _directory_parameter(self, name: str, default: Path) -> Path:
        value = str(self.get_parameter(name).value).strip()
        path = Path(value).expanduser() if value else default
        if not path.is_dir():
            raise NotADirectoryError(f"{name} does not exist: {path}")
        return path

    def _float_tuple(self, name: str, expected_length: int) -> tuple[float, ...]:
        values = tuple(float(value) for value in self.get_parameter(name).value)
        if len(values) != expected_length:
            raise ValueError(f"{name} must contain {expected_length} values")
        return values


def _make_key_callback(node: MuJoCoJointStateViewer):
    """Return a GLFW key callback for manual joint control and planning.

    Keyboard controls
    -----------------
    1-9          select joint (J1 … J9)
    Up / = / +   increase selected joint by 0.05 rad  (0.02 m for J1 prismatic)
    Down / - / _ decrease selected joint
    M            toggle Manual / ROS-tracking mode
    P            force-publish current MuJoCo qpos to manual_joint_state_topic
    T            request the closed-loop transition plan
    R            reset selected joint to zero (clamped to limits)
    Z            reset ALL joints to zero (clamped to limits)

    In manual mode the joint state is published continuously at
    ``manual_joint_state_publish_rate_hz`` (default 20 Hz).
    """

    from mujoco.glfw import glfw

    DELTA = 0.05       # rad for revolute joints
    DELTA_PRISMATIC = 0.02  # m for J1

    def _on_key(keycode: int) -> None:
        try:
            if glfw.KEY_1 <= keycode <= glfw.KEY_9:
                idx = keycode - glfw.KEY_1
                node.selected_joint = idx
                name = node._joint_names[idx]
                val = float(node._data.qpos[node._qpos_addresses[name]])
                node.get_logger().info(
                    f"Selected J{idx + 1} (current = {val:.3f})"
                )
                return

            if keycode in (glfw.KEY_M,):
                node.toggle_manual_mode()
                return

            if keycode in (glfw.KEY_P,):
                if node.manual_mode:
                    node.publish_current_qpos()
                    manual_topic = node.get_parameter(
                        "manual_joint_state_topic"
                    ).value
                    node.get_logger().info(
                        f"Pose published to {manual_topic}. "
                        "Press T to request the transition."
                    )
                else:
                    node.get_logger().info(
                        "Press M first to enter manual mode, then P to publish"
                    )
                return

            if keycode in (glfw.KEY_T,):
                if node.manual_mode:
                    node.request_transition_plan()
                else:
                    node._transition_status = (
                        "Press M first to enter manual mode"
                    )
                    node.get_logger().info(node._transition_status)
                return

            if keycode in (glfw.KEY_R,):
                if node.manual_mode and node.selected_joint >= 0:
                    name = node._joint_names[node.selected_joint]
                    adr = node._qpos_addresses[name]
                    node._data.qpos[adr] = node._clamp_joint(name, 0.0)
                    mujoco.mj_forward(node._model, node._data)
                    node.get_logger().info(f"Reset J{node.selected_joint + 1} to 0")
                return

            if keycode in (glfw.KEY_Z,):
                if node.manual_mode:
                    for name in node._joint_names:
                        adr = node._qpos_addresses[name]
                        node._data.qpos[adr] = node._clamp_joint(name, 0.0)
                    mujoco.mj_forward(node._model, node._data)
                    node.get_logger().info("Reset all joints to 0")
                return

            # Joint adjustment (only in manual mode)
            if node.manual_mode and node.selected_joint >= 0:
                delta = (
                    DELTA_PRISMATIC
                    if node.selected_joint == 0
                    else DELTA
                )
                name = node._joint_names[node.selected_joint]
                adr = node._qpos_addresses[name]

                if keycode in (glfw.KEY_UP, glfw.KEY_EQUAL, glfw.KEY_KP_ADD):
                    new_val = float(node._data.qpos[adr]) + delta
                    node._data.qpos[adr] = node._clamp_joint(name, new_val)
                    mujoco.mj_forward(node._model, node._data)
                    node.get_logger().info(
                        f"J{node.selected_joint + 1} += {delta:.2f} → "
                        f"{node._data.qpos[adr]:.3f}"
                    )
                elif keycode in (
                    glfw.KEY_DOWN, glfw.KEY_MINUS, glfw.KEY_KP_SUBTRACT,
                ):
                    new_val = float(node._data.qpos[adr]) - delta
                    node._data.qpos[adr] = node._clamp_joint(name, new_val)
                    mujoco.mj_forward(node._model, node._data)
                    node.get_logger().info(
                        f"J{node.selected_joint + 1} -= {delta:.2f} → "
                        f"{node._data.qpos[adr]:.3f}"
                    )
        except Exception:
            node.get_logger().error(
                f"Key callback failed for key {keycode}", exc_info=True
            )

    return _on_key


def _overlay_text(node: MuJoCoJointStateViewer) -> str:
    """Single-line status string rendered in the MuJoCo left panel."""
    if node.manual_mode:
        status = node._transition_status or "T = request transition"
        return (
            f"MANUAL | {status} | M = tracking"
        )
    status = node._transition_status or "Press M, adjust joints, then press T"
    return f"ROS tracking | {status}"


def main(args: Sequence[str] | None = None) -> int:
    """Run the viewer until its window is closed or Ctrl-C is pressed."""

    rclpy.init(args=args)
    node: MuJoCoJointStateViewer | None = None
    try:
        node = MuJoCoJointStateViewer()
        rate_hz = node.viewer_rate_hz()
        period_s = 1.0 / rate_hz
        node.get_logger().info(
            "MuJoCo viewer ready. Press M, adjust joints, then press T."
        )
        key_cb = _make_key_callback(node)
        with mujoco.viewer.launch_passive(
            node.model, node.data, key_callback=key_cb,
        ) as viewer:
            while rclpy.ok() and viewer.is_running():
                frame_start = monotonic()
                rclpy.spin_once(node, timeout_sec=min(period_s, 0.02))
                with viewer.lock():
                    node.apply_latest_joint_state()
                    node.apply_latest_obstacles()
                    node.process_transition_result()
                viewer.sync()
                remaining_s = period_s - (monotonic() - frame_start)
                if remaining_s > 0.0:
                    sleep(remaining_s)
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
