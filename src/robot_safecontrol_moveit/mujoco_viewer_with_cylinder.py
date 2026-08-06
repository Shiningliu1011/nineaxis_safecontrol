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
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from .plan_transition import (
    DEFAULT_JOINT_NAMES,
    load_mat_trajectory,
)


# The project URDF is authored in the legacy Y-up convention.  The wrapper is
# display-only: it rotates both robot and annotations for MuJoCo's Z-up view,
# while joint qpos values retain their URDF/MoveIt units and signs unchanged.
Y_UP_TO_Z_UP_EULER = "1.5707963267948966 0 0"


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
        scene_mjcf = self._inject_display_scene(
            raw_mjcf, target_path, tracking_cylinder,
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
        tracking_cylinder: TrackingCylinderSpec | None = None,
        joint_names: tuple[str, ...] = (),
    ) -> str:
        """Add display-only floor, lighting, path, frame axes,
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
        result = mjcf_xml.replace("<worldbody>", "<worldbody>" + world_prefix, 1)
        result = result.replace("</worldbody>", annotations + "    </body>\n  </worldbody>", 1)

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
