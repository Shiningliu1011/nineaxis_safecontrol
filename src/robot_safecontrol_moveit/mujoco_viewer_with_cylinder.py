"""Display the project URDF in MuJoCo and mirror ROS 2 joint feedback.

This is deliberately a visualisation bridge, not a second planner, IK solver,
or controller.  MoveIt 2 remains responsible for planning and collision
checking; this node loads ``ninezzhou.urdf`` and its STL meshes from this
repository, then writes the received ``/joint_states`` values to MuJoCo's
``qpos`` array for inspection.
"""

from __future__ import annotations

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
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

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
        if show_target_path:
            target_path = self._load_display_path(
                self._file_parameter("trajectory_mat", self._default_trajectory_mat()),
                self._float_tuple("trajectory_offset_m", 3),
                int(self.get_parameter("path_max_points").value),
            )

        tracking_cylinder: TrackingCylinderSpec | None = None
        if (
            target_path
            and bool(self.get_parameter("show_tracking_cylinder").value)
        ):
            tracking_cylinder = self._fit_tracking_cylinder(
                target_path,
                self._float_tuple("tracking_cylinder_axis_direction", 3),
                float(self.get_parameter("tracking_cylinder_height_margin_m").value),
            )
            if bool(self.get_parameter("project_display_path_to_cylinder").value):
                target_path = self._project_path_to_cylinder(
                    target_path,
                    tracking_cylinder,
                    float(self.get_parameter("path_surface_offset_m").value),
                )
            self.get_logger().info(
                "Tracking cylinder fitted in base_link: "
                f"center={tracking_cylinder.center}, "
                f"axis={tracking_cylinder.axis_direction}, "
                f"radius={tracking_cylinder.radius:.6f} m, "
                f"height={tracking_cylinder.height:.6f} m, "
                f"radial RMS={tracking_cylinder.radial_rms_error * 1000.0:.3f} mm, "
                f"max={tracking_cylinder.radial_max_error * 1000.0:.3f} mm."
            )

        obstacles: tuple[CollisionObjectSpec, ...] = ()
        if bool(self.get_parameter("show_obstacles").value):
            obstacles = load_collision_objects(
                self._file_parameter("obstacles_file", self._default_obstacles_file()),
                "base_link",
            )

        raw_mjcf = self._urdf_to_mjcf(urdf_path, mesh_directory)
        scene_mjcf = self._inject_display_scene(
            raw_mjcf, target_path, obstacles, tracking_cylinder
        )
        self._model = mujoco.MjModel.from_xml_string(scene_mjcf)
        self._model.opt.gravity[:] = 0.0
        self._data = mujoco.MjData(self._model)
        self._qpos_addresses = self._joint_qpos_addresses(joint_names)
        self._latest_positions: dict[str, float] = {}
        self._received_joint_state = False

        topic = str(self.get_parameter("joint_state_topic").value)
        self.create_subscription(
            JointState,
            topic,
            self._joint_state_callback,
            qos_profile_sensor_data,
        )
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
        """Copy the latest complete joint state to MuJoCo and run forward kinematics."""

        if not self._received_joint_state:
            return False
        for name, qpos_address in self._qpos_addresses.items():
            self._data.qpos[qpos_address] = self._latest_positions[name]
        mujoco.mj_forward(self._model, self._data)
        return True

    def _declare_parameters(self) -> None:
        self.declare_parameter("joint_names", list(DEFAULT_JOINT_NAMES))
        self.declare_parameter("joint_state_topic", "/joint_states")
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
        # Lift the red display line very slightly above the transparent surface
        # to avoid z-fighting. This changes visualization only, not IK input.
        self.declare_parameter("path_surface_offset_m", 0.002)
        self.declare_parameter("show_obstacles", True)
        self.declare_parameter("obstacles_file", "")

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

    def _load_display_path(
        self,
        trajectory_mat: Path,
        offset_m: Sequence[float],
        maximum_points: int,
    ) -> list[tuple[float, float, float]]:
        if maximum_points < 2:
            raise ValueError("path_max_points must be at least two")
        points, _ = load_mat_trajectory(trajectory_mat, offset_m, max_points=0, point_stride=1)
        if len(points) <= maximum_points:
            return points
        last = len(points) - 1
        indices = [round(index * last / (maximum_points - 1)) for index in range(maximum_points)]
        return [points[index] for index in indices]

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
    ) -> str:
        """Add display-only floor, lighting, path and reviewed MoveIt obstacles."""

        if "<worldbody>" not in mjcf_xml or "</worldbody>" not in mjcf_xml:
            raise ValueError("MuJoCo conversion did not produce a worldbody")

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
        return result.replace("</worldbody>", annotations + "    </body>\n  </worldbody>", 1)

    @staticmethod
    def _fit_tracking_cylinder(
        points: Sequence[tuple[float, float, float]],
        axis_direction: Sequence[float],
        height_margin_m: float,
    ) -> TrackingCylinderSpec:
        """Fit a circle in the plane normal to a configured cylinder axis."""

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
        axial_center = 0.5 * (float(axial_values.min()) + float(axial_values.max()))
        height = float(axial_values.max() - axial_values.min()) + 2.0 * height_margin_m
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
    def _share_file(package_name: str, relative_path: str) -> Path:
        try:
            return Path(get_package_share_directory(package_name)) / relative_path
        except PackageNotFoundError as error:
            raise FileNotFoundError(
                f"ROS package '{package_name}' is not available; source install/setup.bash first"
            ) from error

    def _default_urdf_path(self) -> Path:
        return self._share_file("ninezzhou", "urdf/ninezzhou.urdf")

    def _default_mesh_directory(self) -> Path:
        return self._share_file("ninezzhou", "meshes")

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


def main(args: Sequence[str] | None = None) -> int:
    """Run the passive viewer until its window is closed or Ctrl-C is pressed."""

    rclpy.init(args=args)
    node: MuJoCoJointStateViewer | None = None
    try:
        node = MuJoCoJointStateViewer()
        rate_hz = node.viewer_rate_hz()
        period_s = 1.0 / rate_hz
        node.get_logger().info(
            "MuJoCo viewer ready. Start MoveIt/demo and run plan_transition; "
            "the project arm will follow /joint_states."
        )
        with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
            while rclpy.ok() and viewer.is_running():
                frame_start = monotonic()
                rclpy.spin_once(node, timeout_sec=min(period_s, 0.02))
                with viewer.lock():
                    node.apply_latest_joint_state()
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
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
