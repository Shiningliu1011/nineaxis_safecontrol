"""Thin orchestration node for the three MoveIt-backed pipeline modules.

The only domain stages are deliberately explicit:

1. :class:`ContinuousIK` solves the Cartesian input sequence via MoveIt.
2. :class:`MotionPlanner` plans the collision-free transition to its first IK
   state via MoveIt/OMPL.
3. :class:`TrajectoryExecutor` validates and optionally executes the resulting
   MoveIt trajectories.

This file contains input/configuration plumbing only; it does not implement IK,
collision checking, or a motion planner.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, sqrt
from pathlib import Path
from time import monotonic
from typing import Sequence

import numpy as np
import rclpy
import scipy.io
from pymoveit2 import MoveIt2
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from .continuous_ik import ContinuousIK, IKOptions
from .motion_planning import (
    MotionPlanner,
    PlanningOptions,
)
from .trajectory_execution import TrajectoryExecutor
from .task_target import (
    compute_first_task_orientation,
    load_first_task_target,
    solve_first_task_state,
)


DEFAULT_JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9")


@dataclass(frozen=True)
class PipelineConfig:
    joint_names: tuple[str, ...]
    planning_group: str
    base_frame: str
    tool_link: str
    trajectory_mat: Path
    trajectory_offset_m: tuple[float, float, float]
    max_points: int
    point_stride: int
    use_current_state: bool
    start_joint_positions: tuple[float, ...]
    joint_state_timeout_s: float
    allow_joint_state_fallback: bool
    orientation_xyzw: tuple[float, float, float, float]
    max_joint_delta: float
    ik_service_timeout_s: float
    planner_options: PlanningOptions
    execute_transition: bool
    execute_task_path: bool
    dry_run: bool
    replay_rate_hz: float
    task_lead_time_s: float
    task_time_scale: float
    align_tool_x_to_surface_normal: bool
    cylinder_axis_direction: tuple[float, float, float]


class TransitionPipelineNode(Node):
    """Loads explicit ROS parameters for a reproducible MoveIt request."""

    def __init__(self) -> None:
        super().__init__("plan_transition")
        self._declare_parameters()

    def configuration(self) -> PipelineConfig:
        get = self.get_parameter
        joint_names = tuple(str(value) for value in get("joint_names").value)
        if not joint_names:
            raise ValueError("joint_names must not be empty")

        trajectory_mat = self._path_parameter("trajectory_mat", self._default_mat_path())
        start_positions = tuple(
            float(value) for value in get("start_joint_positions").value
        )
        if len(start_positions) != len(joint_names):
            raise ValueError(
                "start_joint_positions must have the same length as joint_names"
            )

        config = PipelineConfig(
            joint_names=joint_names,
            planning_group=str(get("planning_group").value),
            base_frame=str(get("base_frame").value),
            tool_link=str(get("tool_link").value),
            trajectory_mat=trajectory_mat,
            trajectory_offset_m=self._float_tuple("trajectory_offset_m", 3),
            max_points=int(get("max_points").value),
            point_stride=int(get("point_stride").value),
            use_current_state=bool(get("use_current_state").value),
            start_joint_positions=start_positions,
            joint_state_timeout_s=float(get("joint_state_timeout_s").value),
            allow_joint_state_fallback=bool(
                get("allow_joint_state_fallback").value
            ),
            orientation_xyzw=self._float_tuple("orientation_xyzw", 4),
            max_joint_delta=float(get("max_joint_delta").value),
            ik_service_timeout_s=float(get("ik_service_timeout_s").value),
            planner_options=PlanningOptions(
                pipeline_id=str(get("planning_pipeline").value),
                planner_id=str(get("planner_id").value),
                planning_time_s=float(get("planning_time_s").value),
                planning_attempts=int(get("planning_attempts").value),
                velocity_scale=float(get("velocity_scale").value),
                acceleration_scale=float(get("acceleration_scale").value),
                goal_joint_tolerance=float(get("goal_joint_tolerance").value),
            ),
            execute_transition=bool(get("execute_transition").value),
            execute_task_path=bool(get("execute_task_path").value),
            dry_run=bool(get("dry_run").value),
            replay_rate_hz=float(get("replay_rate_hz").value),
            task_lead_time_s=float(get("task_lead_time_s").value),
            task_time_scale=float(get("task_time_scale").value),
            align_tool_x_to_surface_normal=bool(
                get("align_tool_x_to_surface_normal").value
            ),
            cylinder_axis_direction=self._float_tuple(
                "cylinder_axis_direction", 3
            ),
        )
        if config.max_points < 0:
            raise ValueError("max_points must be zero (all) or positive")
        if config.point_stride < 1:
            raise ValueError("point_stride must be at least one")
        if config.joint_state_timeout_s <= 0.0:
            raise ValueError("joint_state_timeout_s must be positive")
        if config.execute_task_path and not config.execute_transition:
            raise ValueError(
                "execute_task_path requires execute_transition=true so the robot first "
                "reaches the first IK waypoint"
            )
        return config

    def _declare_parameters(self) -> None:
        self.declare_parameter("joint_names", list(DEFAULT_JOINT_NAMES))
        self.declare_parameter("planning_group", "arm")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tool_link", "tool0")
        # Empty path values select package/source-tree defaults at runtime.
        self.declare_parameter("trajectory_mat", "")
        self.declare_parameter("trajectory_offset_m", [0.0, 0.343, 1.587])
        # One point is the safe quick-start; zero requests the full MAT sequence.
        self.declare_parameter("max_points", 1)
        self.declare_parameter("point_stride", 1)
        self.declare_parameter("use_current_state", True)
        self.declare_parameter("start_joint_positions", [0.0] * len(DEFAULT_JOINT_NAMES))
        self.declare_parameter("joint_state_timeout_s", 5.0)
        self.declare_parameter("max_joint_state_age_s", 1.0)
        self.declare_parameter("allow_joint_state_fallback", False)
        self.declare_parameter("joint_state_topic", "/joint_states")
        # KDL ignores this if config/kinematics.yaml has position_only_ik: true.
        self.declare_parameter("orientation_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("max_joint_delta", 0.15)
        self.declare_parameter("ik_service_timeout_s", 2.0)
        self.declare_parameter("planning_pipeline", "ompl")
        self.declare_parameter(
            "planner_id", "AEBRRTstarFaithfulConfigDefault"
        )
        self.declare_parameter("planning_time_s", 10.0)
        self.declare_parameter("planning_attempts", 1)
        self.declare_parameter("velocity_scale", 0.2)
        self.declare_parameter("acceleration_scale", 0.2)
        self.declare_parameter("goal_joint_tolerance", 0.001)
        # Both switches are deliberately opt-in for a real controller.
        self.declare_parameter("execute_transition", False)
        self.declare_parameter("execute_task_path", False)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("replay_rate_hz", 5.0)
        self.declare_parameter("replay_joint_state_topic", "/joint_states")
        self.declare_parameter("task_lead_time_s", 0.05)
        self.declare_parameter("task_time_scale", 1.0)
        # When enabled, every IK waypoint is solved with tool0's X-axis
        # aligned to the outward radial (surface-normal) direction of the
        # cylinder fitted to the trajectory points.
        self.declare_parameter("align_tool_x_to_surface_normal", False)
        self.declare_parameter("cylinder_axis_direction", [0.0, 1.0, 0.0])

    def _float_tuple(self, name: str, expected_length: int) -> tuple[float, ...]:
        values = tuple(float(value) for value in self.get_parameter(name).value)
        if len(values) != expected_length:
            raise ValueError(f"{name} must contain {expected_length} values")
        return values

    def _path_parameter(self, name: str, default: Path) -> Path:
        value = str(self.get_parameter(name).value).strip()
        path = Path(value).expanduser() if value else default
        if not path.is_file():
            raise FileNotFoundError(f"{name} does not exist: {path}")
        return path

    @staticmethod
    def _source_root() -> Path:
        return Path(__file__).resolve().parents[2]

    def _default_mat_path(self) -> Path:
        installed = self._installed_share_file("data/nurbs/ik_input.mat")
        return installed if installed.is_file() else self._source_root() / "data/nurbs/ik_input.mat"

    @staticmethod
    def _installed_share_file(relative_path: str) -> Path:
        try:
            from ament_index_python.packages import get_package_share_directory

            return Path(get_package_share_directory("robot_safecontrol_moveit")) / relative_path
        except (ImportError, LookupError):
            return Path("/__robot_safecontrol_moveit_not_installed__")


def load_mat_trajectory(
    path: Path,
    offset_m: Sequence[float],
    max_points: int,
    point_stride: int,
) -> tuple[list[tuple[float, float, float]], list[float]]:
    """Load the MAT position path in ``base_link`` coordinates.

    The legacy MuJoCo-only Y-up→Z-up conversion is intentionally absent.  The
    URDF, MoveIt PlanningScene, and this calibration offset all
    use the same ``base_link`` coordinates.
    """

    mat_data = scipy.io.loadmat(path)
    try:
        ik_input = mat_data["ik_input"][0, 0]
        positions_mm = np.asarray(ik_input["position_series"], dtype=float)
        times_s = np.asarray(ik_input["time_series"], dtype=float).reshape(-1)
    except (KeyError, IndexError, TypeError) as error:
        raise ValueError(f"{path} is not a supported ik_input.mat file") from error

    if positions_mm.ndim != 2 or positions_mm.shape[1] != 3:
        raise ValueError("position_series must have shape (N, 3)")
    if positions_mm.shape[0] != times_s.shape[0]:
        raise ValueError("position_series and time_series have different lengths")
    if not np.isfinite(positions_mm).all() or not np.isfinite(times_s).all():
        raise ValueError("MAT trajectory contains non-finite values")

    indices = np.arange(0, len(positions_mm), point_stride, dtype=int)
    if max_points > 0:
        indices = indices[:max_points]
    if len(indices) == 0:
        raise ValueError("Trajectory selection produced no waypoints")

    offset = np.asarray(offset_m, dtype=float)
    positions_m = positions_mm[indices] / 1000.0 + offset
    selected_times = times_s[indices]
    return (
        [tuple(float(value) for value in point) for point in positions_m],
        [float(value) for value in selected_times],
    )


def current_joint_state(
    node: Node,
    moveit: MoveIt2,
    joint_names: Sequence[str],
    timeout_s: float,
    *,
    topic: str = "/joint_states",
    max_age_s: float = 1.0,
    allow_fallback: bool = False,
) -> JointState:
    """Wait for complete joint state feedback from the configured topic.

    Creates a short-lived subscription to *topic* with the same QoS the
    MuJoCo Viewer publisher uses (``qos_profile_sensor_data``, i.e.
    BEST_EFFORT), so publisher and subscriber are always compatible.

    When *allow_fallback* is False (default) and no state arrives on *topic*,
    the call raises ``RuntimeError("START_STATE_UNAVAILABLE")`` — it never
    silently uses MoveIt's internal state monitor, which may hold stale or
    unrelated data.
    """
    names_set = set(joint_names)
    latest: JointState | None = None

    def _cb(msg: JointState) -> None:
        nonlocal latest
        if names_set.issubset(msg.name):
            latest = msg

    # Use the same QoS as the Viewer publisher so BEST_EFFORT ↔ BEST_EFFORT.
    sub = node.create_subscription(
        JointState, topic, _cb, qos_profile_sensor_data
    )
    try:
        deadline = monotonic() + timeout_s
        while monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            candidate = latest
            if candidate is not None:
                stamp_sec = (
                    candidate.header.stamp.sec
                    + candidate.header.stamp.nanosec * 1e-9
                )
                now_sec = node.get_clock().now().nanoseconds * 1e-9
                age_s = now_sec - stamp_sec
                if age_s <= max_age_s:
                    positions = dict(zip(candidate.name, candidate.position))
                    if len(candidate.position) != len(candidate.name):
                        raise RuntimeError(
                            "START_STATE_MALFORMED: position/name length mismatch"
                        )
                    result = JointState()
                    result.header = candidate.header
                    result.name = list(joint_names)
                    result.position = [
                        float(positions[name]) for name in joint_names
                    ]
                    if not all(isfinite(float(p)) for p in result.position):
                        raise RuntimeError(
                            "START_STATE_NON_FINITE: current joint state "
                            "contains NaN or Inf values"
                        )
                    node.get_logger().info(
                        f"Planning start state source: {topic}; "
                        f"age={age_s:.3f}s, "
                        f"positions={[f'{v:.3f}' for v in result.position]}"
                    )
                    return result
                else:
                    node.get_logger().warning(
                        f"Joint state from {topic} is stale "
                        f"(age={age_s:.3f}s > max_age={max_age_s:.3f}s); "
                        "waiting for fresher data"
                    )
                    latest = None

        # Fallback is DISABLED by default (allow_fallback=False).
        if not allow_fallback:
            raise RuntimeError(
                "START_STATE_UNAVAILABLE: No complete joint state received on "
                f"{topic} within {timeout_s:.1f}s. Start a JointStateBroadcaster, "
                "check manual_joint_state_topic, or pass use_current_state:=false."
            )

        # Only reached when allow_fallback=True — validate fallback thoroughly.
        state = moveit.joint_state
        if state is None or not all(name in state.name for name in joint_names):
            raise RuntimeError(
                "START_STATE_UNAVAILABLE: No complete joint state on "
                f"{topic} and MoveIt fallback also unavailable."
            )
        positions = dict(zip(state.name, state.position))
        if len(state.position) != len(state.name):
            raise RuntimeError(
                "START_STATE_MALFORMED: fallback position/name length mismatch"
            )
        result = JointState()
        result.header = state.header
        result.name = list(joint_names)
        result.position = [float(positions[name]) for name in joint_names]
        if not all(isfinite(float(p)) for p in result.position):
            raise RuntimeError(
                "START_STATE_NON_FINITE: fallback state contains NaN or Inf"
            )
        stamp_sec = (
            result.header.stamp.sec
            + result.header.stamp.nanosec * 1e-9
        )
        if stamp_sec == 0.0:
            raise RuntimeError(
                "START_STATE_STALE: fallback state has zero timestamp"
            )
        now_sec = node.get_clock().now().nanoseconds * 1e-9
        age_s = now_sec - stamp_sec
        if age_s > max_age_s:
            raise RuntimeError(
                f"START_STATE_STALE: fallback state age={age_s:.3f}s "
                f"exceeds max_age={max_age_s:.3f}s"
            )
        node.get_logger().warning(
            "WARNING: start state is using MoveIt fallback, "
            f"not MuJoCo manual topic {topic}; age={age_s:.3f}s"
        )
        return result
    finally:
        node.destroy_subscription(sub)


def configured_joint_state(
    joint_names: Sequence[str], positions: Sequence[float]
) -> JointState:
    if len(joint_names) != len(positions):
        raise ValueError("Joint names and configured positions have different lengths")
    state = JointState()
    state.name = list(joint_names)
    state.position = [float(value) for value in positions]
    return state


# ---------------------------------------------------------------------------
#  Surface-normal → quaternion helper
# ---------------------------------------------------------------------------


def _rotation_matrix_to_quaternion_xyzw(
    matrix: np.ndarray,
) -> tuple[float, float, float, float]:
    """Convert a 3×3 rotation matrix to a normalised xyzw quaternion."""
    m = np.asarray(matrix, dtype=float)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        qw = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        qw = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        qw = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s
    # Normalise
    length = sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    return (float(qx / length), float(qy / length), float(qz / length), float(qw / length))


def compute_surface_normal_orientations(
    points: list[tuple[float, float, float]],
    axis_direction: tuple[float, float, float],
    *,
    fit_points: Sequence[Sequence[float]] | None = None,
) -> list[tuple[float, float, float, float]]:
    """Return one xyzw quaternion per point, each with X-axis = outward radial
    (surface normal) of the cylinder whose *axis_direction* passes through the
    fitted centre.

    The tool frame is:
      X  ←  surface normal  (radial, orthogonal to cylinder axis)
      Y  ←  cylinder axis   (axial direction along the cylinder)
      Z  ←  X × Y           (tangent to the cylinder surface)

    *fit_points* supplies the samples used to fit the cylinder axis centre.
    Passing the *full* trajectory here is important: the IK waypoints may lie
    in a near-stationary segment where a least-squares circle fit degenerates
    (radius → 0) and the implied normal direction becomes numerical noise.

    Note on the sign: the X-axis points *toward* the cylinder axis (inward
    radial).  The mathematical outward normal points away from the axis, which
    for this trajectory lies below the robot's reachable workspace — the arm's
    tool0 cannot point downward, so the inward direction is used instead.
    """
    values = np.asarray(points, dtype=float)
    axis = np.asarray(axis_direction, dtype=float)
    axis_len = float(np.linalg.norm(axis))
    if axis_len < 1e-12:
        raise ValueError("cylinder_axis_direction must be a non-zero 3-vector")
    axis /= axis_len

    # Fit a cylinder axis centre by projecting the *fit* samples onto the
    # plane perpendicular to the axis.
    fit_values = (
        np.asarray(fit_points, dtype=float)
        if fit_points is not None
        else values
    )
    if fit_values.ndim != 2 or fit_values.shape[1] != 3 or len(fit_values) < 3:
        raise ValueError("fit_points must be an (N, 3) array with at least 3 samples")

    helper = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(helper, axis))) > 0.9:
        helper = np.array([0.0, 0.0, 1.0])
    u = np.cross(axis, helper)
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    v /= np.linalg.norm(v)

    plane_x = fit_values @ u
    plane_y = fit_values @ v
    A = np.column_stack((plane_x, plane_y, np.ones(len(fit_values))))
    rhs = -(plane_x * plane_x + plane_y * plane_y)
    coeff, *_ = np.linalg.lstsq(A, rhs, rcond=None)
    d_val, e_val, _f_val = coeff
    cx = -0.5 * d_val
    cy = -0.5 * e_val

    # 3-D centre of the cylinder axis
    axial_vals = fit_values @ axis
    axial_centre = 0.5 * (float(axial_vals.min()) + float(axial_vals.max()))
    centre = cx * u + cy * v + axial_centre * axis

    orientations: list[tuple[float, float, float, float]] = []
    for point in values:
        rel = point - centre
        axial = axis * float(np.dot(rel, axis))
        radial = rel - axial
        rlen = float(np.linalg.norm(radial))
        if rlen < 1e-12:
            # Point lies on the cylinder axis — fall back to identity.
            orientations.append((0.0, 0.0, 0.0, 1.0))
            continue
        # X-axis points toward the cylinder axis (inward radial): the outward
        # normal would point below the robot's reachable workspace.
        col_x = -radial / rlen

        # Build orthonormal frame:  cols = (X, Y, Z)
        col_y = axis  # cylinder axis
        col_z = np.cross(col_x, col_y)
        col_z /= np.linalg.norm(col_z)
        # Re-orthogonalise Y against X and Z
        col_y = np.cross(col_z, col_x)

        rot = np.column_stack((col_x, col_y, col_z))
        orientations.append(_rotation_matrix_to_quaternion_xyzw(rot))

    return orientations


def run_pipeline(node: TransitionPipelineNode) -> None:
    config = node.configuration()
    node.get_logger().info(
        "Pipeline: continuous IK -> MoveIt transition planning -> controlled execution"
    )
    node.get_logger().info(
        f"MAT input: {config.trajectory_mat}; max_points={config.max_points or 'all'}; "
        f"stride={config.point_stride}"
    )

    moveit = MoveIt2(
        node=node,
        joint_names=list(config.joint_names),
        base_link_name=config.base_frame,
        end_effector_name=config.tool_link,
        group_name=config.planning_group,
        ignore_new_calls_while_executing=True,
    )
    planner = MotionPlanner(
        node,
        moveit,
        config.joint_names,
        config.planner_options,
        planning_group=config.planning_group,
    )

    start_state = (
        current_joint_state(
            node,
            moveit,
            config.joint_names,
            config.joint_state_timeout_s,
            topic=str(node.get_parameter("joint_state_topic").value),
            max_age_s=float(node.get_parameter("max_joint_state_age_s").value),
            allow_fallback=bool(
                node.get_parameter("allow_joint_state_fallback").value
            ),
        )
        if config.use_current_state
        else configured_joint_state(config.joint_names, config.start_joint_positions)
    )
    positions, source_times_s = load_first_task_target(
        config.trajectory_mat,
        config.trajectory_offset_m,
        config.max_points,
        config.point_stride,
    )

    # Compute first-target orientation via shared logic (Issue #9).
    first_quat, per_point_orientations = compute_first_task_orientation(
        positions,
        align_tool_x_to_surface_normal=config.align_tool_x_to_surface_normal,
        cylinder_axis_direction=config.cylinder_axis_direction,
        orientation_xyzw=config.orientation_xyzw,
        trajectory_mat=config.trajectory_mat,
        offset_m=config.trajectory_offset_m,
    )

    if per_point_orientations:
        node.get_logger().info(
            f"Computed {len(per_point_orientations)} surface-normal-aligned "
            f"orientation(s); cylinder fitted on full trajectory samples; "
            f"axis={config.cylinder_axis_direction}."
        )

    node.get_logger().info(
        f"FIRST_TARGET_POSITION={[f'{v:.4f}' for v in positions[0]]}"
    )
    node.get_logger().info(
        f"FIRST_TARGET_ORIENTATION=[{', '.join(f'{v:.6f}' for v in first_quat)}]"
    )

    # Use ContinuousIK directly so we have access to the full IKPath
    # (needed for task trajectory execution below).
    from .continuous_ik import ContinuousIK, IKOptions as CIKOptions
    ik = ContinuousIK(
        moveit,
        config.joint_names,
        CIKOptions(
            tool_link=config.tool_link,
            orientation_xyzw=first_quat,
            planning_group=config.planning_group,
            base_frame=config.base_frame,
            max_joint_delta=config.max_joint_delta,
            service_timeout_s=config.ik_service_timeout_s,
        ),
    )
    # Log IK request before solving.
    node.get_logger().info(
        f"IK_REQUEST "
        f"position=({positions[0][0]:.4f},{positions[0][1]:.4f},"
        f"{positions[0][2]:.4f}) "
        f"orientation_xyzw=({first_quat[0]:.6f},{first_quat[1]:.6f},"
        f"{first_quat[2]:.6f},{first_quat[3]:.6f}) "
        f"base_frame={config.base_frame} "
        f"tool_link={config.tool_link} "
        f"planning_group={config.planning_group} "
        f"seed_names={list(config.joint_names)} "
        f"seed_positions=[{', '.join(f'{v:.3f}' for v in start_state.position)}] "
        f"align_tool_x_to_surface_normal="
        f"{'true' if per_point_orientations else 'false'}"
    )
    ik_path = ik.solve(
        positions,
        start_state,
        orientations=per_point_orientations,
    )
    first_goal = ik_path.first_state
    node.get_logger().info(
        f"Continuous IK succeeded for {len(ik_path.states)} waypoint(s). "
        f"First IK goal: {[f'{v:.3f}' for v in first_goal.position]}"
    )

    plan_start_time = monotonic()
    try:
        transition = planner.plan_transition(start_state, first_goal)
        plan_elapsed = monotonic() - plan_start_time
        node.get_logger().info(
            f"TRANSITION_PLANNED: "
            f"planner={config.planner_options.planner_id}, "
            f"time={plan_elapsed:.3f}s, "
            f"points={len(transition.points)}, "
            f"start={[f'{v:.3f}' for v in start_state.position]}, "
            f"goal={[f'{v:.3f}' for v in first_goal.position]}"
        )
    except Exception:
        node.get_logger().error(
            f"TRANSITION_FAILED after {monotonic() - plan_start_time:.3f}s: "
            f"start={[f'{v:.3f}' for v in start_state.position]}, "
            f"goal={[f'{v:.3f}' for v in first_goal.position]}"
        )
        raise

    executor = TrajectoryExecutor(node, moveit, config.joint_names)
    if not config.execute_transition:
        node.get_logger().info(
            "Planning finished. Execution remains disabled; set execute_transition:=true "
            "and dry_run:=false only after reviewing the PlanningScene and controller."
        )
        return

    replay_topic = str(node.get_parameter("replay_joint_state_topic").value)
    if config.dry_run:
        executor.execute(transition, dry_run=True, wait=True)
    else:
        executor.replay(
            transition,
            topic=replay_topic,
            rate_hz=config.replay_rate_hz,
            switch_viewer_to_tracking=True,
        )
    if not config.execute_task_path:
        node.get_logger().info("Transition execution stage finished.")
        return

    task_trajectory = executor.make_task_trajectory(
        ik_path.states,
        source_times_s,
        lead_time_s=config.task_lead_time_s,
        time_scale=config.task_time_scale,
    )
    if config.dry_run:
        executor.execute(task_trajectory, dry_run=True, wait=True)
    else:
        executor.replay(
            task_trajectory,
            topic=replay_topic,
            rate_hz=config.replay_rate_hz,
            switch_viewer_to_tracking=False,
        )
    node.get_logger().info("Continuous task trajectory execution stage finished.")


def main(args: Sequence[str] | None = None) -> int:
    rclpy.init(args=args)
    node = TransitionPipelineNode()
    try:
        run_pipeline(node)
        return 0
    except KeyboardInterrupt:
        node.get_logger().warning("Pipeline interrupted by user.")
        return 130
    except FileNotFoundError as error:
        node.get_logger().error(f"CONFIG_ERROR: {error}")
        return 1
    except ValueError as error:
        node.get_logger().error(f"CONFIG_ERROR: {error}")
        return 1
    except RuntimeError as error:
        msg = str(error)
        if msg.startswith("START_STATE_"):
            node.get_logger().error(f"START_STATE_INVALID: {msg}")
        elif "PlanningScene" in msg:
            node.get_logger().error(f"SCENE_ERROR: {msg}")
        elif "IK" in msg or "ik_input" in msg:
            node.get_logger().error(f"IK_ERROR: {msg}")
        elif "trajectory" in msg.lower() or "PLANNER" in msg:
            node.get_logger().error(f"PLANNING_ERROR: {msg}")
        elif "execute" in msg.lower() or "Execution" in msg:
            node.get_logger().error(f"EXECUTION_ERROR: {msg}")
        else:
            node.get_logger().error(f"PIPELINE_ERROR: {msg}")
        return 1
    except Exception as error:
        node.get_logger().error(f"UNEXPECTED_ERROR: {error}")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
