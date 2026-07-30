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
from pathlib import Path
from time import monotonic
from typing import Sequence

import numpy as np
import rclpy
import scipy.io
import yaml
from pymoveit2 import MoveIt2
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .continuous_ik import ContinuousIK, IKOptions
from .motion_planning import (
    CollisionObjectSpec,
    MotionPlanner,
    PlanningOptions,
)
from .trajectory_execution import TrajectoryExecutor


DEFAULT_JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9")


@dataclass(frozen=True)
class PipelineConfig:
    joint_names: tuple[str, ...]
    planning_group: str
    base_frame: str
    tool_link: str
    trajectory_mat: Path
    trajectory_offset_m: tuple[float, float, float]
    obstacles_file: Path
    max_points: int
    point_stride: int
    use_current_state: bool
    start_joint_positions: tuple[float, ...]
    joint_state_timeout_s: float
    orientation_xyzw: tuple[float, float, float, float]
    max_joint_delta: float
    ik_service_timeout_s: float
    planner_options: PlanningOptions
    scene_sync_timeout_s: float
    execute_transition: bool
    execute_task_path: bool
    dry_run: bool
    replay_rate_hz: float
    task_lead_time_s: float
    task_time_scale: float


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
        obstacles_file = self._path_parameter(
            "obstacles_file", self._default_obstacles_path()
        )
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
            obstacles_file=obstacles_file,
            max_points=int(get("max_points").value),
            point_stride=int(get("point_stride").value),
            use_current_state=bool(get("use_current_state").value),
            start_joint_positions=start_positions,
            joint_state_timeout_s=float(get("joint_state_timeout_s").value),
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
            scene_sync_timeout_s=float(get("scene_sync_timeout_s").value),
            execute_transition=bool(get("execute_transition").value),
            execute_task_path=bool(get("execute_task_path").value),
            dry_run=bool(get("dry_run").value),
            replay_rate_hz=float(get("replay_rate_hz").value),
            task_lead_time_s=float(get("task_lead_time_s").value),
            task_time_scale=float(get("task_time_scale").value),
        )
        if config.max_points < 0:
            raise ValueError("max_points must be zero (all) or positive")
        if config.point_stride < 1:
            raise ValueError("point_stride must be at least one")
        if config.joint_state_timeout_s <= 0.0:
            raise ValueError("joint_state_timeout_s must be positive")
        if config.scene_sync_timeout_s <= 0.0:
            raise ValueError("scene_sync_timeout_s must be positive")
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
        self.declare_parameter("obstacles_file", "")
        # One point is the safe quick-start; zero requests the full MAT sequence.
        self.declare_parameter("max_points", 1)
        self.declare_parameter("point_stride", 1)
        self.declare_parameter("use_current_state", True)
        self.declare_parameter("start_joint_positions", [0.0] * len(DEFAULT_JOINT_NAMES))
        self.declare_parameter("joint_state_timeout_s", 5.0)
        # KDL ignores this if config/kinematics.yaml has position_only_ik: true.
        self.declare_parameter("orientation_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("max_joint_delta", 0.15)
        self.declare_parameter("ik_service_timeout_s", 2.0)
        self.declare_parameter("planning_pipeline", "ompl")
        self.declare_parameter("planner_id", "RRTConnectkConfigDefault")
        self.declare_parameter("planning_time_s", 10.0)
        self.declare_parameter("planning_attempts", 5)
        self.declare_parameter("velocity_scale", 0.2)
        self.declare_parameter("acceleration_scale", 0.2)
        self.declare_parameter("goal_joint_tolerance", 0.001)
        self.declare_parameter("scene_sync_timeout_s", 5.0)
        # Both switches are deliberately opt-in for a real controller.
        self.declare_parameter("execute_transition", False)
        self.declare_parameter("execute_task_path", False)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("replay_rate_hz", 5.0)
        self.declare_parameter("task_lead_time_s", 0.05)
        self.declare_parameter("task_time_scale", 1.0)

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

    def _default_obstacles_path(self) -> Path:
        installed = self._installed_share_file("config/obstacles.yaml")
        return installed if installed.is_file() else self._source_root() / "config/obstacles.yaml"

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
    URDF, MoveIt PlanningScene, obstacle YAML, and this calibration offset all
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


def load_collision_objects(path: Path, default_frame: str) -> tuple[CollisionObjectSpec, ...]:
    """Load primitive PlanningScene objects from a reviewed YAML file."""

    with path.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    entries = document.get("obstacles", [])
    if not isinstance(entries, list):
        raise ValueError("obstacles.yaml must contain an 'obstacles' list")

    objects: list[CollisionObjectSpec] = []
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError("Every obstacle entry must be a mapping")
        try:
            objects.append(
                CollisionObjectSpec(
                    object_id=str(entry["id"]),
                    shape=str(entry["shape"]),
                    position=tuple(float(value) for value in entry["position"]),
                    dimensions=tuple(float(value) for value in entry["dimensions"]),
                    frame_id=str(entry.get("frame_id", default_frame)),
                    quaternion_xyzw=tuple(
                        float(value)
                        for value in entry.get("quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
                    ),
                )
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"Invalid collision object entry: {entry}") from error
    return tuple(objects)


def current_joint_state(
    node: Node,
    moveit: MoveIt2,
    joint_names: Sequence[str],
    timeout_s: float,
) -> JointState:
    """Wait for complete feedback from the configured ROS 2 controller."""

    deadline = monotonic() + timeout_s
    while monotonic() < deadline:
        state = moveit.joint_state
        if state is not None and all(name in state.name for name in joint_names):
            positions = dict(zip(state.name, state.position))
            result = JointState()
            result.header = state.header
            result.name = list(joint_names)
            result.position = [float(positions[name]) for name in joint_names]
            return result
        rclpy.spin_once(node, timeout_sec=0.05)
    raise RuntimeError(
        "No complete /joint_states feedback received. Start a ros2_control "
        "JointStateBroadcaster or pass use_current_state:=false explicitly."
    )


def configured_joint_state(
    joint_names: Sequence[str], positions: Sequence[float]
) -> JointState:
    if len(joint_names) != len(positions):
        raise ValueError("Joint names and configured positions have different lengths")
    state = JointState()
    state.name = list(joint_names)
    state.position = [float(value) for value in positions]
    return state


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
    planner = MotionPlanner(node, moveit, config.joint_names, config.planner_options)
    planner.install_collision_objects(
        load_collision_objects(config.obstacles_file, config.base_frame),
        sync_timeout_s=config.scene_sync_timeout_s,
    )

    start_state = (
        current_joint_state(
            node, moveit, config.joint_names, config.joint_state_timeout_s
        )
        if config.use_current_state
        else configured_joint_state(config.joint_names, config.start_joint_positions)
    )
    positions, source_times_s = load_mat_trajectory(
        config.trajectory_mat,
        config.trajectory_offset_m,
        config.max_points,
        config.point_stride,
    )

    ik = ContinuousIK(
        moveit,
        config.joint_names,
        IKOptions(
            tool_link=config.tool_link,
            orientation_xyzw=config.orientation_xyzw,
            max_joint_delta=config.max_joint_delta,
            service_timeout_s=config.ik_service_timeout_s,
        ),
    )
    ik_path = ik.solve(positions, start_state)
    node.get_logger().info(f"Continuous IK succeeded for {len(ik_path.states)} waypoint(s).")

    transition = planner.plan_transition(start_state, ik_path.first_state)
    node.get_logger().info(
        f"MoveIt planned a transition with {len(transition.points)} trajectory point(s)."
    )

    executor = TrajectoryExecutor(node, moveit, config.joint_names)
    if not config.execute_transition:
        node.get_logger().info(
            "Planning finished. Execution remains disabled; set execute_transition:=true "
            "and dry_run:=false only after reviewing the PlanningScene and controller."
        )
        return

    if config.dry_run:
        executor.execute(transition, dry_run=True, wait=True)
    else:
        executor.replay(transition, rate_hz=config.replay_rate_hz)
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
        executor.replay(task_trajectory, rate_hz=config.replay_rate_hz)
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
    except Exception as error:
        node.get_logger().error(f"Pipeline failed: {error}")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
