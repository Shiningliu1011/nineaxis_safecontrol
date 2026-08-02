"""Unified launch: move_group + robot_state_publisher + planning server + Viewer.

Start with::

    ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py

Then: press M, adjust joints, press T to trigger planning.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_safecontrol_moveit.moveit_runtime_config import build_moveit_params


def generate_launch_description() -> LaunchDescription:
    share_dir = Path(get_package_share_directory("robot_safecontrol_moveit"))

    # --- Build the shared MoveIt parameter dict once ------------------------
    moveit_params = build_moveit_params(share_dir)

    # --- Runtime config for our own nodes ----------------------------------
    runtime_yaml = str(share_dir / "config" / "mujoco_transition_runtime.yaml")

    # --- Obstacles YAML for static_obstacle_publisher ----------------------
    obstacles_yaml = str(share_dir / "config" / "obstacles.yaml")

    # --- Validate required config files exist ------------------------------
    for path_str, label in [
        (runtime_yaml, "runtime YAML"),
        (obstacles_yaml, "obstacles YAML"),
    ]:
        if not Path(path_str).is_file():
            raise FileNotFoundError(
                f"{label} not found: {path_str}"
            )

    return LaunchDescription(
        [
            # Defaults to the full interactive demo. The opt-out exists only
            # for headless launch tests; users need not pass this argument.
            DeclareLaunchArgument("start_viewer", default_value="true"),
            # Log startup info.
            LogInfo(
                msg=f"Starting unified MoveIt transition demo. "
                f"share={share_dir}"
            ),

            # 1. robot_state_publisher — remaps /joint_states → /mujoco_joint_states
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description":
                            moveit_params[
                                "robot_description"
                            ]
                    }
                ],
                remappings=[
                    ("joint_states", "/mujoco_joint_states"),
                ],
            ),

            # 2. move_group — the real MoveIt 2 motion planning node
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                name="move_group",
                output="screen",
                parameters=[
                    moveit_params,
                    {
                        "publish_robot_description_semantic": True,
                        "publish_robot_description": True,
                        "use_sim_time": False,
                    },
                ],
                # The planning-scene monitor must consume exactly the same
                # state stream as the Viewer, replay, and RSP.
                remappings=[
                    ("joint_states", "/mujoco_joint_states"),
                ],
            ),

            # 3. Static obstacle publisher. Static and dynamic objects use
            # separate topics/QoS contracts; the planning server applies the
            # retained static registry to MoveIt.
            Node(
                package="robot_safecontrol_moveit",
                executable="static_obstacle_publisher",
                name="static_obstacle_publisher",
                output="screen",
                parameters=[
                    {
                        "obstacles_file": obstacles_yaml,
                        "static_collision_object_topic": "/static_collision_object",
                        "default_frame_id": "base_link",
                    }
                ],
            ),

            # 4. Transition planning server (persistent, multi-threaded)
            Node(
                package="robot_safecontrol_moveit",
                executable="transition_planning_server",
                name="transition_planning_server",
                output="screen",
                parameters=[runtime_yaml],
                # pymoveit2 creates its state monitor on the hosting node;
                # keep that internal subscription on the same MuJoCo stream.
                remappings=[
                    ("joint_states", "/mujoco_joint_states"),
                ],
            ),

            # 5. MuJoCo Viewer (delayed so move_group and server are ready)
            TimerAction(
                period=3.0,
                condition=IfCondition(LaunchConfiguration("start_viewer")),
                actions=[
                    Node(
                        package="robot_safecontrol_moveit",
                        executable="mujoco_viewer",
                        name="mujoco_joint_state_viewer",
                        output="screen",
                        parameters=[runtime_yaml],
                    ),
                ],
            ),
        ]
    )
