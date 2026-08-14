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

    # --- Validate required config files exist ------------------------------
    for path_str, label in [
        (runtime_yaml, "runtime YAML"),
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
            # The OSCBF controller closes the loop on the MuJoCo joint-state
            # stream; disable it only for MoveIt-only demos or CI.
            DeclareLaunchArgument("start_oscbf_controller", default_value="true"),
            # Jerk-limited actuator plant (closed-loop OSCBF flow).  Off for
            # the interactive M/T MoveIt demo, where manual mode owns the
            # joint-state stream.
            DeclareLaunchArgument("start_oscbf_plant", default_value="false"),
            # With the plant on, the controller must wait for the transition
            # replay to finish before it takes over the command stream.
            DeclareLaunchArgument("oscbf_wait_for_start", default_value="false"),
            # In the full OSCBF flow the plant owns /mujoco_joint_states, so
            # the replayed transition becomes a display-only stream there.
            DeclareLaunchArgument(
                "transition_replay_topic", default_value="/mujoco_joint_states"
            ),
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

            # 3. Transition planning server (persistent, multi-threaded)
            Node(
                package="robot_safecontrol_moveit",
                executable="transition_planning_server",
                name="transition_planning_server",
                output="screen",
                parameters=[
                    runtime_yaml,
                    {
                        "replay_joint_state_topic":
                            LaunchConfiguration("transition_replay_topic"),
                    },
                ],
                # pymoveit2 creates its state monitor on the hosting node;
                # keep that internal subscription on the same MuJoCo stream.
                remappings=[
                    ("joint_states", "/mujoco_joint_states"),
                ],
            ),

            # 4. OSCBF safe controller (JAX kernel, no MoveIt dependency).
            #    It consumes /mujoco_joint_states and publishes the safe
            #    state back onto the same stream after its JIT warm-up.
            Node(
                package="robot_safecontrol_moveit",
                executable="oscbf_controller",
                name="oscbf_controller",
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("start_oscbf_controller")
                ),
                parameters=[
                    str(share_dir / "config" / "oscbf_controller.yaml"),
                    {
                        "wait_for_start":
                            LaunchConfiguration("oscbf_wait_for_start"),
                    },
                ],
            ),

            # 4b. Jerk-limited actuator stand-in for the closed-loop demo.
            Node(
                package="robot_safecontrol_moveit",
                executable="oscbf_plant",
                name="oscbf_plant",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_oscbf_plant")),
                parameters=[
                    str(share_dir / "config" / "oscbf_plant.yaml"),
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
