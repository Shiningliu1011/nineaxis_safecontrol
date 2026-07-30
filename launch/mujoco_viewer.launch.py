"""Launch the MuJoCo observer for the project ninezzhou URDF model."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="robot_safecontrol_moveit",
                executable="mujoco_viewer",
                name="mujoco_joint_state_viewer",
                output="screen",
                parameters=[
                    {
                        "joint_state_topic": "/joint_states",
                        "show_target_path": True,
                        "show_obstacles": True,
                    }
                ],
            )
        ]
    )
