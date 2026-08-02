"""Launch the MuJoCo observer — Viewer only, no planning nodes.

This launch file starts ONLY the MuJoCo viewer. For the full demo
including MoveIt, planning server, and obstacle publisher, use::

    ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_dir = Path(
        get_package_share_directory("robot_safecontrol_moveit")
    ) / "config"
    params_file = str(config_dir / "mujoco_transition_runtime.yaml")

    return LaunchDescription(
        [
            Node(
                package="robot_safecontrol_moveit",
                executable="mujoco_viewer",
                name="mujoco_joint_state_viewer",
                output="screen",
                parameters=[params_file],
            )
        ]
    )
