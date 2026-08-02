"""Unified test launch: MuJoCo Viewer + static obstacle publisher.

This launch file does NOT start plan_transition — that is triggered manually
after the user has set a manual pose in the Viewer.

Usage::

    ros2 launch robot_safecontrol_moveit mujoco_transition_test.launch.py
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_dir = Path(
        get_package_share_directory("robot_safecontrol_moveit")
    ) / "config"

    viewer_params = str(config_dir / "mujoco_transition_test.yaml")
    obstacles_yaml = str(config_dir / "obstacles.yaml")

    return LaunchDescription(
        [
            Node(
                package="robot_safecontrol_moveit",
                executable="mujoco_viewer",
                name="mujoco_joint_state_viewer",
                output="screen",
                parameters=[viewer_params],
            ),
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
        ]
    )
