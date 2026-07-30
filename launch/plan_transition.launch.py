"""Launch the parameterised pipeline after MoveIt bringup is running."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("robot_safecontrol_moveit"))
    return LaunchDescription(
        [
            Node(
                package="robot_safecontrol_moveit",
                executable="plan_transition",
                name="plan_transition",
                output="screen",
                parameters=[str(package_share / "config" / "plan_transition.yaml")],
            )
        ]
    )
