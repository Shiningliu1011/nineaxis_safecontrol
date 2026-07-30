"""ROS 2 visualisation for the description package."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("ninezzhou"))
    robot_description = (package_share / "urdf" / "ninezzhou.urdf").read_text()
    return LaunchDescription(
        [
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(package="rviz2", executable="rviz2", output="screen"),
        ]
    )
