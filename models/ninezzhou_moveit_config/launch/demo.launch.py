"""Bring up MoveIt 2 and a non-physical mock ros2_control system.

Replace the mock hardware plugin in ``ninezzhou.ros2_control.xacro`` before
connecting this configuration to a physical robot.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    moveit_config = (
        MoveItConfigsBuilder("ninezzhou", package_name="ninezzhou_moveit_config")
        .robot_description(file_path="config/ninezzhou.urdf.xacro")
        .robot_description_semantic(file_path="config/ninezzhou.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], load_all=False)
        .to_moveit_configs()
    )
    package_share = Path(get_package_share_directory("ninezzhou_moveit_config"))

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[str(package_share / "config" / "ros2_controllers.yaml")],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_trajectory_controller", "-c", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            ros2_control,
            joint_state_broadcaster,
            arm_controller,
            move_group,
        ]
    )
