from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("source_manifest", description="已核验场景来源 manifest 的绝对路径"),
        DeclareLaunchArgument("shared_path", default_value="/dev/shm/robot_safecontrol_collision_scene"),
        DeclareLaunchArgument("lidar_source_topic", default_value="/livox/lidar"),
        DeclareLaunchArgument("lidar_source_clock", description="已核验的 LiDAR 原始逐点时间域"),
        DeclareLaunchArgument("calibration_record", description="manifest 使用的同一 CalibrationRecord 绝对路径"),
        Node(package="robot_safecontrol_moveit", executable="perception_bridge", output="screen",
             parameters=[{"scene_server_mode": True, "source_topic": "",
                          "source_topic_lidar": LaunchConfiguration("lidar_source_topic"),
                          "source_time_domain_lidar": LaunchConfiguration("lidar_source_clock"),
                          "input_frame_lidar": "livox_frame", "world_frame": "base_link",
                          "calibration_record_path": LaunchConfiguration("calibration_record")}]),
        Node(package="robot_safecontrol_moveit", executable="collision_scene_server", output="screen",
             parameters=[PathJoinSubstitution([FindPackageShare("robot_safecontrol_moveit"), "config", "collision_scene_server.yaml"]),
                         {"source_manifest": LaunchConfiguration("source_manifest"),
                          "shared_path": LaunchConfiguration("shared_path")}]),
    ])
