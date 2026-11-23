# ROS 2: ros2 launch huateng_camera_ros camera.launch.py [config:=lidar_sync] [config_path:=/path/to/file.yaml]
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _node(context):
    config = LaunchConfiguration("config").perform(context)
    config_path = LaunchConfiguration("config_path").perform(context)
    if not config_path:
        config_path = os.path.join(get_package_share_directory("huateng_camera_ros"), "config", config + ".yaml")
    return [
        Node(
            package="huateng_camera_ros",
            executable="camera_driver",
            name="camera_driver",
            output="screen",
            parameters=[{
                "config_path": config_path,
                "time_source": LaunchConfiguration("time_source").perform(context),
                "log_frames": int(LaunchConfiguration("log_frames").perform(context)),
            }],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value="camera", description="camera or lidar_sync"),
        DeclareLaunchArgument("config_path", default_value="", description="own config file (overrides config)"),
        DeclareLaunchArgument("time_source", default_value="", description="override: host / lidar"),
        DeclareLaunchArgument("log_frames", default_value="0", description="log raw values of the first N images"),
        OpaqueFunction(function=_node),
    ])
