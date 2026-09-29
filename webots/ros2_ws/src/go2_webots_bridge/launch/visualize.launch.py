from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config = Path(get_package_share_directory("go2_webots_bridge")) / "config" / "go2_sensors.rviz"
    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("port", default_value="15000"),
            DeclareLaunchArgument("topic", default_value="/cmd_vel"),
            Node(
                package="go2_webots_bridge",
                executable="cmd_vel_bridge",
                name="go2_webots_bridge",
                output="screen",
                parameters=[
                    {
                        "host": LaunchConfiguration("host"),
                        "port": LaunchConfiguration("port"),
                        "topic": LaunchConfiguration("topic"),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="go2_rviz",
                output="screen",
                arguments=["-d", str(config)],
            ),
        ]
    )
