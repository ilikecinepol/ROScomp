from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("port", default_value="15000"),
            DeclareLaunchArgument("topic", default_value="/cmd_vel"),
            Node(
                package="go2_webots_bridge",
                executable="cmd_vel_bridge",
                name="go2_webots_cmd_vel_bridge",
                output="screen",
                parameters=[
                    {
                        "host": LaunchConfiguration("host"),
                        "port": LaunchConfiguration("port"),
                        "topic": LaunchConfiguration("topic"),
                    }
                ],
            )
        ]
    )
