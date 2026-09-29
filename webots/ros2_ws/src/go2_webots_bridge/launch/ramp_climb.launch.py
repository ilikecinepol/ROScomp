from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("port", default_value="15002"),
            Node(
                package="go2_webots_bridge",
                executable="cmd_vel_bridge",
                name="go2_webots_aframe_bridge",
                output="screen",
                parameters=[{"host": LaunchConfiguration("host"), "port": LaunchConfiguration("port")}],
            ),
            Node(
                package="go2_webots_bridge",
                executable="ramp_climber",
                output="screen",
                parameters=[
                    {
                        "telemetry_timeout": 3.0,
                        "fault_on_stale": False,
                        "approach_speed": 0.40,
                        "climb_speed": 0.45,
                        "descent_speed": 0.35,
                        "exit_speed": 0.35,
                    }
                ],
            ),
        ]
    )
