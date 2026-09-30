from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    package_share = Path(get_package_share_directory("go2_webots_bridge"))
    slam_share = Path(get_package_share_directory("slam_toolbox"))
    nav2_share = Path(get_package_share_directory("nav2_bringup"))

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(slam_share / "launch" / "online_async_launch.py")),
        launch_arguments={
            "use_sim_time": "False",
            "autostart": "True",
            "slam_params_file": str(package_share / "config" / "go2_slam.yaml"),
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2_share / "launch" / "navigation_launch.py")),
        launch_arguments={
            "use_sim_time": "False",
            "autostart": "True",
            "use_composition": "False",
            "use_respawn": "False",
            "params_file": str(package_share / "config" / "go2_nav2.yaml"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("port", default_value="15000"),
            Node(
                package="go2_webots_bridge",
                executable="cmd_vel_bridge",
                name="go2_webots_bridge",
                output="screen",
                parameters=[
                    {
                        "host": LaunchConfiguration("host"),
                        "port": LaunchConfiguration("port"),
                        "topic": "/cmd_vel",
                    }
                ],
            ),
            Node(
                package="go2_webots_bridge",
                executable="safety_cmd_mux",
                name="go2_safety_cmd_mux",
                output="screen",
                parameters=[{"observe_only": False, "dry_run": False}],
            ),
            slam,
            TimerAction(
                period=4.0,
                actions=[
                    GroupAction(
                        [
                            SetRemap(src="/cmd_vel", dst="/cmd_vel_nav"),
                            navigation,
                        ]
                    )
                ],
            ),
            TimerAction(
                period=7.0,
                actions=[
                    Node(
                        package="go2_webots_bridge",
                        executable="goal_web_ui",
                        name="go2_goal_web_ui",
                        output="screen",
                        respawn=True,
                        respawn_delay=2.0,
                    )
                ],
            ),
        ]
    )
