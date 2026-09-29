from setuptools import find_packages, setup


package_name = "go2_webots_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/bridge.launch.py",
                "launch/visualize.launch.py",
                "launch/ramp_climb.launch.py",
                "launch/slam_navigation.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/config",
            [
                "config/go2_sensors.rviz",
                "config/go2_navigation.rviz",
                "config/go2_nav2.yaml",
                "config/go2_slam.yaml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="True Tech Arena team",
    maintainer_email="local@example.com",
    description="ROS 2 velocity, camera and lidar UDP bridge for the Webots Unitree Go2.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "cmd_vel_bridge = go2_webots_bridge.cmd_vel_bridge:main",
            "goal_pose_relay = go2_webots_bridge.goal_pose_relay:main",
            "goal_map_gui = go2_webots_bridge.goal_map_gui:main",
            "goal_web_ui = go2_webots_bridge.goal_web_ui:main",
            "ramp_climber = go2_webots_bridge.ramp_climber:main",
        ]
    },
)
