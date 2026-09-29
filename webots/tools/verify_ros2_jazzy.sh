#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /home/drmma/ros2_ws/install/setup.bash
set -u

echo "ROS_DISTRO=$ROS_DISTRO"
python3 - <<'PY'
import cv2
import numpy
import rclpy

print(f"rclpy=OK cv2={cv2.__version__} numpy={numpy.__version__}")
PY

echo "webots_ros2_driver=$(ros2 pkg prefix webots_ros2_driver)"
echo "go2_webots_bridge=$(ros2 pkg prefix go2_webots_bridge)"
ros2 pkg executables go2_webots_bridge
dpkg-query -W ros-jazzy-desktop ros-jazzy-webots-ros2
