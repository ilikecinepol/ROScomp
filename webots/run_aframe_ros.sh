#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /home/drmma/ros2_ws/install/setup.bash

windows_host="$(ip route show default | awk '{print $3; exit}')"
echo "ROS 2 -> Webots at ${windows_host}:15002"
echo "Press Ctrl+C to stop the bridge and the ramp controller."

exec ros2 launch go2_webots_bridge ramp_climb.launch.py \
  host:="$windows_host" \
  port:=15002
