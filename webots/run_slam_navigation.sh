#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

# A closed WSLg/RViz window can leave lifecycle nodes from the preceding run.
# Stop only this project's bridge/navigation processes before rebuilding.
pkill -TERM -f 'ros2 launch go2_webots_bridge slam_navigation.launch.py' 2>/dev/null || true
pkill -TERM -f '/go2_webots_bridge/' 2>/dev/null || true
pkill -TERM -f '/slam_toolbox/' 2>/dev/null || true
pkill -TERM -f '/nav2_' 2>/dev/null || true
pkill -TERM -x rviz2 2>/dev/null || true
sleep 1

cd /home/drmma/ros2_ws
colcon build --symlink-install --packages-select go2_webots_bridge
source install/setup.bash
set -u

windows_host="$(ip route show default | awk '{print $3; exit}')"

echo "Webots bridge: ${windows_host}:15000"
echo "Goal map: http://localhost:8765"
exec ros2 launch go2_webots_bridge slam_navigation.launch.py \
  host:="${windows_host}" port:=15000
