#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /home/drmma/ros2_ws/install/setup.bash

windows_host="$(ip route show default | awk '{print $3; exit}')"
echo "WINDOWS_HOST=$windows_host"

bridge_log=/tmp/go2_webots_bridge.log
rm -f "$bridge_log"
ros2 launch go2_webots_bridge bridge.launch.py host:="$windows_host" >"$bridge_log" 2>&1 &
bridge_pid=$!
cleanup() {
  kill -TERM "$bridge_pid" 2>/dev/null || true
  sleep 1
  kill -KILL "$bridge_pid" 2>/dev/null || true
  wait "$bridge_pid" 2>/dev/null || true
}
trap cleanup EXIT

sleep 2
timeout 3 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.25, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.35}}' || true
sleep 1

ros2 node list
cat "$bridge_log"
