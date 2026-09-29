#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /home/drmma/ros2_ws/install/setup.bash

windows_host="$(ip route show default | awk '{print $3; exit}')"
bridge_log=/tmp/go2_sensor_bridge.log
rm -f "$bridge_log"
ros2 launch go2_webots_bridge bridge.launch.py \
  host:="$windows_host" port:=15001 >"$bridge_log" 2>&1 &
bridge_pid=$!
cleanup() {
  kill -TERM "$bridge_pid" 2>/dev/null || true
  sleep 1
  kill -KILL "$bridge_pid" 2>/dev/null || true
  wait "$bridge_pid" 2>/dev/null || true
}
trap cleanup EXIT

sleep 2
python3 "/mnt/c/Users/drmma/Documents/ChatGPT/ROS2 Competition/webots/tools/check_sensor_topics.py"
cat "$bridge_log"
