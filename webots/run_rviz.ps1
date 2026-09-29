$ErrorActionPreference = 'Stop'

$command = @'
source /opt/ros/jazzy/setup.bash
source /home/drmma/ros2_ws/install/setup.bash
pkill -TERM -x rviz2 2>/dev/null || true
if ros2 node list | grep -Eq '^/go2_webots_(cmd_vel_)?bridge$'; then
  rviz2 -d /home/drmma/ros2_ws/install/go2_webots_bridge/share/go2_webots_bridge/config/go2_sensors.rviz
else
  windows_host="$(ip route show default | awk '{print $3; exit}')"
  ros2 launch go2_webots_bridge visualize.launch.py host:="$windows_host"
fi
'@

wsl.exe -d Ubuntu-24.04 -- bash -lc $command
