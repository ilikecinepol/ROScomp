#!/usr/bin/env bash
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y curl ca-certificates locales

locale-gen en_US en_US.UTF-8
update-locale LANG=en_US.UTF-8

ros_source_deb=/tmp/ros2-apt-source_1.3.0.noble_all.deb
if [[ ! -s "$ros_source_deb" ]]; then
  curl -fL -o "$ros_source_deb" \
    https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.noble_all.deb
fi
dpkg -i "$ros_source_deb"

apt-get update
apt-get install -y \
  ros-jazzy-desktop \
  ros-jazzy-webots-ros2 \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep

if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi
su - drmma -c 'rosdep update'

install -d -o drmma -g drmma /home/drmma/ros2_ws/src
bridge_source='/mnt/c/Users/drmma/Documents/ChatGPT/ROS2 Competition/webots/ros2_ws/src/go2_webots_bridge'
bridge_link=/home/drmma/ros2_ws/src/go2_webots_bridge
if [[ -L "$bridge_link" || -e "$bridge_link" ]]; then
  rm -rf "$bridge_link"
fi
ln -s "$bridge_source" "$bridge_link"
chown -h drmma:drmma "$bridge_link"

setup_line='source /opt/ros/jazzy/setup.bash'
grep -qxF "$setup_line" /home/drmma/.bashrc || echo "$setup_line" >> /home/drmma/.bashrc
workspace_line='[[ -f ~/ros2_ws/install/setup.bash ]] && source ~/ros2_ws/install/setup.bash'
grep -qxF "$workspace_line" /home/drmma/.bashrc || echo "$workspace_line" >> /home/drmma/.bashrc
chown drmma:drmma /home/drmma/.bashrc
chown -R drmma:drmma /home/drmma/ros2_ws /home/drmma/.ros

runuser -u drmma -- bash -lc 'source /opt/ros/jazzy/setup.bash && cd /home/drmma/ros2_ws && colcon build --symlink-install'

echo 'ROS 2 Jazzy and webots_ros2 installation completed.'
