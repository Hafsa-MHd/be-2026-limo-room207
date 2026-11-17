#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

if ! pgrep -x gzserver >/dev/null; then
  echo "no gzserver — cannot show"
  exit 1
fi

killall -9 gzclient 2>/dev/null || true
sleep 1
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}"
# Prefer software GL if hardware black-screens on WSL
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
nohup gzclient --gui-client-plugin=libgazebo_ros_eol_gui.so >/tmp/limo_full/gzclient.log 2>&1 &
sleep 4
pgrep -af gzclient || echo FAIL
echo "GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"
echo "Check Gazebo window — should show track + limo at (-0.725,-0.087)"
timeout 3 ros2 topic echo /odom --once 2>/dev/null | sed -n '8,12p' || true
