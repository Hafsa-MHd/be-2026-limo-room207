#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
source "$HOME/ros2_ws_simple/install/setup.bash"

pkill -f 'room207_vision.launch' 2>/dev/null || true
pkill -f 'room207_limo.launch' 2>/dev/null || true
pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true
pkill -f rviz2 2>/dev/null || true
pkill -f road_vision 2>/dev/null || true
pkill -f robot_state_publisher 2>/dev/null || true
pkill -f joint_state_publisher 2>/dev/null || true
sleep 2

cd "$HOME/ros2_ws_simple"
colcon build --packages-select limo_simple --symlink-install
source install/setup.bash

export DISPLAY="${DISPLAY:-:0}"
ros2 launch limo_simple room207_vision.launch.py
