#!/usr/bin/env bash
kill -9 39246 39252 52097 52100 56292 56294 2>/dev/null || true
pkill -9 -f gzserver || true
pkill -9 -f gzclient || true
pkill -9 -f gazebo || true
pkill -9 -f rviz2 || true
pkill -9 -f road_vision || true
pkill -9 -f polar_follow || true
pkill -9 -f lane_follow || true
pkill -9 -f ackermann_controller || true
pkill -9 -f robot_state_publisher || true
pkill -9 -f joint_state_publisher || true
sleep 1
fuser -k 11345/tcp 2>/dev/null || true
echo done
pgrep -af gzserver || echo no_gzserver
pgrep -af rviz2 || echo no_rviz
pgrep -af road_vision || echo no_vision
pgrep -af polar_follow || echo no_polar
