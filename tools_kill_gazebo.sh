#!/usr/bin/env bash
# Stop Gazebo + related LIMO / vision / control processes.
# Avoid matching this script name (contains "gazebo").
pkill -x gzserver 2>/dev/null || true
pkill -x gzclient 2>/dev/null || true
pkill -x gazebo 2>/dev/null || true
pkill -f '/gazebo |gazebo --|gzserver |gzclient ' 2>/dev/null || true
pkill -f 'limo_simple.road_vision|road_vision.py' 2>/dev/null || true
pkill -f 'limo_simple.lane_follow|lane_follow.py' 2>/dev/null || true
pkill -f 'limo_simple.polar_follow|polar_follow.py' 2>/dev/null || true
pkill -f 'rviz2' 2>/dev/null || true
pkill -f 'robot_state_publisher|joint_state_publisher' 2>/dev/null || true
pkill -f 'spawn_entity.py|ackermann_controller' 2>/dev/null || true
pkill -f 'mission_destinations|tools_run_mission' 2>/dev/null || true
pkill -f 'ros2 launch limo_simple|ros2 launch room207_limo' 2>/dev/null || true
sleep 1
fuser -k 11345/tcp 2>/dev/null || true
sleep 1
echo "=== remaining ==="
pgrep -af 'gzserver|gzclient|road_vision|polar_follow|lane_follow|rviz2' || echo "all stopped"
