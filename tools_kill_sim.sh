#!/usr/bin/env bash
pkill -x gzserver 2>/dev/null || true
pkill -x gzclient 2>/dev/null || true
pkill -x rviz2 2>/dev/null || true
pkill -f '/limo_simple/.*road_vision' 2>/dev/null || true
pkill -f 'robot_state_publisher' 2>/dev/null || true
pkill -f 'joint_state_publisher' 2>/dev/null || true
pkill -f 'spawn_entity.py' 2>/dev/null || true
sleep 2
echo KILLED
