#!/usr/bin/env bash
pkill -9 -f gzserver || true
pkill -9 -f gzclient || true
pkill -9 -f rviz2 || true
pkill -9 -f room207_drive || true
pkill -9 -f tools_run_multi || true
pkill -9 -f tools_rerun_live || true
pkill -9 -f tools_test_control || true
pkill -9 -f spawn_parking || true
pkill -9 -f polar_follow || true
pkill -9 -f road_vision || true
sleep 2
if pgrep -f gzserver >/dev/null; then
  echo still-gz
else
  echo clean
fi
