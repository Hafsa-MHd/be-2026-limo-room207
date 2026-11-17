#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFG_V="$ROOT/limo_simple/config/road_vision.yaml"
CFG_P="$ROOT/limo_simple/config/polar_follow.yaml"

echo "=== procs before ==="
pgrep -af gzserver || echo no_gz
pgrep -af rviz2 || echo no_rviz
pgrep -af road_vision || echo no_vision
pgrep -af polar_follow || echo no_polar

# Start drive launch if gazebo is down
if ! pgrep -x gzserver >/dev/null 2>&1; then
  echo "=== launching room207_drive (polar) ==="
  nohup ros2 launch limo_simple room207_drive.launch.py control:=polar \
    >/tmp/drive_polar.log 2>&1 &
  disown || true
  # wait for spawn + vision
  for i in $(seq 1 40); do
    if ros2 topic list 2>/dev/null | grep -q /rgb/image_raw; then
      echo "camera up after ${i}s"
      break
    fi
    sleep 1
  done
  sleep 8
else
  echo "=== gazebo already up; ensure vision+polar ==="
  if ! pgrep -af road_vision | grep -v pgrep >/dev/null; then
    nohup ros2 run limo_simple road_vision --ros-args --params-file "$CFG_V" >/tmp/rv.log 2>&1 &
    disown || true
  fi
  pkill -f 'limo_simple.polar_follow|polar_follow.py' 2>/dev/null || true
  sleep 1
  nohup ros2 run limo_simple polar_follow --ros-args --params-file "$CFG_P" >/tmp/polar.log 2>&1 &
  disown || true
  sleep 3
fi

# Put robot on road so road_ok becomes true and marker publishes
python3 "$ROOT/tools_respawn_on_road.py" 2>&1 | tee /tmp/respawn_marker.txt | tail -15

echo "=== marker topics ==="
ros2 topic list | grep control || true

echo "=== echo goal ==="
timeout 5 ros2 topic echo /control/goal --once || echo NO_GOAL

echo "=== echo marker (first lines) ==="
timeout 5 ros2 topic echo /control/goal_marker --once 2>&1 | head -60 || echo NO_MARKER

echo "=== road_ok / cmd ==="
timeout 4 ros2 topic echo /vision/road_ok --once || true
timeout 4 ros2 topic echo /cmd_vel --once || true

echo "=== polar log ==="
tail -15 /tmp/polar.log 2>/dev/null || tail -15 /tmp/drive_polar.log 2>/dev/null || true
