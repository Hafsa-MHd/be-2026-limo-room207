#!/usr/bin/env bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set -eo pipefail
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash

CFG="$ROOT/limo_simple/config"

pkill -f 'limo_simple.road_vision|limo_simple.lane_follow|road_vision.py|lane_follow.py' 2>/dev/null || true
sleep 1

nohup ros2 run limo_simple road_vision --ros-args --params-file "$CFG/road_vision.yaml" >/tmp/rv.log 2>&1 &
disown || true
nohup ros2 run limo_simple lane_follow --ros-args --params-file "$CFG/lane_follow.yaml" >/tmp/lf.log 2>&1 &
disown || true
sleep 4

echo "--- procs ---"
pgrep -af 'road_vision|lane_follow' || true
echo "--- topics ---"
ros2 topic list | grep vision || true
echo "--- black_bias ---"
timeout 5 ros2 topic echo /vision/black_bias --once || echo "NO_BIAS"
echo "--- road_ok ---"
timeout 5 ros2 topic echo /vision/road_ok --once || echo "NO_OK"
echo "--- rv log ---"
tail -20 /tmp/rv.log || true
