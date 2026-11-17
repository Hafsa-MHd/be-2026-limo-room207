#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd ~/ros2_ws_simple
colcon build --packages-select limo_simple --symlink-install >/tmp/limo_full/build.log 2>&1
source ~/ros2_ws_simple/install/setup.bash

bash "$ROOT/tools_force_parking_now.sh" || true
sleep 2

pkill -9 -f 'polar_follow|road_vision' 2>/dev/null || true
sleep 1

nohup ros2 run limo_simple road_vision --ros-args --params-file "$ROOT/limo_simple/config/road_vision.yaml" >/tmp/limo_full/vision.log 2>&1 &
nohup ros2 run limo_simple polar_follow --ros-args --params-file "$ROOT/limo_simple/config/polar_follow.yaml" >/tmp/limo_full/polar.log 2>&1 &
sleep 4

echo "=== cmd_vel publishers ==="
ros2 topic info /cmd_vel 2>/dev/null | head -20 || true
echo "=== perception ==="
timeout 3 ros2 topic echo /vision/road_ok --once || true
echo "=== control test ==="
PYTHONUNBUFFERED=1 python3 -u "$ROOT/tools_test_control_once.py"
