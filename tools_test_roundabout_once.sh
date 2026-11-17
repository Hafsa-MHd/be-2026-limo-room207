#!/usr/bin/env bash
# Quick retest: parking → NE roundabout showcase goal.
set -eo pipefail
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
sed -i 's/\r$//' "$ROOT"/tools_*.sh 2>/dev/null || true

pkill -f 'tools_test_control_once' 2>/dev/null || true
pkill -f 'limo_simple/polar_follow' 2>/dev/null || true
pkill -f 'limo_simple/road_vision' 2>/dev/null || true
sleep 1

if ! timeout 4 ros2 topic echo /odom --once >/dev/null 2>&1; then
  echo "ERROR: no /odom — run tools_rerun_live_demo.sh first"
  exit 1
fi

nohup ros2 run limo_simple road_vision --ros-args \
  --params-file "$ROOT/limo_simple/config/road_vision.yaml" \
  -p use_sim_time:=true >/tmp/limo_full/vision.log 2>&1 &
nohup ros2 run limo_simple polar_follow --ros-args \
  --params-file "$ROOT/limo_simple/config/polar_follow.yaml" \
  -p d8_south_route:=true -p use_sim_time:=true >/tmp/limo_full/map-planner-controller.log 2>&1 &
sleep 4

bash "$ROOT/tools_force_parking_now.sh"
sleep 1
PYTHONUNBUFFERED=1 python3 -u "$ROOT/tools_test_control_once.py" \
  --goal 2.100 2.110 --timeout 175
