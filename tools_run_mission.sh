#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFG="$ROOT/limo_simple/config/polar_follow.yaml"

cd ~/ros2_ws_simple
colcon build --packages-select limo_simple --symlink-install
source ~/ros2_ws_simple/install/setup.bash

# Ensure sim + vision + polar
if ! pgrep -x gzserver >/dev/null 2>&1; then
  echo "Gazebo not running — start room207_drive first"
  nohup ros2 launch limo_simple room207_drive.launch.py control:=polar >/tmp/drive_polar.log 2>&1 &
  disown || true
  for i in $(seq 1 45); do
    if timeout 1 ros2 topic hz /rgb/image_raw 2>&1 | grep -q average; then
      echo "camera up"
      break
    fi
    sleep 1
  done
  sleep 10
fi

pkill -f 'limo_simple.polar_follow|polar_follow.py' 2>/dev/null || true
sleep 1
# Ensure vision
if ! pgrep -af road_vision | grep -v pgrep >/dev/null; then
  nohup ros2 run limo_simple road_vision --ros-args --params-file "$ROOT/limo_simple/config/road_vision.yaml" >/tmp/rv.log 2>&1 &
  disown || true
  sleep 3
fi
nohup ros2 run limo_simple polar_follow --ros-args --params-file "$CFG" >/tmp/polar.log 2>&1 &
disown || true
sleep 3

sed -i 's/\r$//' "$ROOT/tools_mission_destinations.py"
python3 "$ROOT/tools_mission_destinations.py" | tee /tmp/mission_results.txt
echo "results saved /tmp/mission_results.txt"
