#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFG="$ROOT/limo_simple/config/polar_follow.yaml"

echo "=== unit test ==="
sed -i 's/\r$//' "$ROOT/tools_test_polar.py"
python3 "$ROOT/tools_test_polar.py"

echo "=== build ==="
cd ~/ros2_ws_simple
colcon build --packages-select limo_simple --symlink-install
source ~/ros2_ws_simple/install/setup.bash

echo "=== swap controller (keep road_vision) ==="
pkill -f 'limo_simple.lane_follow|lane_follow.py|limo_simple.polar_follow|polar_follow.py' 2>/dev/null || true
sleep 1
# ensure vision still up
if ! pgrep -af 'road_vision' | grep -v pgrep >/dev/null; then
  nohup ros2 run limo_simple road_vision --ros-args --params-file "$ROOT/limo_simple/config/road_vision.yaml" >/tmp/rv.log 2>&1 &
  disown || true
  sleep 3
fi
nohup ros2 run limo_simple polar_follow --ros-args --params-file "$CFG" >/tmp/polar.log 2>&1 &
disown || true
sleep 4

echo "=== procs ==="
pgrep -af 'road_vision|polar_follow|lane_follow' || true
echo "=== topics ==="
ros2 topic list | grep -E 'vision|control|cmd_vel' || true
echo "=== samples ==="
timeout 4 ros2 topic echo /control/rho --once || echo NO_RHO
timeout 4 ros2 topic echo /control/alpha --once || echo NO_ALPHA
timeout 4 ros2 topic echo /cmd_vel --once || echo NO_CMD
timeout 4 ros2 topic echo /vision/road_ok --once || echo NO_OK
echo "=== polar log ==="
tail -20 /tmp/polar.log || true
