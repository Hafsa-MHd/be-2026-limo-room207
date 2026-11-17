#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
source "$HOME/ros2_ws_simple/install/setup.bash"

pkill -f 'room207_vision.launch' 2>/dev/null || true
pkill -f 'room207_limo.launch' 2>/dev/null || true
pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true
sleep 2

export DISPLAY="${DISPLAY:-:0}"
mkdir -p "$HOME/ros2_ws_simple/captures" /tmp
nohup ros2 launch limo_simple room207_vision.launch.py \
  > /tmp/room207_vision.log 2>&1 &
echo "LAUNCH_PID=$!"

for i in $(seq 1 60); do
  if timeout 2 ros2 topic list 2>/dev/null | grep -q '/rgb/image_raw'; then
    echo "CAMERA_UP at ${i}s"
    break
  fi
  if grep -q "No module named\|Caught exception" /tmp/room207_vision.log 2>/dev/null; then
    echo "EARLY_ERROR at ${i}s"
    break
  fi
  sleep 2
done

echo "==== topics ===="
timeout 5 ros2 topic list 2>/dev/null | grep -E 'rgb|vision|cmd_vel' || true
echo "==== hz image ===="
timeout 10 ros2 topic hz /rgb/image_raw --window 5 2>&1 | head -20 || true
echo "==== hz panel ===="
timeout 10 ros2 topic hz /vision/panel --window 5 2>&1 | head -20 || true
echo "==== road_ok ===="
timeout 8 ros2 topic echo /vision/road_ok --once 2>&1 | head -20 || true
echo "==== center ===="
timeout 8 ros2 topic echo /vision/center --once 2>&1 | head -30 || true
echo "==== capture ===="
ls -la "$HOME/ros2_ws_simple/captures/" 2>&1 || true
echo "==== log tail ===="
tail -80 /tmp/room207_vision.log || true
echo SMOKE_DONE
