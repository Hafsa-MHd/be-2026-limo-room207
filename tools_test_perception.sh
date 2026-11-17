#!/usr/bin/env bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set -e
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

# Clean previous runs (Gazebo master on 11345 often survives pkill)
bash "$ROOT/tools_kill_gazebo.sh"
pkill -x rviz2 2>/dev/null || true
pkill -f 'spawn_entity.py' 2>/dev/null || true
pkill -f 'joint_state_publisher' 2>/dev/null || true
pkill -f 'robot_state_publisher' 2>/dev/null || true
pkill -f 'lib/limo_simple/road_vision' 2>/dev/null || true
pkill -f 'room207_limo.launch' 2>/dev/null || true
pkill -f 'room207_vision.launch' 2>/dev/null || true
sleep 2

cd "$HOME/ros2_ws_simple"
colcon build --packages-select limo_simple --symlink-install
source "$HOME/ros2_ws_simple/install/setup.bash"

rm -f "$HOME/ros2_ws_simple/captures/road_vision.png"
mkdir -p "$HOME/ros2_ws_simple/captures" /tmp
export DISPLAY="${DISPLAY:-:0}"

nohup ros2 launch limo_simple room207_vision.launch.py > /tmp/perception_test.log 2>&1 &
echo "LAUNCH_PID=$!"

CAMERA_OK=0
VISION_OK=0
SPAWN_OK=0
for i in $(seq 1 90); do
  if grep -qE 'successfully spawned|Entity .* already' /tmp/perception_test.log 2>/dev/null; then
    SPAWN_OK=1
    echo "SPAWN_OK at ${i}s"
    break
  fi
  if grep -q 'gzserver-1.: process has died\|Spawn service failed' /tmp/perception_test.log 2>/dev/null; then
    echo "GAZEBO_OR_SPAWN_FAIL at ${i}s"
    break
  fi
  sleep 2
done

echo "==== camera hz ===="
CAM_HZ=$(timeout 12 ros2 topic hz /rgb/image_raw --window 5 2>&1 | tee /tmp/cam_hz.txt | head -12 || true)
cat /tmp/cam_hz.txt | head -12 || true
if grep -q 'average rate' /tmp/cam_hz.txt 2>/dev/null; then
  CAMERA_OK=1
fi

echo "==== vision hz ===="
timeout 12 ros2 topic hz /vision/panel --window 5 2>&1 | tee /tmp/vis_hz.txt | head -12 || true
if grep -q 'average rate' /tmp/vis_hz.txt 2>/dev/null; then
  VISION_OK=1
fi

echo "==== road_ok ===="
timeout 8 ros2 topic echo /vision/road_ok --once 2>&1 | head -15 || true
echo "==== center ===="
timeout 8 ros2 topic echo /vision/center --once 2>&1 | head -25 || true
echo "==== cmd_vel pubs (must be 0 from perception) ===="
timeout 5 ros2 topic info /cmd_vel 2>&1 | head -15 || true
echo "==== capture ===="
# wait a bit for save_once
sleep 5
ls -la "$HOME/ros2_ws_simple/captures/" 2>&1 || true
if [ -f "$HOME/ros2_ws_simple/captures/road_vision.png" ]; then
  cp -f "$HOME/ros2_ws_simple/captures/road_vision.png" \
    "$ROOT/captures_road_vision.png"
  echo COPIED_CAPTURE
fi
echo "==== log hints ===="
strings /tmp/perception_test.log | grep -E 'successfully spawned|Spawn service failed|road_vision on|saved |process has died|ERROR|Caught exception' | tail -40 || true
echo "RESULT spawn=$SPAWN_OK camera=$CAMERA_OK vision=$VISION_OK"
echo PERCEPTION_TEST_DONE

