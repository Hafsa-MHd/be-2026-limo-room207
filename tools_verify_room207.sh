#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

pkill -x gzserver 2>/dev/null || true
pkill -x gzclient 2>/dev/null || true
pkill -x rviz2 2>/dev/null || true
pkill -f 'spawn_entity.py' 2>/dev/null || true
sleep 2

export DISPLAY="${DISPLAY:-:0}"
nohup ros2 launch room207_limo room207_limo.launch.py > /tmp/room207_school.log 2>&1 &
echo "LAUNCH_PID=$!"

for i in $(seq 1 90); do
  if grep -qE 'successfully spawned|Spawn service failed|Entity .* already' /tmp/room207_school.log 2>/dev/null; then
    echo "SPAWN_EVENT at ${i}s"
    break
  fi
  sleep 2
done

sleep 5
echo "==== nodes ===="
timeout 5 ros2 node list 2>/dev/null | sort -u || true
echo "==== camera ===="
timeout 10 ros2 topic hz /rgb/image_raw --window 5 2>&1 | head -12 || true
echo "==== scan ===="
timeout 8 ros2 topic hz /scan --window 5 2>&1 | head -8 || true
echo "==== odom ===="
timeout 5 ros2 topic echo /odom --once 2>&1 | head -20 || true
echo "==== spawn lines ===="
strings /tmp/room207_school.log | grep -E 'successfully spawned|Spawn service failed|Entity|ERROR|Error|ensem_room207|world' | tail -40 || true
echo VERIFY_DONE
