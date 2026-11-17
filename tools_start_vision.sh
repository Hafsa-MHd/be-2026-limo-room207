#!/usr/bin/env bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set -e
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

cd "$HOME/ros2_ws_simple"
colcon build --packages-select limo_simple --symlink-install
source "$HOME/ros2_ws_simple/install/setup.bash"

bash "$ROOT/tools_kill_sim.sh"

export DISPLAY="${DISPLAY:-:0}"
nohup ros2 launch limo_simple room207_vision.launch.py > /tmp/room207_vision.log 2>&1 &
echo "LAUNCH_PID=$!"

# Wait for spawn success or failure
for i in $(seq 1 90); do
  if grep -q "successfully spawned\|Spawn service failed\|Entity .* already exists" /tmp/room207_vision.log 2>/dev/null; then
    echo "SPAWN_LINE at ${i}s"
    break
  fi
  sleep 2
done

echo "==== nodes ===="
timeout 5 ros2 node list 2>/dev/null | sort -u || true
echo "==== joint pubs ===="
timeout 5 ros2 topic info /joint_states 2>/dev/null | head -8 || true
echo "==== wheel tf ===="
timeout 6 bash -c 'ros2 run tf2_ros tf2_echo base_link front_left_wheel_link' 2>&1 | head -20 || true
echo "==== camera ===="
timeout 8 ros2 topic hz /rgb/image_raw --window 5 2>&1 | head -8 || true
echo "==== spawn/log ===="
grep -E 'spawn|Spawn|Error|successfully|joint_state' /tmp/room207_vision.log | tail -30 || true
echo START_DONE
