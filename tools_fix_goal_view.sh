#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFG="$ROOT/limo_simple/config/polar_follow.yaml"
RVIZ="$ROOT/limo_simple/rviz/room207_vision.rviz"

cd ~/ros2_ws_simple
colcon build --packages-select limo_simple --symlink-install
source ~/ros2_ws_simple/install/setup.bash

echo "=== restart polar_follow ==="
pkill -f 'limo_simple.polar_follow|polar_follow.py' 2>/dev/null || true
sleep 1
nohup ros2 run limo_simple polar_follow --ros-args --params-file "$CFG" >/tmp/polar.log 2>&1 &
disown || true
sleep 3

echo "=== swap RViz to config with GoalMarker ==="
pkill -f rviz2 2>/dev/null || true
sleep 1
nohup rviz2 -d "$RVIZ" >/tmp/rviz_goal.log 2>&1 &
disown || true
sleep 4

echo "=== wait for goal ball / marker ==="
for i in $(seq 1 15); do
  if timeout 2 ros2 topic echo /control/goal --once 2>/dev/null | grep -q position; then
    echo "goal pose ok"
    break
  fi
  sleep 1
done

timeout 3 ros2 topic echo /control/goal --once || echo NO_GOAL
timeout 3 ros2 topic echo /vision/road_ok --once || true

# List models via gz topic if available
echo "=== gazebo models (look for polar_goal_ball) ==="
timeout 3 gz model --list 2>/dev/null || true
timeout 2 ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{}" 2>&1 | head -2 || true

# Check if ball entity exists by trying delete (non-destructive check via log)
python3 - <<'PY'
import time
import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
rclpy.init()
n = rclpy.create_node("check_ball")
# just see services
print("delete ready", n.create_client(DeleteEntity, "/delete_entity").wait_for_service(2))
print("spawn ready", n.create_client(SpawnEntity, "/spawn_entity").wait_for_service(2))
n.destroy_node(); rclpy.shutdown()
PY

echo "=== polar log ==="
tail -25 /tmp/polar.log
echo "DONE: look in Gazebo for a cyan ball ahead of the robot"
echo "In RViz: Fixed Frame=odom, displays GoalMarker + GoalPose enabled"
