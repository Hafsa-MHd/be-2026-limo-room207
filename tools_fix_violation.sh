#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Hard stop + clear mission
python3 - <<'PY'
import time
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

rclpy.init()
n = rclpy.create_node("fix_violation_stop")
p = n.create_publisher(Twist, "/cmd_vel", 10)
m = n.create_publisher(Bool, "/control/mission_enable", 10)
time.sleep(0.3)
p.publish(Twist())
b = Bool()
b.data = False
m.publish(b)
time.sleep(0.2)
n.destroy_node()
rclpy.shutdown()
print("cmd stopped, mission cleared")
PY

pkill -f 'limo_simple.road_vision|road_vision.py' 2>/dev/null || true
pkill -f 'limo_simple.lane_follow|lane_follow.py' 2>/dev/null || true
pkill -f 'limo_simple.polar_follow|polar_follow.py' 2>/dev/null || true
sleep 1

# Vision only (no lane_follow — it fights polar on /cmd_vel)
CFG="$ROOT/limo_simple/config"
pkill -f 'limo_simple.road_vision|road_vision.py|limo_simple.lane_follow|lane_follow.py' 2>/dev/null || true
sleep 1
nohup ros2 run limo_simple road_vision --ros-args --params-file "$CFG/road_vision.yaml" >/tmp/rv.log 2>&1 &
disown || true
sleep 3
bash "$ROOT/tools_restart_polar_only.sh"
sleep 1
bash "$ROOT/tools_spawn_correct_parking.sh"

echo "=== polar log ==="
tail -15 /tmp/polar.log || true
echo "=== vision road_ok ==="
timeout 5 ros2 topic echo /vision/road_ok --once || true
