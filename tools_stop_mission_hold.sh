#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash

echo "=== who publishes mission? ==="
timeout 3 ros2 topic info /control/mission_goal -v 2>/dev/null | head -40 || true

# Kill mission runners
pkill -f 'tools_run_mission|tools_mission|run_mission|mission_destinations' 2>/dev/null || true
pkill -f 'tmp_check_mission|tmp_try_detect' 2>/dev/null || true

python3 - <<'PY'
import time
import rclpy
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import DeleteEntity
from std_msgs.msg import Bool

rclpy.init()
n = rclpy.create_node("hold_parking")
cmd = n.create_publisher(Twist, "/cmd_vel", 10)
en = n.create_publisher(Bool, "/control/mission_enable", 10)
cli = n.create_client(DeleteEntity, "/delete_entity")
cli.wait_for_service(5)
req = DeleteEntity.Request(); req.name = "polar_goal_ball"
fut = cli.call_async(req)
rclpy.spin_until_future_complete(n, fut, timeout_sec=5)
print("ball", fut.result())

b = Bool(); b.data = False
for _ in range(20):
    cmd.publish(Twist())
    en.publish(b)
    rclpy.spin_once(n, timeout_sec=0.02)
    time.sleep(0.05)
print("mission disabled, cmd zeroed")
n.destroy_node(); rclpy.shutdown()
PY

echo "=== odom ==="
timeout 3 ros2 topic echo /odom --once 2>/dev/null | sed -n '1,12p' || true
