#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash

pkill -f 'limo_simple.lane_follow|lane_follow.py' 2>/dev/null || true
sleep 0.3

echo "=== procs ==="
pgrep -af 'polar_follow|road_vision|lane_follow' || echo none

echo "=== odom / road_ok / cmd ==="
python3 - <<'PY'
import time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

rclpy.init()
n = rclpy.create_node("live_status")
st = {"o": None, "ok": None, "cmd": None}
n.create_subscription(Odometry, "/odom", lambda m: st.__setitem__("o", m), 10)
n.create_subscription(Bool, "/vision/road_ok", lambda m: st.__setitem__("ok", m.data), 10)
n.create_subscription(Twist, "/cmd_vel", lambda m: st.__setitem__("cmd", m), 10)
pub = n.create_publisher(Twist, "/cmd_vel", 10)
t0 = time.time()
while time.time() - t0 < 2.5:
    rclpy.spin_once(n, timeout_sec=0.05)
# hard stop
pub.publish(Twist())
time.sleep(0.1)
o = st["o"]
if o is None:
    print("NO_ODOM")
else:
    p = o.pose.pose.position
    print("odom=(%.3f,%.3f,z=%.3f)" % (p.x, p.y, p.z))
print("road_ok", st["ok"])
c = st["cmd"]
if c is None:
    print("cmd=None")
else:
    print("cmd vx=%.3f wz=%.3f" % (c.linear.x, c.angular.z))
n.destroy_node()
rclpy.shutdown()
PY
