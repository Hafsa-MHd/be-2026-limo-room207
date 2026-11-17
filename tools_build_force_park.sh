#!/usr/bin/env bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set -eo pipefail
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws_simple
colcon build --packages-select limo_simple --symlink-install
source ~/ros2_ws_simple/install/setup.bash
bash "$ROOT/tools_force_parking_now.sh"
python3 - <<'PY'
import time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

PARK = (-0.725347, -0.086613)
SCHOOL = (-0.725347, -0.086613)
rclpy.init()
n = rclpy.create_node("chk")
st = {"o": None}
n.create_subscription(Odometry, "/odom", lambda m: st.__setitem__("o", m), 10)
c = n.create_publisher(Twist, "/cmd_vel", 10)
e = n.create_publisher(Bool, "/control/mission_enable", 10)
t0 = time.time()
while time.time() - t0 < 5:
    rclpy.spin_once(n, timeout_sec=0.05)
    c.publish(Twist())
    b = Bool()
    b.data = False
    e.publish(b)
if st["o"] is None:
    print("NO_ODOM")
else:
    p = st["o"].pose.pose.position
    ep = ((p.x - PARK[0]) ** 2 + (p.y - PARK[1]) ** 2) ** 0.5
    es = ((p.x - SCHOOL[0]) ** 2 + (p.y - SCHOOL[1]) ** 2) ** 0.5
    print("odom=(%.3f,%.3f) d_parking=%.3f d_school=%.3f" % (p.x, p.y, ep, es))
    if ep < 0.4:
        label = "GITLAB_ROOM207_SPAWN"
    else:
        label = "OTHER"
    print("START_POSE", label)
n.destroy_node()
rclpy.shutdown()
PY
