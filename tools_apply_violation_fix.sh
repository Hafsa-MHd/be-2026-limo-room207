#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFG="$ROOT/limo_simple/config"

cd ~/ros2_ws_simple
colcon build --packages-select limo_simple --symlink-install
source ~/ros2_ws_simple/install/setup.bash

# Stop everything that can publish cmd_vel
python3 - <<'PY'
import time
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
rclpy.init()
n = rclpy.create_node("stop_all")
p = n.create_publisher(Twist, "/cmd_vel", 10)
m = n.create_publisher(Bool, "/control/mission_enable", 10)
time.sleep(0.3)
for _ in range(5):
    p.publish(Twist())
    b = Bool(); b.data = False; m.publish(b)
    time.sleep(0.05)
n.destroy_node(); rclpy.shutdown()
print("stopped")
PY

pkill -f 'limo_simple.lane_follow|lane_follow.py' 2>/dev/null || true
pkill -f 'limo_simple.polar_follow|polar_follow.py' 2>/dev/null || true
pkill -f 'limo_simple.road_vision|road_vision.py' 2>/dev/null || true
sleep 1

nohup ros2 run limo_simple road_vision --ros-args --params-file "$CFG/road_vision.yaml" >/tmp/rv.log 2>&1 &
disown || true
sleep 3
bash "$ROOT/tools_restart_polar_only.sh"
sleep 1
bash "$ROOT/tools_spawn_correct_parking.sh"

echo "=== LIVE CHECK ==="
python3 - <<'PY'
import time
import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

TEX = "/home/ha054/ros2_ws/src/environment/qcar_large_track/models/qcar_large_track/materials/textures/SDCS_MapLayout.png"
SX, SY = 4.88, 6.2
PARK = (-1.15, 1.45)

rclpy.init()
n = rclpy.create_node("live_chk")
st = {"o": None, "ok": None}
n.create_subscription(Odometry, "/odom", lambda m: st.__setitem__("o", m), 10)
n.create_subscription(Bool, "/vision/road_ok", lambda m: st.__setitem__("ok", m.data), 10)
pub = n.create_publisher(Twist, "/cmd_vel", 10)
t0 = time.time()
while time.time() - t0 < 4:
    rclpy.spin_once(n, timeout_sec=0.05)
    pub.publish(Twist())

img = cv2.imread(TEX, 0)
h, w = img.shape
bright = (img > 160).astype(np.uint8)
safe = (bright == 0).astype(np.uint8) * 255
dist = cv2.distanceTransform(safe, cv2.DIST_L2, 5)
mpp = SX / (w - 1)

o = st["o"]
if o is None:
    print("NO_ODOM")
else:
    p = o.pose.pose.position
    col = int(round((p.x / SX + 0.5) * (w - 1)))
    row = int(round((0.5 - p.y / SY) * (h - 1)))
    g = float(np.median(img[row-2:row+3, col-2:col+3]))
    dm = float(dist[row, col]) * mpp
    err = ((p.x - PARK[0])**2 + (p.y - PARK[1])**2) ** 0.5
    print("odom=(%.3f,%.3f) err_to_parking=%.3fm" % (p.x, p.y, err))
    print("map gray=%.1f margin_to_white=%.2fm map_ok=%s" % (g, dm, g <= 90 and dm >= 0.12))
print("road_ok", st["ok"])
n.destroy_node(); rclpy.shutdown()
PY

echo "=== polar log ==="
tail -12 /tmp/polar.log || true
