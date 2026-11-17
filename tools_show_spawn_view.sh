#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash
OUT="/home/ha054/ros2_ws_simple/captures"
mkdir -p "$OUT"

python3 - <<'PY'
import time
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

OUT = "/home/ha054/ros2_ws_simple/captures"
PARK = (-0.725347, -0.086613)

rclpy.init()
n = rclpy.create_node("show_spawn_view")
bridge = CvBridge()
st = {"img": None, "odom": None, "ok": None, "panel": None}
n.create_subscription(Image, "/rgb/image_raw", lambda m: st.__setitem__("img", m), 10)
n.create_subscription(Image, "/vision/panel", lambda m: st.__setitem__("panel", m), 10)
n.create_subscription(Odometry, "/odom", lambda m: st.__setitem__("odom", m), 10)
n.create_subscription(Bool, "/vision/road_ok", lambda m: st.__setitem__("ok", m.data), 10)
c = n.create_publisher(Twist, "/cmd_vel", 10)
e = n.create_publisher(Bool, "/control/mission_enable", 10)

t0 = time.time()
while time.time() - t0 < 8:
    rclpy.spin_once(n, timeout_sec=0.05)
    c.publish(Twist())
    b = Bool(); b.data = False; e.publish(b)

# Map overlay with pose
tex = (
    "/home/ha054/ros2_ws/src/environment/qcar_large_track/models/"
    "qcar_large_track/materials/textures/SDCS_MapLayout.png"
)
SX, SY = 4.88, 6.2
mp = cv2.imread(tex)
h, w = mp.shape[:2]
vis = mp.copy()

def mark(x, y, color, label):
    c = int(round((x / SX + 0.5) * (w - 1)))
    r = int(round((0.5 - y / SY) * (h - 1)))
    cv2.circle(vis, (c, r), 18, color, 3)
    cv2.putText(vis, label, (c + 20, r), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

mark(PARK[0], PARK[1], (0, 0, 255), "GITLAB_SPAWN")
if st["odom"] is not None:
    p = st["odom"].pose.pose.position
    mark(p.x, p.y, (0, 255, 0), "ROBOT")
    print("odom=(%.3f,%.3f)" % (p.x, p.y))
print("road_ok", st["ok"])

map_path = OUT + "/gitlab_spawn_map.png"
cv2.imwrite(map_path, vis)
print("saved", map_path)

if st["img"] is not None:
    img = bridge.imgmsg_to_cv2(st["img"], "bgr8")
    cam_path = OUT + "/gitlab_spawn_camera.png"
    cv2.imwrite(cam_path, img)
    print("saved", cam_path)
else:
    print("NO_CAMERA")

if st["panel"] is not None:
    panel = bridge.imgmsg_to_cv2(st["panel"], "bgr8")
    panel_path = OUT + "/gitlab_spawn_vision.png"
    cv2.imwrite(panel_path, panel)
    print("saved", panel_path)

# Also copy to Windows Downloads for easy open
import shutil
for name in ("gitlab_spawn_map.png", "gitlab_spawn_camera.png", "gitlab_spawn_vision.png"):
    src = OUT + "/" + name
    try:
        shutil.copy(src, "/mnt/c/Users/ha054/Downloads/" + name)
        print("copied to Downloads/" + name)
    except Exception as exc:
        print("copy skip", name, exc)

n.destroy_node(); rclpy.shutdown()
PY

# Try open map in Windows viewer
cmd.exe /c start "" "C:\\Users\\ha054\\Downloads\\gitlab_spawn_map.png" 2>/dev/null || true
cmd.exe /c start "" "C:\\Users\\ha054\\Downloads\\gitlab_spawn_camera.png" 2>/dev/null || true
