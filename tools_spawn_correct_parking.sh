#!/usr/bin/env bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash

echo "=== static consignes check ==="
python3 "$ROOT/tools_verify_consignes.py"

if ! pgrep -x gzserver >/dev/null; then
  echo "Gazebo not running — skip live spawn"
  exit 0
fi

# Restart polar with latest code
bash "$ROOT/tools_restart_polar_only.sh"

python3 - <<'PY'
import math, time
import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

# Official GitLab Room 207 spawn (room207_limo.launch.py).
PARK = (-0.725347, -0.086613, 1.5708)

rclpy.init()
node = rclpy.create_node("spawn_true_parking")
xml = {"v": None}
ok = {"v": None}
odom = {"v": None}
qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
node.create_subscription(String, "/robot_description", lambda m: xml.__setitem__("v", m.data if xml["v"] is None else xml["v"]), qos)
node.create_subscription(Bool, "/vision/road_ok", lambda m: ok.__setitem__("v", bool(m.data)), 10)
node.create_subscription(Odometry, "/odom", lambda m: odom.__setitem__("v", m), 10)

t0 = time.time()
while xml["v"] is None and time.time() - t0 < 12:
    rclpy.spin_once(node, timeout_sec=0.1)
if xml["v"] is None:
    raise SystemExit("no robot_description")

d = node.create_client(DeleteEntity, "/delete_entity")
s = node.create_client(SpawnEntity, "/spawn_entity")
d.wait_for_service(10); s.wait_for_service(10)
for name in ("limo", "polar_goal_ball"):
    req = DeleteEntity.Request(); req.name = name
    fut = d.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5)
time.sleep(0.6)

x,y,yaw = PARK
sp = SpawnEntity.Request()
sp.name = "limo"; sp.xml = xml["v"]
sp.initial_pose.position.x = x
sp.initial_pose.position.y = y
sp.initial_pose.position.z = 0.15
sp.initial_pose.orientation.z = math.sin(yaw/2)
sp.initial_pose.orientation.w = math.cos(yaw/2)
sp.reference_frame = "world"
fut = s.call_async(sp)
rclpy.spin_until_future_complete(node, fut, timeout_sec=15)
print("spawn", fut.result())

t0 = time.time()
while time.time() - t0 < 5:
    rclpy.spin_once(node, timeout_sec=0.05)

if odom["v"] is not None:
    p = odom["v"].pose.pose.position
    print(f"odom=({p.x:.2f},{p.y:.2f}) expected≈({x:.2f},{y:.2f})")
print("road_ok", ok["v"])
print("LIVE", "OK" if ok["v"] is True else "CHECK_CAMERA (may be false until facing black road)")
node.destroy_node(); rclpy.shutdown()
PY
