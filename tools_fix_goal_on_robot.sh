#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFG="$ROOT/limo_simple/config/polar_follow.yaml"

# Soft respawn on asphalt
python3 - <<'PY'
import math, time
import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node("put_on_road")
xml = {"v": None}
qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
node.create_subscription(
    String, "/robot_description",
    lambda m: xml.__setitem__("v", m.data if xml["v"] is None else xml["v"]),
    qos,
)
t0 = time.time()
while xml["v"] is None and time.time() - t0 < 12:
    rclpy.spin_once(node, timeout_sec=0.1)
if xml["v"] is None:
    raise SystemExit("no robot_description")

d = node.create_client(DeleteEntity, "/delete_entity")
s = node.create_client(SpawnEntity, "/spawn_entity")
d.wait_for_service(10); s.wait_for_service(10)

# delete limo + old ball
for name in ("limo", "polar_goal_ball"):
    req = DeleteEntity.Request(); req.name = name
    fut = d.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5)
    print("delete", name, fut.result())
time.sleep(0.8)

sp = SpawnEntity.Request()
sp.name = "limo"
sp.xml = xml["v"]
sp.initial_pose.position.x = -0.725347
sp.initial_pose.position.y = -0.086613
sp.initial_pose.position.z = 0.15
yaw = 1.5708
sp.initial_pose.orientation.z = math.sin(yaw/2)
sp.initial_pose.orientation.w = math.cos(yaw/2)
sp.reference_frame = "world"
fut = s.call_async(sp)
rclpy.spin_until_future_complete(node, fut, timeout_sec=15)
print("spawn limo", fut.result())
node.destroy_node(); rclpy.shutdown()
PY

sleep 2

# Restart polar so ball tracks again
pkill -f 'limo_simple.polar_follow|polar_follow.py' 2>/dev/null || true
sleep 1
nohup ros2 run limo_simple polar_follow --ros-args --params-file "$CFG" >/tmp/polar.log 2>&1 &
disown || true
sleep 3

# Force ball once at current lookahead
sed -i 's/\r$//' "$ROOT/tools_force_goal_ball.py"
python3 "$ROOT/tools_force_goal_ball.py"

echo "=== check ==="
timeout 4 ros2 topic echo /vision/road_ok --once || true
timeout 4 ros2 topic echo /control/goal --once || true
echo "DONE: robot on road, red GOAL ball should be just in front"
