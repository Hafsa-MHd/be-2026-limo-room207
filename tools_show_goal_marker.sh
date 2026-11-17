#!/usr/bin/env bash
# Clean restart, put robot on road, prove /control/goal_marker is live.
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RVIZ="$ROOT/limo_simple/rviz/room207_vision.rviz"

echo "=== kill old ==="
pkill -9 -f gzserver || true
pkill -9 -f gzclient || true
pkill -9 -f gazebo || true
pkill -9 -f rviz2 || true
pkill -9 -f road_vision || true
pkill -9 -f polar_follow || true
pkill -9 -f lane_follow || true
pkill -9 -f 'ros2 launch' || true
sleep 2
fuser -k 11345/tcp 2>/dev/null || true
sleep 1

echo "=== launch drive polar ==="
nohup ros2 launch limo_simple room207_drive.launch.py control:=polar \
  >/tmp/drive_polar.log 2>&1 &
disown || true

echo "waiting for camera..."
for i in $(seq 1 60); do
  if timeout 1 ros2 topic hz /rgb/image_raw 2>&1 | grep -q average; then
    echo "camera ok at ${i}s"
    break
  fi
  sleep 1
done

# Wait polar node (starts after 8s timer)
for i in $(seq 1 30); do
  if ros2 node list 2>/dev/null | grep -q polar_follow; then
    echo "polar_follow up"
    break
  fi
  sleep 1
done

# Soft respawn on asphalt (no long trajectory test)
python3 - <<'PY'
import math, time
import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node("soft_respawn")
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
while xml["v"] is None and time.time() - t0 < 15:
    rclpy.spin_once(node, timeout_sec=0.1)
if xml["v"] is None:
    raise SystemExit("no robot_description")
d = node.create_client(DeleteEntity, "/delete_entity")
s = node.create_client(SpawnEntity, "/spawn_entity")
d.wait_for_service(15)
s.wait_for_service(15)
req = DeleteEntity.Request(); req.name = "limo"
fut = d.call_async(req)
rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
time.sleep(1.0)
sp = SpawnEntity.Request()
sp.name = "limo"
sp.xml = xml["v"]
sp.initial_pose.position.x = -0.725347
sp.initial_pose.position.y = -0.086613
sp.initial_pose.position.z = 0.15
yaw = 1.5708
sp.initial_pose.orientation.z = math.sin(yaw / 2)
sp.initial_pose.orientation.w = math.cos(yaw / 2)
sp.reference_frame = "world"
fut = s.call_async(sp)
rclpy.spin_until_future_complete(node, fut, timeout_sec=15)
print("spawn", fut.result())
node.destroy_node()
rclpy.shutdown()
PY

sleep 3
echo "=== road_ok ==="
timeout 6 ros2 topic echo /vision/road_ok --once || echo NO_OK

echo "=== goal pose ==="
timeout 6 ros2 topic echo /control/goal --once || echo NO_GOAL

echo "=== marker (looking for SPHERE / GOAL text) ==="
timeout 6 ros2 topic echo /control/goal_marker --once 2>&1 | tee /tmp/marker_echo.txt | head -80
if grep -q "SPHERE\|GOAL\|polar_goal" /tmp/marker_echo.txt 2>/dev/null; then
  echo "MARKER_OK: destination mark is publishing"
else
  echo "MARKER_MISSING"
fi

# Open our RViz config (has GoalMarker display) if not already
if ! pgrep -af room207_vision.rviz >/dev/null 2>&1; then
  echo "=== start RViz with goal displays ==="
  nohup rviz2 -d "$RVIZ" >/tmp/rviz_goal.log 2>&1 &
  disown || true
fi

echo "=== done: look in RViz for cyan sphere + yellow GOAL text in front of robot ==="
