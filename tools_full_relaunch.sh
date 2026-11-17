#!/usr/bin/env bash
# Full relaunch: hard-kill → Gazebo+LIMO+vision+polar → parking bay → hold.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGDIR="/tmp/limo_full"
mkdir -p "$LOGDIR"

echo "=== HARD KILL ==="
# Precise kills — do not match this script's path/name.
killall -9 gzserver gzclient gazebo 2>/dev/null || true
pkill -9 -f 'limo_simple.road_vision|road_vision.py' 2>/dev/null || true
pkill -9 -f 'limo_simple.lane_follow|lane_follow.py' 2>/dev/null || true
pkill -9 -f 'limo_simple.polar_follow|polar_follow.py' 2>/dev/null || true
pkill -9 -f '/rviz2( |$)' 2>/dev/null || true
pkill -9 -f 'robot_state_publisher|joint_state_publisher' 2>/dev/null || true
pkill -9 -f 'spawn_entity.py' 2>/dev/null || true
pkill -9 -f 'mission_destinations|tools_run_mission' 2>/dev/null || true
pkill -9 -f 'ros2 launch limo_simple|ros2 launch room207_limo' 2>/dev/null || true
sleep 2
fuser -k 11345/tcp 2>/dev/null || true
sleep 2
# Wait until gzserver is really gone
for i in $(seq 1 20); do
  if ! pgrep -x gzserver >/dev/null; then
    echo "gzserver gone"
    break
  fi
  killall -9 gzserver 2>/dev/null || true
  sleep 1
done
if pgrep -x gzserver >/dev/null; then
  echo "WARN: gzserver still alive:"; pgrep -af gzserver
fi

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "=== BUILD ==="
cd ~/ros2_ws_simple
colcon build --packages-select limo_simple --symlink-install >"$LOGDIR/build.log" 2>&1
source ~/ros2_ws_simple/install/setup.bash

echo "=== LAUNCH ==="
nohup ros2 launch limo_simple room207_drive.launch.py control:=polar spawn_z:=0.15 \
  >"$LOGDIR/drive.log" 2>&1 &
echo $! >"$LOGDIR/drive.pid"

echo "waiting for fresh Gazebo + /robot_description..."
READY=0
for i in $(seq 1 90); do
  if pgrep -x gzserver >/dev/null \
     && timeout 1 ros2 topic list 2>/dev/null | grep -q /robot_description \
     && timeout 1 ros2 topic list 2>/dev/null | grep -q /odom; then
    echo "ready after ${i}x2s"
    READY=1
    break
  fi
  sleep 2
done
if [ "$READY" != "1" ]; then
  echo "FAIL: sim not ready"
  tail -50 "$LOGDIR/drive.log" || true
  exit 1
fi
sleep 8

echo "=== STOP + CLEAR MISSION ==="
python3 - <<'PY'
import time, rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
rclpy.init(); n=rclpy.create_node("pre")
c=n.create_publisher(Twist,"/cmd_vel",10)
e=n.create_publisher(Bool,"/control/mission_enable",10)
time.sleep(0.4)
for _ in range(12):
  c.publish(Twist()); b=Bool(); b.data=False; e.publish(b); time.sleep(0.05)
n.destroy_node(); rclpy.shutdown(); print("cleared")
PY
pkill -f 'mission_destinations|tools_run_mission' 2>/dev/null || true

echo "=== SPAWN PARKING BAY ==="
python3 - <<'PY'
import math, time
import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

PARK = (-1.15, 1.45, -0.85)
rclpy.init()
node = rclpy.create_node("spawn_park_bay")
xml = {"v": None}
odom = {"v": None}
ok = {"v": None}
qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
node.create_subscription(String, "/robot_description", lambda m: xml.__setitem__("v", m.data if xml["v"] is None else xml["v"]), qos)
node.create_subscription(Odometry, "/odom", lambda m: odom.__setitem__("v", m), 10)
node.create_subscription(Bool, "/vision/road_ok", lambda m: ok.__setitem__("v", bool(m.data)), 10)

t0 = time.time()
while xml["v"] is None and time.time() - t0 < 20:
    rclpy.spin_once(node, timeout_sec=0.1)
if xml["v"] is None:
    raise SystemExit("no robot_description")

d = node.create_client(DeleteEntity, "/delete_entity")
s = node.create_client(SpawnEntity, "/spawn_entity")
d.wait_for_service(15); s.wait_for_service(15)

for name in ("limo", "polar_goal_ball"):
    req = DeleteEntity.Request(); req.name = name
    fut = d.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=8)
    print("delete", name, fut.result())
time.sleep(1.0)

x, y, yaw = PARK
odom["v"] = None
sp = SpawnEntity.Request()
sp.name = "limo"
sp.xml = xml["v"]
sp.initial_pose.position.x = x
sp.initial_pose.position.y = y
sp.initial_pose.position.z = 0.15
sp.initial_pose.orientation.z = math.sin(yaw / 2.0)
sp.initial_pose.orientation.w = math.cos(yaw / 2.0)
sp.reference_frame = "world"
fut = s.call_async(sp)
rclpy.spin_until_future_complete(node, fut, timeout_sec=20)
print("spawn", fut.result())

t0 = time.time()
while time.time() - t0 < 6:
    rclpy.spin_once(node, timeout_sec=0.05)

if odom["v"] is None:
    raise SystemExit("no odom after spawn")
p = odom["v"].pose.pose.position
err = ((p.x - x) ** 2 + (p.y - y) ** 2) ** 0.5
print("odom=(%.3f,%.3f) err=%.3fm road_ok=%s" % (p.x, p.y, err, ok["v"]))
if err > 0.30:
    raise SystemExit("BAD_PARKING_POSE")
print("PARKING_OK")
node.destroy_node(); rclpy.shutdown()
PY

echo "=== ENSURE VISION+POLAR ==="
CFG="$ROOT/limo_simple/config"
# Keep launch nodes if alive; restart polar with our yaml (require_mission)
pkill -f 'limo_simple.lane_follow|lane_follow.py' 2>/dev/null || true
pkill -f 'limo_simple.polar_follow|polar_follow.py' 2>/dev/null || true
sleep 1
nohup ros2 run limo_simple polar_follow --ros-args --params-file "$CFG/polar_follow.yaml" \
  >"$LOGDIR/polar.log" 2>&1 &
# vision: restart if missing
if ! pgrep -f 'limo_simple.road_vision|road_vision.py' >/dev/null; then
  nohup ros2 run limo_simple road_vision --ros-args --params-file "$CFG/road_vision.yaml" \
    >"$LOGDIR/vision.log" 2>&1 &
fi
sleep 4

python3 - <<'PY'
import time, rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
PARK=(-1.15,1.45)
rclpy.init(); n=rclpy.create_node("hold")
st={"o":None,"ok":None}
n.create_subscription(Odometry,"/odom",lambda m: st.__setitem__("o",m),10)
n.create_subscription(Bool,"/vision/road_ok",lambda m: st.__setitem__("ok",m.data),10)
c=n.create_publisher(Twist,"/cmd_vel",10)
e=n.create_publisher(Bool,"/control/mission_enable",10)
t0=time.time()
while time.time()-t0<4:
  rclpy.spin_once(n,timeout_sec=0.05)
  c.publish(Twist()); b=Bool(); b.data=False; e.publish(b)
p=st["o"].pose.pose.position
err=((p.x-PARK[0])**2+(p.y-PARK[1])**2)**0.5
print("HOLD odom=(%.3f,%.3f) err=%.3fm road_ok=%s"%(p.x,p.y,err,st["ok"]))
print("RELAUNCH", "OK" if err<0.25 else "BAD")
n.destroy_node(); rclpy.shutdown()
PY

echo "=== procs ==="
pgrep -af 'gzserver|road_vision|polar_follow|rviz' | head -20 || true
echo "FULL RELAUNCH DONE — parking bay hold (no mission)"
