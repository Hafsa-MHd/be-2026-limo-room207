#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFG="$ROOT/limo_simple/config"

echo "=== procs ==="
pgrep -af 'gzserver|gzclient|ros2 launch limo_simple' | head -10 || echo none

if ! pgrep -x gzserver >/dev/null; then
  echo "starting launch"
  nohup ros2 launch limo_simple room207_drive.launch.py control:=polar spawn_z:=0.15 >/tmp/limo_full/drive.log 2>&1 &
  sleep 15
fi

python3 - <<'PY'
import math, time
import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

PARK=(-1.15,1.45,-0.85)
rclpy.init(); n=rclpy.create_node("rescue")
xml={"v":None}; odom={"v":None}; ok={"v":None}
qos=QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
n.create_subscription(String,"/robot_description",lambda m: xml.__setitem__("v", m.data if xml["v"] is None else xml["v"]), qos)
n.create_subscription(Odometry,"/odom",lambda m: odom.__setitem__("v",m),10)
n.create_subscription(Bool,"/vision/road_ok",lambda m: ok.__setitem__("v",bool(m.data)),10)
cmd=n.create_publisher(Twist,"/cmd_vel",10)
en=n.create_publisher(Bool,"/control/mission_enable",10)
d=n.create_client(DeleteEntity,"/delete_entity")
s=n.create_client(SpawnEntity,"/spawn_entity")

t0=time.time()
while time.time()-t0<90:
  rclpy.spin_once(n,timeout_sec=0.1)
  if xml["v"] and d.service_is_ready() and s.service_is_ready():
    break
else:
  raise SystemExit("no services")

# If already in parking, just hold
if odom["v"] is not None:
  p=odom["v"].pose.pose.position
  err=((p.x-PARK[0])**2+(p.y-PARK[1])**2)**0.5
  print("current odom=(%.3f,%.3f) err=%.3f"%(p.x,p.y,err))
  if err < 0.30:
    print("already parking — hold")
    for _ in range(20):
      cmd.publish(Twist()); b=Bool(); b.data=False; en.publish(b); time.sleep(0.05)
    print("road_ok", ok["v"], "RESCUE_OK")
    n.destroy_node(); rclpy.shutdown(); raise SystemExit(0)

for name in ("limo","polar_goal_ball"):
  req=DeleteEntity.Request(); req.name=name
  fut=d.call_async(req); rclpy.spin_until_future_complete(n,fut,timeout_sec=8)
  print("delete", name, fut.result())
time.sleep(1.0)
x,y,yaw=PARK
odom["v"]=None
sp=SpawnEntity.Request(); sp.name="limo"; sp.xml=xml["v"]
sp.initial_pose.position.x=x; sp.initial_pose.position.y=y; sp.initial_pose.position.z=0.15
sp.initial_pose.orientation.z=math.sin(yaw/2); sp.initial_pose.orientation.w=math.cos(yaw/2)
sp.reference_frame="world"
fut=s.call_async(sp); rclpy.spin_until_future_complete(n,fut,timeout_sec=20)
print("spawn", fut.result())
t0=time.time()
while time.time()-t0<20:
  rclpy.spin_once(n,timeout_sec=0.05)
  cmd.publish(Twist()); b=Bool(); b.data=False; en.publish(b)
  if odom["v"] is not None:
    p=odom["v"].pose.pose.position
    err=((p.x-x)**2+(p.y-y)**2)**0.5
    if err<0.35:
      print("PARKING_OK (%.3f,%.3f) road_ok=%s"%(p.x,p.y,ok["v"]))
      break
else:
  raise SystemExit("spawn pose fail")
n.destroy_node(); rclpy.shutdown()
PY

# single controllers
pkill -f 'limo_simple.road_vision|road_vision.py|limo_simple.polar_follow|polar_follow.py|lane_follow' 2>/dev/null || true
sleep 2
nohup ros2 run limo_simple road_vision --ros-args --params-file "$CFG/road_vision.yaml" >/tmp/limo_full/vision.log 2>&1 &
nohup ros2 run limo_simple polar_follow --ros-args --params-file "$CFG/polar_follow.yaml" >/tmp/limo_full/polar.log 2>&1 &
sleep 4
python3 - <<'PY'
import time, rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
rclpy.init(); n=rclpy.create_node("h")
st={"o":None,"ok":None}
n.create_subscription(Odometry,"/odom",lambda m: st.__setitem__("o",m),10)
n.create_subscription(Bool,"/vision/road_ok",lambda m: st.__setitem__("ok",m.data),10)
c=n.create_publisher(Twist,"/cmd_vel",10); e=n.create_publisher(Bool,"/control/mission_enable",10)
t0=time.time()
while time.time()-t0<4:
  rclpy.spin_once(n,timeout_sec=0.05)
  c.publish(Twist()); b=Bool(); b.data=False; e.publish(b)
p=st["o"].pose.pose.position
print("FINAL (%.3f,%.3f) road_ok=%s"%(p.x,p.y,st["ok"]))
n.destroy_node(); rclpy.shutdown()
PY
echo DONE
