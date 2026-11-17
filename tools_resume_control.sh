#!/usr/bin/env bash
# Resume perception+control from official GitLab parking spawn.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGDIR="/tmp/limo_full"
mkdir -p "$LOGDIR"

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash

if ! pgrep -x gzserver >/dev/null; then
  echo "=== relaunch sim ==="
  nohup ros2 launch limo_simple room207_drive.launch.py control:=polar spawn_z:=0.15 \
    park_x:=-0.725347 park_y:=-0.086613 park_yaw:=1.5708 \
    >"$LOGDIR/drive.log" 2>&1 &
  sleep 8
fi

# Wait odom
python3 - <<'PY'
import time, math, rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

PARK=(-0.725347,-0.086613,1.5708)
rclpy.init(); n=rclpy.create_node("resume")
st={"o":None,"ok":None,"bias":None,"xml":None}
qos=QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
n.create_subscription(Odometry,"/odom",lambda m: st.__setitem__("o",m),10)
n.create_subscription(Bool,"/vision/road_ok",lambda m: st.__setitem__("ok",m.data),10)
n.create_subscription(Float32,"/vision/black_bias",lambda m: st.__setitem__("bias",m.data),10)
n.create_subscription(String,"/robot_description",lambda m: st.__setitem__("xml", m.data if st["xml"] is None else st["xml"]), qos)
cmd=n.create_publisher(Twist,"/cmd_vel",10)
en=n.create_publisher(Bool,"/control/mission_enable",10)

t0=time.time()
while time.time()-t0<40:
  rclpy.spin_once(n,timeout_sec=0.1)
  cmd.publish(Twist()); b=Bool(); b.data=False; en.publish(b)
  if st["o"] is not None:
    break
if st["o"] is None:
  print("NO_ODOM — trying respawn")
  d=n.create_client(DeleteEntity,"/delete_entity")
  s=n.create_client(SpawnEntity,"/spawn_entity")
  if not (st["xml"] and d.wait_for_service(10) and s.wait_for_service(10)):
    raise SystemExit("cannot respawn")
  for name in ("limo","polar_goal_ball"):
    req=DeleteEntity.Request(); req.name=name
    fut=d.call_async(req); rclpy.spin_until_future_complete(n,fut,timeout_sec=5)
  time.sleep(0.8)
  x,y,yaw=PARK
  sp=SpawnEntity.Request(); sp.name="limo"; sp.xml=st["xml"]
  sp.initial_pose.position.x=x; sp.initial_pose.position.y=y; sp.initial_pose.position.z=0.15
  sp.initial_pose.orientation.z=math.sin(yaw/2); sp.initial_pose.orientation.w=math.cos(yaw/2)
  sp.reference_frame="world"
  fut=s.call_async(sp); rclpy.spin_until_future_complete(n,fut,timeout_sec=15)
  print("spawn", fut.result())
  t0=time.time()
  while time.time()-t0<10:
    rclpy.spin_once(n,timeout_sec=0.1)
    if st["o"] is not None: break

if st["o"] is None:
  raise SystemExit("still no odom")
p=st["o"].pose.pose.position
err=((p.x-PARK[0])**2+(p.y-PARK[1])**2)**0.5
print("odom=(%.3f,%.3f) err_gitlab=%.3f road_ok=%s bias=%s"%(p.x,p.y,err,st["ok"],st["bias"]))
if err>0.40:
  # force official pose
  d=n.create_client(DeleteEntity,"/delete_entity"); s=n.create_client(SpawnEntity,"/spawn_entity")
  d.wait_for_service(5); s.wait_for_service(5)
  if st["xml"] is None:
    raise SystemExit("no xml")
  req=DeleteEntity.Request(); req.name="limo"
  fut=d.call_async(req); rclpy.spin_until_future_complete(n,fut,timeout_sec=5)
  time.sleep(0.8)
  x,y,yaw=PARK
  sp=SpawnEntity.Request(); sp.name="limo"; sp.xml=st["xml"]
  sp.initial_pose.position.x=x; sp.initial_pose.position.y=y; sp.initial_pose.position.z=0.15
  sp.initial_pose.orientation.z=math.sin(yaw/2); sp.initial_pose.orientation.w=math.cos(yaw/2)
  sp.reference_frame="world"
  fut=s.call_async(sp); rclpy.spin_until_future_complete(n,fut,timeout_sec=15)
  print("forced", fut.result())
  time.sleep(1.5)
  t0=time.time()
  while time.time()-t0<5:
    rclpy.spin_once(n,timeout_sec=0.1)
  p=st["o"].pose.pose.position
  print("odom2=(%.3f,%.3f) road_ok=%s bias=%s"%(p.x,p.y,st["ok"],st["bias"]))
n.destroy_node(); rclpy.shutdown()
PY

# Ensure single vision + polar with Downloads yaml (require_mission, map gate)
pkill -f 'limo_simple.lane_follow|lane_follow.py' 2>/dev/null || true
pkill -f 'limo_simple.polar_follow|polar_follow.py' 2>/dev/null || true
pkill -f 'limo_simple.road_vision|road_vision.py' 2>/dev/null || true
sleep 1
CFG="$ROOT/limo_simple/config"
nohup ros2 run limo_simple road_vision --ros-args --params-file "$CFG/road_vision.yaml" >"$LOGDIR/vision.log" 2>&1 &
nohup ros2 run limo_simple polar_follow --ros-args --params-file "$CFG/polar_follow.yaml" >"$LOGDIR/polar.log" 2>&1 &
sleep 4

python3 - <<'PY'
import time, rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
rclpy.init(); n=rclpy.create_node("chk2")
st={"o":None,"ok":None,"bias":None}
n.create_subscription(Odometry,"/odom",lambda m: st.__setitem__("o",m),10)
n.create_subscription(Bool,"/vision/road_ok",lambda m: st.__setitem__("ok",m.data),10)
n.create_subscription(Float32,"/vision/black_bias",lambda m: st.__setitem__("bias",m.data),10)
c=n.create_publisher(Twist,"/cmd_vel",10)
e=n.create_publisher(Bool,"/control/mission_enable",10)
t0=time.time()
while time.time()-t0<4:
  rclpy.spin_once(n,timeout_sec=0.05)
  c.publish(Twist()); b=Bool(); b.data=False; e.publish(b)
p=st["o"].pose.pose.position if st["o"] else None
print("READY odom=%s road_ok=%s bias=%s"%(None if p is None else (round(p.x,3),round(p.y,3)), st["ok"], st["bias"]))
print("HOLD until mission — perception+control live")
n.destroy_node(); rclpy.shutdown()
PY
