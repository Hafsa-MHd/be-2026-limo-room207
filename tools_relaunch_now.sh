#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGDIR="/tmp/limo_full"
mkdir -p "$LOGDIR"

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash

echo "=== current ==="
pgrep -af 'gzserver|gzclient|ros2 launch limo_simple' | head -10 || echo "no gazebo/launch"

if ! pgrep -x gzserver >/dev/null; then
  echo "=== starting launch ==="
  nohup ros2 launch limo_simple room207_drive.launch.py control:=polar spawn_z:=0.15 \
    >"$LOGDIR/drive.log" 2>&1 &
  echo $! >"$LOGDIR/drive.pid"
fi

echo "=== wait services ==="
python3 - <<'PY'
import math, time
import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

PARK=(-1.15,1.45,-0.85)
rclpy.init()
n=rclpy.create_node("relaunch_now")
xml={"v":None}; odom={"v":None}; ok={"v":None}
qos=QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
n.create_subscription(String,"/robot_description",lambda m: xml.__setitem__("v", m.data if xml["v"] is None else xml["v"]), qos)
n.create_subscription(Odometry,"/odom",lambda m: odom.__setitem__("v",m),10)
n.create_subscription(Bool,"/vision/road_ok",lambda m: ok.__setitem__("v",bool(m.data)),10)
cmd=n.create_publisher(Twist,"/cmd_vel",10)
en=n.create_publisher(Bool,"/control/mission_enable",10)
d=n.create_client(DeleteEntity,"/delete_entity")
s=n.create_client(SpawnEntity,"/spawn_entity")

print("waiting up to 120s for description + spawn services...")
t0=time.time()
while time.time()-t0 < 120:
    rclpy.spin_once(n, timeout_sec=0.1)
    cmd.publish(Twist()); b=Bool(); b.data=False; en.publish(b)
    if xml["v"] and d.service_is_ready() and s.service_is_ready():
        print("services ready in %.1fs"%(time.time()-t0))
        break
else:
    print("xml",bool(xml["v"]),"del",d.service_is_ready(),"spawn",s.service_is_ready())
    raise SystemExit("services not ready")

for name in ("limo","polar_goal_ball"):
    req=DeleteEntity.Request(); req.name=name
    fut=d.call_async(req)
    rclpy.spin_until_future_complete(n,fut,timeout_sec=10)
    print("delete",name, fut.result())
time.sleep(1.0)

x,y,yaw=PARK
odom["v"]=None
sp=SpawnEntity.Request()
sp.name="limo"; sp.xml=xml["v"]
sp.initial_pose.position.x=x; sp.initial_pose.position.y=y; sp.initial_pose.position.z=0.15
sp.initial_pose.orientation.z=math.sin(yaw/2); sp.initial_pose.orientation.w=math.cos(yaw/2)
sp.reference_frame="world"
fut=s.call_async(sp)
rclpy.spin_until_future_complete(n,fut,timeout_sec=20)
print("spawn",fut.result())
if fut.result() is None or not fut.result().success:
    raise SystemExit("spawn failed")

t0=time.time(); good=False
while time.time()-t0 < 20:
    rclpy.spin_once(n, timeout_sec=0.05)
    cmd.publish(Twist()); b=Bool(); b.data=False; en.publish(b)
    if odom["v"] is not None:
        p=odom["v"].pose.pose.position
        err=((p.x-x)**2+(p.y-y)**2)**0.5
        if err < 0.35:
            print("PARKING_OK odom=(%.3f,%.3f) err=%.3fm road_ok=%s"%(p.x,p.y,err,ok["v"]))
            good=True
            break
        print("waiting pose (%.3f,%.3f) err=%.3f"%(p.x,p.y,err))
if not good:
    raise SystemExit("parking pose not reached")

# restart polar with Downloads yaml (require_mission)
import subprocess, os
subprocess.call(['bash','-lc',"pkill -f 'limo_simple.lane_follow|lane_follow.py' || true"])
subprocess.call(['bash','-lc',"pkill -f 'limo_simple.polar_follow|polar_follow.py' || true"])
time.sleep(1)
cfg="$ROOT/limo_simple/config/polar_follow.yaml"
os.system("nohup ros2 run limo_simple polar_follow --ros-args --params-file '%s' >/tmp/limo_full/polar.log 2>&1 &"%cfg)
time.sleep(3)
for _ in range(30):
    cmd.publish(Twist()); b=Bool(); b.data=False; en.publish(b)
    rclpy.spin_once(n, timeout_sec=0.05)
print("RELAUNCH_OK hold in parking")
n.destroy_node(); rclpy.shutdown()
PY

pgrep -af 'gzserver|road_vision|polar_follow' | head -12 || true
