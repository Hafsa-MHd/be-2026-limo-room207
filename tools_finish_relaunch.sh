#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGDIR="/tmp/limo_full"

echo "=== status ==="
pgrep -af 'gzserver|gzclient|road_vision|polar_follow|room207|drive' | head -30 || true
echo "--- drive log ---"
tail -30 "$LOGDIR/drive.log" 2>/dev/null || true
echo "--- topics ---"
timeout 5 ros2 topic list 2>/dev/null | head -40 || true

echo "=== wait odom up to 40s ==="
python3 - <<'PY'
import math, time
import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

PARK = (-1.15, 1.45, -0.85)
rclpy.init()
node = rclpy.create_node("finish_relaunch")
xml={"v":None}; odom={"v":None}; ok={"v":None}
qos=QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
node.create_subscription(String,"/robot_description",lambda m: xml.__setitem__("v", m.data if xml["v"] is None else xml["v"]), qos)
node.create_subscription(Odometry,"/odom",lambda m: odom.__setitem__("v",m),10)
node.create_subscription(Bool,"/vision/road_ok",lambda m: ok.__setitem__("v", bool(m.data)),10)
cmd=node.create_publisher(Twist,"/cmd_vel",10)
en=node.create_publisher(Bool,"/control/mission_enable",10)

t0=time.time()
while time.time()-t0 < 25:
    rclpy.spin_once(node, timeout_sec=0.1)
    cmd.publish(Twist())
    b=Bool(); b.data=False; en.publish(b)
print("xml", "yes" if xml["v"] else "no", "odom", "yes" if odom["v"] else "no")

d=node.create_client(DeleteEntity,"/delete_entity")
s=node.create_client(SpawnEntity,"/spawn_entity")
print("delete svc", d.wait_for_service(5), "spawn svc", s.wait_for_service(5))

if xml["v"] is None:
    raise SystemExit("no robot_description — launch not healthy")

for name in ("limo","polar_goal_ball"):
    req=DeleteEntity.Request(); req.name=name
    fut=d.call_async(req)
    rclpy.spin_until_future_complete(node,fut,timeout_sec=8)
    print("delete", name, fut.result())
time.sleep(1.2)

x,y,yaw=PARK
odom["v"]=None
sp=SpawnEntity.Request()
sp.name="limo"; sp.xml=xml["v"]
sp.initial_pose.position.x=x; sp.initial_pose.position.y=y; sp.initial_pose.position.z=0.15
sp.initial_pose.orientation.z=math.sin(yaw/2); sp.initial_pose.orientation.w=math.cos(yaw/2)
sp.reference_frame="world"
fut=s.call_async(sp)
rclpy.spin_until_future_complete(node,fut,timeout_sec=20)
print("spawn", fut.result())

t0=time.time()
while time.time()-t0 < 15:
    rclpy.spin_once(node, timeout_sec=0.05)
    cmd.publish(Twist())
    if odom["v"] is not None:
        p=odom["v"].pose.pose.position
        err=((p.x-x)**2+(p.y-y)**2)**0.5
        if err < 0.35:
            print("PARKING_OK odom=(%.3f,%.3f) err=%.3fm road_ok=%s"%(p.x,p.y,err,ok["v"]))
            break
else:
    if odom["v"] is None:
        print("FAIL still no odom")
    else:
        p=odom["v"].pose.pose.position
        print("FAIL odom=(%.3f,%.3f)"%(p.x,p.y))

# ensure polar
import subprocess, os
subprocess.call("pkill -f 'limo_simple.lane_follow|lane_follow.py' || true", shell=True)
subprocess.call("pkill -f 'limo_simple.polar_follow|polar_follow.py' || true", shell=True)
time.sleep(1)
cfg="$ROOT/limo_simple/config/polar_follow.yaml"
os.system(f"nohup ros2 run limo_simple polar_follow --ros-args --params-file '{cfg}' >/tmp/limo_full/polar.log 2>&1 &")
if not subprocess.getoutput("pgrep -f road_vision || true"):
    vcfg="$ROOT/limo_simple/config/road_vision.yaml"
    os.system(f"nohup ros2 run limo_simple road_vision --ros-args --params-file '{vcfg}' >/tmp/limo_full/vision.log 2>&1 &")
time.sleep(3)
for _ in range(20):
    cmd.publish(Twist()); b=Bool(); b.data=False; en.publish(b)
    rclpy.spin_once(node, timeout_sec=0.05)
print("HOLD set; road_ok", ok["v"])
node.destroy_node(); rclpy.shutdown()
PY

pgrep -af 'gzserver|road_vision|polar_follow' | head -15 || true
echo DONE
