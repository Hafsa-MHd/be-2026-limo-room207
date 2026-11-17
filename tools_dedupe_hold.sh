#!/usr/bin/env bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set -eo pipefail
source /opt/ros/humble/setup.bash
source ~/ros2_ws_simple/install/setup.bash
CFG="$ROOT/limo_simple/config"

# One vision, one polar only
pkill -f 'limo_simple.road_vision|road_vision.py' 2>/dev/null || true
pkill -f 'limo_simple.polar_follow|polar_follow.py' 2>/dev/null || true
pkill -f 'limo_simple.lane_follow|lane_follow.py' 2>/dev/null || true
sleep 2
nohup ros2 run limo_simple road_vision --ros-args --params-file "$CFG/road_vision.yaml" >/tmp/limo_full/vision.log 2>&1 &
nohup ros2 run limo_simple polar_follow --ros-args --params-file "$CFG/polar_follow.yaml" >/tmp/limo_full/polar.log 2>&1 &
sleep 4

python3 - <<'PY'
import time, rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
PARK=(-1.15,1.45)
rclpy.init(); n=rclpy.create_node("dedupe")
st={"o":None,"ok":None}
n.create_subscription(Odometry,"/odom",lambda m: st.__setitem__("o",m),10)
n.create_subscription(Bool,"/vision/road_ok",lambda m: st.__setitem__("ok",m.data),10)
c=n.create_publisher(Twist,"/cmd_vel",10)
e=n.create_publisher(Bool,"/control/mission_enable",10)
t0=time.time()
while time.time()-t0<5:
  rclpy.spin_once(n,timeout_sec=0.05)
  c.publish(Twist()); b=Bool(); b.data=False; e.publish(b)
p=st["o"].pose.pose.position if st["o"] else None
if p:
  err=((p.x-PARK[0])**2+(p.y-PARK[1])**2)**0.5
  print("odom=(%.3f,%.3f) err=%.3fm in_parking=%s"%(p.x,p.y,err,err<0.25))
print("road_ok", st["ok"])
print("READY")
n.destroy_node(); rclpy.shutdown()
PY
echo "=== procs ==="
pgrep -af 'road_vision|polar_follow|gzserver' | grep -v pgrep || true
tail -5 /tmp/limo_full/polar.log || true
