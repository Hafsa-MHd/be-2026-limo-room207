#!/usr/bin/env bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source ~/ros2_ws_simple/install/setup.bash
# dedupe polar if spawn script restarted it
N=$(pgrep -fc 'limo_simple.polar_follow|polar_follow.py' || true)
echo "polar_count=$N"
if [ "${N:-0}" -gt 2 ]; then
  pkill -f 'limo_simple.polar_follow|polar_follow.py' 2>/dev/null || true
  sleep 1
  CFG="$ROOT/limo_simple/config"
  nohup ros2 run limo_simple polar_follow --ros-args --params-file "$CFG/polar_follow.yaml" >/tmp/limo_long/polar.log 2>&1 &
  sleep 2
fi
python3 - <<'PY'
import time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
rclpy.init()
n=rclpy.create_node("confirm_hold")
st={"o":None,"ok":None,"cmd":None}
n.create_subscription(Odometry,"/odom",lambda m: st.__setitem__("o",m),10)
n.create_subscription(Bool,"/vision/road_ok",lambda m: st.__setitem__("ok",m.data),10)
n.create_subscription(Twist,"/cmd_vel",lambda m: st.__setitem__("cmd",m),10)
pub=n.create_publisher(Twist,"/cmd_vel",10)
en=n.create_publisher(Bool,"/control/mission_enable",10)
# sample for 3s while forcing hold
t0=time.time(); xs=[]; ys=[]
while time.time()-t0<3.5:
  rclpy.spin_once(n,timeout_sec=0.05)
  pub.publish(Twist())
  b=Bool(); b.data=False; en.publish(b)
  if st["o"] is not None:
    p=st["o"].pose.pose.position; xs.append(p.x); ys.append(p.y)
if xs:
  print("odom_end=(%.3f,%.3f) drift=%.3fm"%(xs[-1],ys[-1], ((xs[-1]-xs[0])**2+(ys[-1]-ys[0])**2)**0.5))
  print("in_parking", abs(xs[-1]+1.15)<0.25 and abs(ys[-1]-1.45)<0.25)
print("road_ok", st["ok"])
c=st["cmd"]
print("cmd", None if c is None else (c.linear.x, c.angular.z))
# mission publishers?
import subprocess
out=subprocess.getoutput("timeout 2 ros2 topic info /control/mission_goal 2>/dev/null | head -8")
print(out)
n.destroy_node(); rclpy.shutdown()
PY
