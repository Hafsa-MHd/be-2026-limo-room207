#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws_simple/install/setup.bash
pkill -f 'mission_destinations|tools_mission_destinations|tools_run_mission' 2>/dev/null || true
sleep 0.5
pgrep -af 'mission' || echo "no mission procs"
python3 - <<'PY'
import time
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
rclpy.init()
n=rclpy.create_node("fin")
st={"o":None}
n.create_subscription(Odometry,"/odom",lambda m: st.__setitem__("o",m),10)
c=n.create_publisher(Twist,"/cmd_vel",10)
e=n.create_publisher(Bool,"/control/mission_enable",10)
t0=time.time()
while time.time()-t0<2:
  rclpy.spin_once(n,timeout_sec=0.05)
  c.publish(Twist())
  b=Bool(); b.data=False; e.publish(b)
o=st["o"]
if o:
  p=o.pose.pose.position
  print("HOLD odom=(%.3f,%.3f) in_parking=%s"%(p.x,p.y, abs(p.x+1.15)<0.25 and abs(p.y-1.45)<0.25))
n.destroy_node(); rclpy.shutdown()
PY
# verify no mission publisher
timeout 2 ros2 topic info /control/mission_goal 2>/dev/null | head -20 || true
