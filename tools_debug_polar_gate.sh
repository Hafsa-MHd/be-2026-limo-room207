#!/usr/bin/env bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source ~/ros2_ws_simple/install/setup.bash
echo "=== polar log ==="
tail -30 /tmp/limo_full/polar.log || tail -30 /tmp/polar.log || true
echo "=== rebuild ==="
cd ~/ros2_ws_simple && colcon build --packages-select limo_simple --symlink-install 2>&1 | tail -6
source ~/ros2_ws_simple/install/setup.bash
pkill -f 'limo_simple.polar_follow|polar_follow.py' 2>/dev/null || true
sleep 1
CFG="$ROOT/limo_simple/config/polar_follow.yaml"
nohup ros2 run limo_simple polar_follow --ros-args --params-file "$CFG" >/tmp/limo_full/polar.log 2>&1 &
sleep 3
# publish mission a few times
python3 - <<'PY'
import time, rclpy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Odometry
rclpy.init(); n=rclpy.create_node("dbg")
st={"rho":None,"o":None,"ok":None}
n.create_subscription(Float32,"/control/rho",lambda m: st.__setitem__("rho",m.data),10)
n.create_subscription(Odometry,"/odom",lambda m: st.__setitem__("o",m),10)
n.create_subscription(Bool,"/vision/road_ok",lambda m: st.__setitem__("ok",m.data),10)
n.create_subscription(Twist,"/cmd_vel",lambda m: st.__setitem__("cmd",(m.linear.x,m.angular.z)),10)
pg=n.create_publisher(PoseStamped,"/control/mission_goal",10)
pe=n.create_publisher(Bool,"/control/mission_enable",10)
for i in range(40):
  g=PoseStamped(); g.header.frame_id="odom"; g.header.stamp=n.get_clock().now().to_msg()
  g.pose.position.x=0.976; g.pose.position.y=-0.897; g.pose.orientation.w=1.0
  pg.publish(g); b=Bool(); b.data=True; pe.publish(b)
  rclpy.spin_once(n,timeout_sec=0.05)
  time.sleep(0.05)
print("after publish: road_ok",st["ok"],"rho",st["rho"],"cmd",st.get("cmd"),"odom",None if st["o"] is None else (st["o"].pose.pose.position.x,st["o"].pose.pose.position.y))
time.sleep(2)
t0=time.time()
while time.time()-t0<3:
  rclpy.spin_once(n,timeout_sec=0.05)
print("later: road_ok",st["ok"],"rho",st["rho"],"cmd",st.get("cmd"))
# clear
b=Bool(); b.data=False; pe.publish(b)
n.destroy_node(); rclpy.shutdown()
PY
echo "=== polar log after ==="
tail -20 /tmp/limo_full/polar.log
