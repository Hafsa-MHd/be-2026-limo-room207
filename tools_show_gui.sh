#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGDIR="/tmp/limo_full"
mkdir -p "$LOGDIR"

killall -9 gzserver gzclient 2>/dev/null || true
pkill -9 -f 'ros2 launch limo_simple' 2>/dev/null || true
pkill -9 -f 'limo_simple.road_vision|road_vision.py|limo_simple.polar_follow|polar_follow.py|limo_simple.spawn_parking|spawn_parking.py|rviz2|robot_state_publisher|joint_state_publisher' 2>/dev/null || true
sleep 2
for i in $(seq 1 10); do
  pgrep -x gzserver >/dev/null || break
  killall -9 gzserver 2>/dev/null || true
  sleep 1
done

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash

echo "Launching Gazebo GUI + official Room 207 spawn..."
nohup ros2 launch limo_simple room207_drive.launch.py control:=polar spawn_z:=0.15 \
  park_x:=-0.725347 park_y:=-0.086613 park_yaw:=1.5708 \
  >"$LOGDIR/drive.log" 2>&1 &

python3 - <<'PY'
import time, rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
PARK=(-0.725347,-0.086613)
rclpy.init(); n=rclpy.create_node("wait_gui")
st={"o":None}
n.create_subscription(Odometry,"/odom",lambda m: st.__setitem__("o",m),10)
c=n.create_publisher(Twist,"/cmd_vel",10)
e=n.create_publisher(Bool,"/control/mission_enable",10)
t0=time.time()
while time.time()-t0<90:
  rclpy.spin_once(n,timeout_sec=0.1)
  c.publish(Twist()); b=Bool(); b.data=False; e.publish(b)
  if st["o"] is None: continue
  p=st["o"].pose.pose.position
  err=((p.x-PARK[0])**2+(p.y-PARK[1])**2)**0.5
  if err<0.40:
    print("READY odom=(%.3f,%.3f) — look at Gazebo"%(p.x,p.y))
    break
else:
  print("TIMEOUT — check /tmp/limo_full/drive.log")
n.destroy_node(); rclpy.shutdown()
PY

pgrep -af 'gzserver|gzclient' | head -5 || true
echo "GitLab start: (-0.725, -0.087, yaw=north) just below parking bay"
