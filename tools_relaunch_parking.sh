#!/usr/bin/env bash
# Clean relaunch: one sim, auto-move from school road → parking stall.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGDIR="/tmp/limo_full"
mkdir -p "$LOGDIR"

echo "=== hard stop old sims ==="
killall -9 gzserver gzclient 2>/dev/null || true
pkill -9 -f 'ros2 launch limo_simple' 2>/dev/null || true
pkill -9 -f 'limo_simple.road_vision|road_vision.py' 2>/dev/null || true
pkill -9 -f 'limo_simple.polar_follow|polar_follow.py' 2>/dev/null || true
pkill -9 -f 'limo_simple.lane_follow|lane_follow.py' 2>/dev/null || true
pkill -9 -f 'limo_simple.spawn_parking|spawn_parking.py' 2>/dev/null || true
pkill -9 -f '/rviz2( |$)|robot_state_publisher|joint_state_publisher|spawn_entity.py' 2>/dev/null || true
sleep 2
for i in $(seq 1 15); do
  pgrep -x gzserver >/dev/null || break
  killall -9 gzserver 2>/dev/null || true
  sleep 1
done

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws_simple
colcon build --packages-select limo_simple --symlink-install >"$LOGDIR/build.log" 2>&1
source ~/ros2_ws_simple/install/setup.bash

echo "=== launch (school spawn then auto parking stall) ==="
nohup ros2 launch limo_simple room207_drive.launch.py control:=polar spawn_z:=0.15 \
  park_x:=-0.725347 park_y:=-0.086613 park_yaw:=1.5708 \
  >"$LOGDIR/drive.log" 2>&1 &
echo $! >"$LOGDIR/drive.pid"

echo "waiting for parking pose..."
python3 - <<'PY'
import time
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

PARK=(-0.725347,-0.086613)
rclpy.init(); n=rclpy.create_node("wait_park")
st={"o":None}
n.create_subscription(Odometry,"/odom",lambda m: st.__setitem__("o",m),10)
c=n.create_publisher(Twist,"/cmd_vel",10)
e=n.create_publisher(Bool,"/control/mission_enable",10)
t0=time.time(); ok=False
while time.time()-t0 < 90:
  rclpy.spin_once(n, timeout_sec=0.1)
  c.publish(Twist()); b=Bool(); b.data=False; e.publish(b)
  o=st["o"]
  if o is None:
    continue
  p=o.pose.pose.position
  err=((p.x-PARK[0])**2+(p.y-PARK[1])**2)**0.5
  # reject school road pose
  school=((p.x+0.725)**2+(p.y+0.087)**2)**0.5
  if err < 0.40:
    print("PARKING_OK odom=(%.3f,%.3f) err=%.3fm (not school, d_school=%.2f)"%(p.x,p.y,err,school))
    ok=True
    break
  if int(time.time()-t0) % 5 == 0:
    print("... odom=(%.3f,%.3f) err_park=%.2f d_school=%.2f"%(p.x,p.y,err,school))
if not ok:
  print("FAIL: still not in parking after 90s")
  raise SystemExit(1)
n.destroy_node(); rclpy.shutdown()
PY

echo "=== tail spawn_parking ==="
grep -E 'parking|spawn_parking|PARKING|school' "$LOGDIR/drive.log" | tail -20 || true
pgrep -af 'gzserver|spawn_parking|polar_follow|road_vision' | head -15 || true
echo "RELAUNCH DONE — start pose is GitLab Room 207 spawn (-0.725, -0.087)"
