#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGDIR="/tmp/limo_full"
mkdir -p "$LOGDIR"

if ! pgrep -x gzserver >/dev/null; then
  echo "Gazebo down — launching full stack..."
  nohup ros2 launch limo_simple room207_drive.launch.py control:=polar spawn_z:=0.15 \
    park_x:=-0.725347 park_y:=-0.086613 park_yaw:=1.5708 \
    >"$LOGDIR/drive.log" 2>&1 &
  # wait for odom near official pose
  python3 - <<'PY'
import time, rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
PARK=(-0.725347,-0.086613)
rclpy.init(); n=rclpy.create_node("wait_show")
st={"o":None}
n.create_subscription(Odometry,"/odom",lambda m: st.__setitem__("o",m),10)
c=n.create_publisher(Twist,"/cmd_vel",10)
e=n.create_publisher(Bool,"/control/mission_enable",10)
t0=time.time()
while time.time()-t0<75:
  rclpy.spin_once(n,timeout_sec=0.1)
  c.publish(Twist()); b=Bool(); b.data=False; e.publish(b)
  if st["o"] is None: continue
  p=st["o"].pose.pose.position
  err=((p.x-PARK[0])**2+(p.y-PARK[1])**2)**0.5
  if err<0.35:
    print("VISIBLE_SPAWN odom=(%.3f,%.3f)"%(p.x,p.y)); break
else:
  print("timeout waiting odom")
n.destroy_node(); rclpy.shutdown()
PY
else
  echo "Gazebo already up — forcing official spawn"
  bash "$ROOT/tools_force_parking_now.sh"
fi

echo "=== status ==="
pgrep -af 'gzserver|gzclient' | head -5 || true
python3 - <<'PY'
import time, rclpy
from nav_msgs.msg import Odometry
rclpy.init(); n=rclpy.create_node("pose")
st={"o":None}
n.create_subscription(Odometry,"/odom",lambda m: st.__setitem__("o",m),10)
t0=time.time()
while time.time()-t0<3 and st["o"] is None:
  rclpy.spin_once(n,timeout_sec=0.1)
if st["o"] is None:
  print("NO_ODOM")
else:
  p=st["o"].pose.pose.position
  print("odom=(%.3f,%.3f)  GitLab spawn=(-0.725,-0.087)"%(p.x,p.y))
n.destroy_node(); rclpy.shutdown()
PY
echo "Look at the Gazebo window — robot at Room 207 parking start (below the bay, heading north)."
