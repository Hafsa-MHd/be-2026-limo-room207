#!/usr/bin/env bash
# Long-running clean stack: parking bay + vision + polar (hold until mission).
set -eo pipefail
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFG="$ROOT/limo_simple/config"
LOGDIR="/tmp/limo_long"
mkdir -p "$LOGDIR"

echo "=== 1) stop cmd / mission / control ==="
python3 - <<'PY'
import time
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from gazebo_msgs.srv import DeleteEntity

rclpy.init()
n = rclpy.create_node("relaunch_stop")
cmd = n.create_publisher(Twist, "/cmd_vel", 10)
en = n.create_publisher(Bool, "/control/mission_enable", 10)
time.sleep(0.3)
b = Bool(); b.data = False
for _ in range(15):
    cmd.publish(Twist())
    en.publish(b)
    time.sleep(0.04)
cli = n.create_client(DeleteEntity, "/delete_entity")
if cli.wait_for_service(timeout_sec=3.0):
    req = DeleteEntity.Request(); req.name = "polar_goal_ball"
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=4)
print("stopped + mission cleared")
n.destroy_node(); rclpy.shutdown()
PY

pkill -f 'mission_destinations|tools_run_mission|tools_mission_destinations' 2>/dev/null || true
pkill -f 'tmp_check_mission|tmp_try_detect' 2>/dev/null || true
pkill -f 'limo_simple.lane_follow|lane_follow.py' 2>/dev/null || true
pkill -f 'limo_simple.polar_follow|polar_follow.py' 2>/dev/null || true
pkill -f 'limo_simple.road_vision|road_vision.py' 2>/dev/null || true
pkill -f 'rviz2' 2>/dev/null || true
sleep 2

if ! pgrep -x gzserver >/dev/null; then
  echo "Gazebo not running — start school sim first, then re-run this script."
  exit 1
fi
echo "Gazebo OK (keeping long sim session)"

echo "=== 2) rebuild limo_simple ==="
cd ~/ros2_ws_simple
colcon build --packages-select limo_simple --symlink-install >/tmp/limo_long/build.log 2>&1
source ~/ros2_ws_simple/install/setup.bash

echo "=== 3) vision + polar (nohup, long-lived) ==="
nohup ros2 run limo_simple road_vision --ros-args --params-file "$CFG/road_vision.yaml" \
  >"$LOGDIR/vision.log" 2>&1 &
echo $! >"$LOGDIR/vision.pid"
sleep 2
nohup ros2 run limo_simple polar_follow --ros-args --params-file "$CFG/polar_follow.yaml" \
  >"$LOGDIR/polar.log" 2>&1 &
echo $! >"$LOGDIR/polar.pid"
sleep 3

echo "=== 4) spawn in parking bay (black + stall lines) ==="
bash "$ROOT/tools_spawn_correct_parking.sh" | tee "$LOGDIR/spawn.log"

echo "=== 5) force hold (no mission) ==="
python3 - <<'PY'
import time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

PARK = (-1.15, 1.45)
rclpy.init()
n = rclpy.create_node("relaunch_hold")
st = {"o": None, "ok": None}
n.create_subscription(Odometry, "/odom", lambda m: st.__setitem__("o", m), 10)
n.create_subscription(Bool, "/vision/road_ok", lambda m: st.__setitem__("ok", m.data), 10)
cmd = n.create_publisher(Twist, "/cmd_vel", 10)
en = n.create_publisher(Bool, "/control/mission_enable", 10)
t0 = time.time()
while time.time() - t0 < 4.0:
    rclpy.spin_once(n, timeout_sec=0.05)
    cmd.publish(Twist())
    b = Bool(); b.data = False; en.publish(b)
o = st["o"]
ok = False
if o is not None:
    p = o.pose.pose.position
    err = ((p.x - PARK[0]) ** 2 + (p.y - PARK[1]) ** 2) ** 0.5
    ok = err < 0.20
    print("odom=(%.3f,%.3f) err=%.3fm parking=%s" % (p.x, p.y, err, ok))
print("road_ok", st["ok"])
print("HOLD_READY", "YES" if ok else "CHECK")
n.destroy_node(); rclpy.shutdown()
PY

echo "=== 6) optional RViz (camera panel) ==="
nohup ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true -- \
  -d "$ROOT/limo_simple/rviz/room207_vision.rviz" >"$LOGDIR/rviz.log" 2>&1 &
echo $! >"$LOGDIR/rviz.pid" || true

echo "=== procs ==="
pgrep -af 'road_vision|polar_follow|rviz2' | grep -v pgrep || true
echo "logs: $LOGDIR"
echo "LONG RELAUNCH DONE — robot holds in parking until you publish /control/mission_goal"
