#!/usr/bin/env bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set -e
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

bash "$ROOT/tools_kill_gazebo.sh"
pkill -x rviz2 2>/dev/null || true
pkill -f 'spawn_entity.py' 2>/dev/null || true
pkill -f 'joint_state_publisher' 2>/dev/null || true
pkill -f 'robot_state_publisher' 2>/dev/null || true
pkill -f 'lib/limo_simple/road_vision' 2>/dev/null || true
pkill -f 'lib/limo_simple/lane_follow' 2>/dev/null || true
pkill -f 'room207_drive.launch' 2>/dev/null || true
pkill -f 'room207_vision.launch' 2>/dev/null || true
sleep 2

cd "$HOME/ros2_ws_simple"
colcon build --packages-select limo_simple --symlink-install
source "$HOME/ros2_ws_simple/install/setup.bash"

export DISPLAY="${DISPLAY:-:0}"
mkdir -p "$HOME/ros2_ws_simple/captures" /tmp
nohup ros2 launch limo_simple room207_drive.launch.py > /tmp/trajectory_test.log 2>&1 &
echo "LAUNCH_PID=$!"

for i in $(seq 1 60); do
  if timeout 3 ros2 topic hz /rgb/image_raw --window 3 2>&1 | grep -q 'average rate'; then
    echo "CAMERA_LIVE at ${i}s"
    break
  fi
  if grep -q 'process has died.*gzserver' /tmp/trajectory_test.log 2>/dev/null; then
    echo "GAZEBO_DEAD"
    exit 1
  fi
  sleep 2
done

# Wait for entity, then teleport onto black asphalt (school spawn).
sleep 5
python3 - <<'PY'
import time
import math
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist, Point, Quaternion
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

rclpy.init()
node = Node('traj_reset_and_measure')

# --- teleport ---
cli = node.create_client(SetEntityState, '/gazebo/set_entity_state')
if not cli.wait_for_service(timeout_sec=20.0):
    raise SystemExit('set_entity_state unavailable')

# School spawn on qcar track: facing +y
yaw = 1.5708
q = Quaternion(x=0.0, y=0.0, z=math.sin(yaw/2), w=math.cos(yaw/2))
state = EntityState()
state.name = 'limo'
state.pose = Pose(position=Point(x=-0.725347, y=-0.086613, z=0.15), orientation=q)
state.twist = Twist()
state.reference_frame = 'world'
req = SetEntityState.Request(state=state)
fut = cli.call_async(req)
rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
print('teleport', fut.result())

road_ok = {'v': False}
poses = []
cmds = []

def on_ok(msg):
    road_ok['v'] = bool(msg.data)

def on_odom(msg):
    poses.append((
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.twist.twist.linear.x,
    ))

def on_cmd(msg):
    cmds.append((msg.linear.x, msg.angular.z))

node.create_subscription(Bool, '/vision/road_ok', on_ok, 10)
node.create_subscription(Odometry, '/odom', on_odom, 10)
node.create_subscription(Twist, '/cmd_vel', on_cmd, 10)

# wait road
t0 = time.time()
while time.time() - t0 < 15.0:
    rclpy.spin_once(node, timeout_sec=0.1)
    if road_ok['v']:
        print('ROAD_OK')
        break
else:
    print('WARN: road_ok still false after teleport')

poses.clear()
cmds.clear()
t0 = time.time()
while time.time() - t0 < 25.0:
    rclpy.spin_once(node, timeout_sec=0.1)

node.destroy_node()
rclpy.shutdown()

if len(poses) < 5:
    print('FAIL: few odom', len(poses))
    raise SystemExit(2)

x0, y0, _ = poses[0]
x1, y1, _ = poses[-1]
dist = 0.0
for i in range(1, len(poses)):
    dx = poses[i][0] - poses[i-1][0]
    dy = poses[i][1] - poses[i-1][1]
    dist += math.hypot(dx, dy)
disp = math.hypot(x1 - x0, y1 - y0)
vmax = max(abs(p[2]) for p in poses)
cmd_v = max((c[0] for c in cmds), default=0.0)
cmd_w = max((abs(c[1]) for c in cmds), default=0.0)
print(f'start=({x0:.3f},{y0:.3f}) end=({x1:.3f},{y1:.3f})')
print(f'path_length={dist:.3f}m displacement={disp:.3f}m')
print(f'odom_v_max={vmax:.3f} cmd_v_max={cmd_v:.3f} cmd_w_max={cmd_w:.3f} cmds={len(cmds)}')
if cmd_v < 0.02 or dist < 0.20:
    print('RESULT FAIL: no trajectory follow')
    raise SystemExit(3)
print('RESULT OK: trajectory follow observed')
PY

echo "==== log ===="
strings /tmp/trajectory_test.log | grep -E 'lane_follow|road_vision|successfully spawned|died|ERROR' | tail -25 || true
echo TRAJECTORY_TEST_DONE
