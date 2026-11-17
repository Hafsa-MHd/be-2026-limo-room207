#!/usr/bin/env bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set -e
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
source "$HOME/ros2_ws_simple/install/setup.bash"

echo "==== save panel ===="
python3 - <<'PY'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

rclpy.init()
node = Node('grab_panel')
bridge = CvBridge()
saved = {'ok': False}

def cb(msg):
    if saved['ok']:
        return
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    path = '/home/ha054/ros2_ws_simple/captures/traj_debug_panel.png'
    cv2.imwrite(path, img)
    print('saved', path, img.shape)
    saved['ok'] = True

node.create_subscription(Image, '/vision/panel', cb, 10)
t0 = node.get_clock().now()
while rclpy.ok() and not saved['ok']:
    rclpy.spin_once(node, timeout_sec=0.2)
    if (node.get_clock().now() - t0).nanoseconds > 8e9:
        print('timeout waiting panel')
        break
node.destroy_node()
rclpy.shutdown()
PY

cp -f "$HOME/ros2_ws_simple/captures/traj_debug_panel.png" \
  "$ROOT/captures_traj_debug_panel.png" 2>/dev/null || true

echo "==== stop lane_follow ===="
pkill -f 'lib/limo_simple/lane_follow' 2>/dev/null || true
sleep 1

echo "==== open-loop cmd_vel 4s ===="
python3 - <<'PY'
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

rclpy.init()
node = Node('open_loop')
pub = node.create_publisher(Twist, '/cmd_vel', 10)
pose = {'x': None, 'y': None, 'v': None}

def on_odom(msg):
    pose['x'] = msg.pose.pose.position.x
    pose['y'] = msg.pose.pose.position.y
    pose['v'] = msg.twist.twist.linear.x

node.create_subscription(Odometry, '/odom', on_odom, 10)
# wait odom
t0 = time.time()
while pose['x'] is None and time.time() - t0 < 5:
    rclpy.spin_once(node, timeout_sec=0.1)
x0, y0 = pose['x'], pose['y']
print(f'start x={x0} y={y0}')
cmd = Twist()
cmd.linear.x = 0.2
cmd.angular.z = 0.0
t0 = time.time()
while time.time() - t0 < 4.0:
    pub.publish(cmd)
    rclpy.spin_once(node, timeout_sec=0.05)
print(f'end x={pose["x"]} y={pose["y"]} v={pose["v"]}')
disp = math.hypot((pose['x'] or 0) - (x0 or 0), (pose['y'] or 0) - (y0 or 0))
print(f'displacement={disp:.3f}')
# stop
cmd.linear.x = 0.0
for _ in range(10):
    pub.publish(cmd)
    rclpy.spin_once(node, timeout_sec=0.05)
node.destroy_node()
rclpy.shutdown()
if disp < 0.05:
    raise SystemExit('OPEN_LOOP_FAIL')
print('OPEN_LOOP_OK')
PY

echo DONE
