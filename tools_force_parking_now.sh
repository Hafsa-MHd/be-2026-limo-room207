#!/usr/bin/env bash
# Reset limo to official Room 207 parking via SetEntityState (keeps /odom plugins).
# Falls back to delete+spawn only if the model is missing.
set -eo pipefail
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash

python3 - <<'PY'
import math
import time
import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState, SpawnEntity, DeleteEntity
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

X, Y, Z, YAW = -0.725347, -0.086613, 0.15, 1.5708

rclpy.init()
node = Node("force_parking_now")
qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
xml = {"v": None}
odom = {"v": None}
node.create_subscription(String, "/robot_description", lambda m: xml.__setitem__("v", m.data), qos)
node.create_subscription(Odometry, "/odom", lambda m: odom.__setitem__("v", m), 10)
cmd = node.create_publisher(Twist, "/cmd_vel", 10)
mission = node.create_publisher(Bool, "/control/mission_enable", 10)

def hold():
    cmd.publish(Twist())
    b = Bool(); b.data = False
    mission.publish(b)

# Prefer teleport — does not tear down Gazebo plugins.
cli = node.create_client(SetEntityState, "/gazebo/set_entity_state")
if cli.wait_for_service(timeout_sec=8.0):
    q = Quaternion(x=0.0, y=0.0, z=math.sin(YAW / 2), w=math.cos(YAW / 2))
    st = EntityState()
    st.name = "limo"
    st.pose = Pose(position=Point(x=X, y=Y, z=Z), orientation=q)
    st.twist = Twist()
    st.reference_frame = "world"
    for attempt in range(1, 4):
        hold()
        fut = cli.call_async(SetEntityState.Request(state=st))
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        print(f"teleport attempt {attempt}: {fut.result()}")
        t0 = time.time()
        while time.time() - t0 < 4.0:
            hold()
            rclpy.spin_once(node, timeout_sec=0.05)
            if odom["v"] is None:
                continue
            p = odom["v"].pose.pose.position
            if math.hypot(p.x - X, p.y - Y) < 0.35:
                print(f"parking OK via teleport ({p.x:.2f},{p.y:.2f})")
                for _ in range(20):
                    hold()
                    rclpy.spin_once(node, timeout_sec=0.05)
                node.destroy_node()
                rclpy.shutdown()
                raise SystemExit(0)
        time.sleep(0.5)
    print("teleport did not confirm parking — trying spawn fallback")
else:
    print("set_entity_state unavailable — spawn fallback")

# Fallback: classic delete+spawn (may lose plugins on broken worlds).
t0 = time.time()
while xml["v"] is None and time.time() - t0 < 20.0:
    hold()
    rclpy.spin_once(node, timeout_sec=0.1)
if xml["v"] is None:
    print("ERROR: no /robot_description")
    node.destroy_node(); rclpy.shutdown(); raise SystemExit(1)

cli_del = node.create_client(DeleteEntity, "/delete_entity")
cli_sp = node.create_client(SpawnEntity, "/spawn_entity")
if not cli_del.wait_for_service(10.0) or not cli_sp.wait_for_service(10.0):
    print("ERROR: spawn services missing")
    node.destroy_node(); rclpy.shutdown(); raise SystemExit(1)

for name in ("limo", "polar_goal_ball"):
    req = DeleteEntity.Request(); req.name = name
    fut = cli_del.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=8.0)
    print(f"delete {name}: {fut.result()}")
time.sleep(1.0)
odom["v"] = None
sp = SpawnEntity.Request()
sp.name = "limo"
sp.xml = xml["v"]
sp.initial_pose.position.x = X
sp.initial_pose.position.y = Y
sp.initial_pose.position.z = Z
sp.initial_pose.orientation.z = math.sin(YAW / 2)
sp.initial_pose.orientation.w = math.cos(YAW / 2)
sp.reference_frame = "world"
fut = cli_sp.call_async(sp)
rclpy.spin_until_future_complete(node, fut, timeout_sec=20.0)
print(f"spawn parking: {fut.result()}")
t0 = time.time()
while time.time() - t0 < 20.0:
    hold()
    rclpy.spin_once(node, timeout_sec=0.05)
    if odom["v"] is not None:
        p = odom["v"].pose.pose.position
        print(f"parking OK via spawn ({p.x:.2f},{p.y:.2f})")
        node.destroy_node(); rclpy.shutdown(); raise SystemExit(0)
print("ERROR: no /odom after spawn")
node.destroy_node(); rclpy.shutdown(); raise SystemExit(1)
PY
