#!/usr/bin/env python3
"""Delete and respawn LIMO on the black track, then measure lane_follow motion."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

_DESC_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def main():
    rclpy.init()
    node = Node("respawn_on_road")

    # Get robot_description
    desc = {"xml": None}

    def on_desc(msg):
        if desc["xml"] is None:
            desc["xml"] = msg.data

    sub = node.create_subscription(String, "/robot_description", on_desc, _DESC_QOS)
    t0 = time.time()
    while desc["xml"] is None and time.time() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    if desc["xml"] is None:
        raise SystemExit("no robot_description")

    delete = node.create_client(DeleteEntity, "/delete_entity")
    spawn = node.create_client(SpawnEntity, "/spawn_entity")
    if not delete.wait_for_service(10.0) or not spawn.wait_for_service(10.0):
        raise SystemExit("delete/spawn unavailable")

    dreq = DeleteEntity.Request()
    dreq.name = "limo"
    fut = delete.call_async(dreq)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    print("delete", fut.result())
    time.sleep(1.0)

    sreq = SpawnEntity.Request()
    sreq.name = "limo"
    sreq.xml = desc["xml"]
    sreq.robot_namespace = ""
    sreq.initial_pose.position.x = -0.725347
    sreq.initial_pose.position.y = -0.086613
    sreq.initial_pose.position.z = 0.15
    yaw = 1.5708
    sreq.initial_pose.orientation.z = math.sin(yaw / 2.0)
    sreq.initial_pose.orientation.w = math.cos(yaw / 2.0)
    sreq.reference_frame = "world"
    fut = spawn.call_async(sreq)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
    print("spawn", fut.result())

    road_ok = {"v": False}
    poses = []
    cmds = []

    def on_ok(msg):
        road_ok["v"] = bool(msg.data)

    def on_odom(msg):
        poses.append(
            (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.twist.twist.linear.x,
            )
        )

    def on_cmd(msg):
        cmds.append((msg.linear.x, msg.angular.z))

    node.create_subscription(Bool, "/vision/road_ok", on_ok, 10)
    node.create_subscription(Odometry, "/odom", on_odom, 10)
    node.create_subscription(Twist, "/cmd_vel", on_cmd, 10)

    t0 = time.time()
    while time.time() - t0 < 20.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if road_ok["v"]:
            print("ROAD_OK")
            break
    else:
        print("WARN: road_ok still false")

    poses.clear()
    cmds.clear()
    t0 = time.time()
    while time.time() - t0 < 25.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

    if len(poses) < 5:
        print("FAIL: few odom", len(poses))
        raise SystemExit(2)

    x0, y0, _ = poses[0]
    x1, y1, _ = poses[-1]
    dist = 0.0
    for i in range(1, len(poses)):
        dx = poses[i][0] - poses[i - 1][0]
        dy = poses[i][1] - poses[i - 1][1]
        dist += math.hypot(dx, dy)
    disp = math.hypot(x1 - x0, y1 - y0)
    vmax = max(abs(p[2]) for p in poses)
    cmd_v = max((c[0] for c in cmds), default=0.0)
    cmd_w = max((abs(c[1]) for c in cmds), default=0.0)
    print(f"start=({x0:.3f},{y0:.3f}) end=({x1:.3f},{y1:.3f})")
    print(f"path_length={dist:.3f}m displacement={disp:.3f}m")
    print(
        f"odom_v_max={vmax:.3f} cmd_v_max={cmd_v:.3f} cmd_w_max={cmd_w:.3f} cmds={len(cmds)}"
    )
    if cmd_v < 0.02 or dist < 0.20:
        print("RESULT FAIL: no trajectory follow")
        raise SystemExit(3)
    print("RESULT OK: trajectory follow observed")


if __name__ == "__main__":
    main()
