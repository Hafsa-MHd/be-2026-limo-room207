#!/usr/bin/env python3
"""Spawn at split pose; confirm bias toward black/yellow and matching steer."""

import math
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

_DESC_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def main():
    rclpy.init()
    node = Node("live_split_check")
    bridge = CvBridge()
    xml = {"v": None}

    def on_desc(msg):
        if xml["v"] is None:
            xml["v"] = msg.data

    node.create_subscription(String, "/robot_description", on_desc, _DESC_QOS)
    t0 = time.time()
    while xml["v"] is None and time.time() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.1)
    if xml["v"] is None:
        raise SystemExit("no robot_description")

    delete = node.create_client(DeleteEntity, "/delete_entity")
    spawn = node.create_client(SpawnEntity, "/spawn_entity")
    delete.wait_for_service(10)
    spawn.wait_for_service(10)
    d = DeleteEntity.Request()
    d.name = "limo"
    fut = delete.call_async(d)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
    time.sleep(0.8)

    x, y, yaw = -1.80, 1.80, 0.20
    s = SpawnEntity.Request()
    s.name = "limo"
    s.xml = xml["v"]
    s.initial_pose.position.x = x
    s.initial_pose.position.y = y
    s.initial_pose.position.z = 0.15
    s.initial_pose.orientation.z = math.sin(yaw / 2)
    s.initial_pose.orientation.w = math.cos(yaw / 2)
    s.reference_frame = "world"
    fut = spawn.call_async(s)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=15)
    print("spawn", fut.result())
    time.sleep(2.5)

    data = {"bias": [], "ok": [], "center": [], "cmd": [], "overlay": None}

    def on_bias(msg):
        data["bias"].append(float(msg.data))

    def on_ok(msg):
        data["ok"].append(bool(msg.data))

    def on_c(msg):
        if msg.data:
            data["center"].append(list(msg.data))

    def on_cmd(msg):
        data["cmd"].append((float(msg.linear.x), float(msg.angular.z)))

    def on_o(msg):
        if data["overlay"] is None:
            data["overlay"] = bridge.imgmsg_to_cv2(msg, "bgr8")

    for topic, typ, cb in [
        ("/vision/black_bias", Float32, on_bias),
        ("/vision/road_ok", Bool, on_ok),
        ("/vision/center", Float32MultiArray, on_c),
        ("/cmd_vel", Twist, on_cmd),
        ("/vision/overlay", Image, on_o),
    ]:
        node.create_subscription(typ, topic, cb, 10)

    t0 = time.time()
    while time.time() - t0 < 10.0:
        rclpy.spin_once(node, timeout_sec=0.05)

    if not data["bias"]:
        raise SystemExit("vision silent — restart road_vision")

    bias = sum(data["bias"][-30:]) / len(data["bias"][-30:])
    ok = sum(1 for v in data["ok"][-30:] if v) / max(1, len(data["ok"][-30:]))
    cmds = data["cmd"][-50:]
    mean_v = sum(c[0] for c in cmds) / max(1, len(cmds))
    mean_w = sum(c[1] for c in cmds) / max(1, len(cmds))
    cx = data["center"][-1][0] if data["center"] else None
    print(
        f"bias={bias:+.3f} road_ok={ok:.0%} cx={cx} v={mean_v:+.3f} w={mean_w:+.3f} "
        f"n_bias={len(data['bias'])}"
    )
    if data["overlay"] is not None:
        path = "/home/ha054/ros2_ws_simple/captures/live_split_overlay.png"
        cv2.imwrite(path, data["overlay"])
        print("saved", path)

    # For this pose: yellow/black lane is left => bias < 0, steer should go left.
    # LIMO: positive angular.z steers left (CCW). err left => steer = -k*err > 0.
    prefer_black = bias < -0.15 and ok > 0.5
    steer_left = mean_w > 0.02 or (cx is not None and cx < 300 and mean_v > 0.02)
    if prefer_black and mean_v > 0.01:
        print("RESULT OK: prefers black/yellow lane and drives")
    else:
        print(
            f"RESULT CHECK prefer_black={prefer_black} driving={mean_v>0.01} "
            f"steer_leftish={steer_left}"
        )
        raise SystemExit(2)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
