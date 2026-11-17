#!/usr/bin/env python3
"""Respawn at parking/road edge and check black preference + steer."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from cv_bridge import CvBridge
import cv2

_DESC_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Parking lot is top-left of qcar_large_track (~x=-1, y=3).
# Place at parking exit so camera sees black asphalt + light floor.
CANDIDATES = [
    # x, y, yaw (rad)
    (-1.40, 2.40, -1.20),  # exit looking toward road
    (-1.55, 2.20, -0.80),
    (-1.20, 2.60, -1.57),
    (-1.70, 2.50, -0.40),
    (-1.00, 2.30, -2.00),
    (-1.80, 1.80, 0.20),  # edge asphalt / light floor
    (-0.90, 1.60, -1.40),
]


def wait_desc(node):
    box = {"xml": None}

    def on_desc(msg):
        if box["xml"] is None:
            box["xml"] = msg.data

    sub = node.create_subscription(String, "/robot_description", on_desc, _DESC_QOS)
    t0 = time.time()
    while box["xml"] is None and time.time() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    if box["xml"] is None:
        raise SystemExit("no robot_description")
    return box["xml"]


def delete_spawn(node, xml, x, y, yaw):
    delete = node.create_client(DeleteEntity, "/delete_entity")
    spawn = node.create_client(SpawnEntity, "/spawn_entity")
    if not delete.wait_for_service(10.0) or not spawn.wait_for_service(10.0):
        raise SystemExit("delete/spawn unavailable")
    dreq = DeleteEntity.Request()
    dreq.name = "limo"
    fut = delete.call_async(dreq)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    time.sleep(0.8)
    sreq = SpawnEntity.Request()
    sreq.name = "limo"
    sreq.xml = xml
    sreq.robot_namespace = ""
    sreq.initial_pose.position.x = float(x)
    sreq.initial_pose.position.y = float(y)
    sreq.initial_pose.position.z = 0.15
    sreq.initial_pose.orientation.z = math.sin(yaw / 2.0)
    sreq.initial_pose.orientation.w = math.cos(yaw / 2.0)
    sreq.reference_frame = "world"
    fut = spawn.call_async(sreq)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
    res = fut.result()
    print(f"spawn ({x:.2f},{y:.2f},yaw={yaw:.2f}) -> {res}")
    return res


def sample(node, seconds=6.0):
    data = {
        "bias": [],
        "ok": [],
        "center": [],
        "cmd": [],
        "img": None,
    }
    bridge = CvBridge()

    def on_bias(msg):
        data["bias"].append(float(msg.data))

    def on_ok(msg):
        data["ok"].append(bool(msg.data))

    def on_center(msg):
        if len(msg.data) >= 1:
            data["center"].append(list(msg.data))

    def on_cmd(msg):
        data["cmd"].append((float(msg.linear.x), float(msg.angular.z)))

    def on_img(msg):
        if data["img"] is None:
            data["img"] = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    subs = [
        node.create_subscription(Float32, "/vision/black_bias", on_bias, 10),
        node.create_subscription(Bool, "/vision/road_ok", on_ok, 10),
        node.create_subscription(Float32MultiArray, "/vision/center", on_center, 10),
        node.create_subscription(Twist, "/cmd_vel", on_cmd, 10),
        node.create_subscription(Image, "/vision/overlay", on_img, 10),
    ]
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    for s in subs:
        node.destroy_subscription(s)
    return data


def summarize(tag, data, out_dir):
    bias = data["bias"][-20:] if data["bias"] else []
    ok = data["ok"][-20:] if data["ok"] else []
    cmds = data["cmd"][-40:] if data["cmd"] else []
    centers = data["center"][-10:] if data["center"] else []
    mean_bias = sum(bias) / len(bias) if bias else 0.0
    ok_ratio = sum(1 for v in ok if v) / max(1, len(ok))
    mean_w = sum(c[1] for c in cmds) / len(cmds) if cmds else 0.0
    mean_v = sum(c[0] for c in cmds) / len(cmds) if cmds else 0.0
    cx = None
    if centers and centers[-1]:
        # center msg may be [cx, heading, ...] or flat samples — take first usable
        arr = centers[-1]
        if len(arr) >= 2:
            cx = arr[0] if abs(arr[0]) > 1 else arr[1]
    print(
        f"{tag}: bias_mean={mean_bias:+.2f} road_ok={ok_ratio:.0%} "
        f"v={mean_v:+.3f} w={mean_w:+.3f} cx={cx} n_bias={len(bias)}"
    )
    if data["img"] is not None:
        path = f"{out_dir}/{tag}.png"
        cv2.imwrite(path, data["img"])
        print(f"  saved {path}")
    # Prefer black: nonzero bias with matching steer (bias>0 => black right => +w)
    prefer_ok = abs(mean_bias) > 0.2 and ok_ratio > 0.3
    steer_ok = (mean_bias * mean_w) > 0 or abs(mean_bias) < 0.2
    return prefer_ok, mean_bias, mean_w, steer_ok


def main():
    out_dir = "/home/ha054/ros2_ws_simple/captures"
    import os

    os.makedirs(out_dir, exist_ok=True)
    rclpy.init()
    node = Node("test_parking_exit")
    xml = wait_desc(node)

    # Warmup: confirm vision is alive before respawns.
    warm = sample(node, 3.0)
    if not warm["bias"]:
        raise SystemExit("vision dead: /vision/black_bias silent — restart road_vision")
    print(f"warmup bias={warm['bias'][-1]:+.2f} ok={warm['ok'][-1] if warm['ok'] else None}")

    best = None
    for i, (x, y, yaw) in enumerate(CANDIDATES):
        delete_spawn(node, xml, x, y, yaw)
        time.sleep(2.0)
        data = sample(node, 8.0)
        if not data["bias"]:
            print(f"park_exit_{i}: WARN vision silent after spawn")
            continue
        tag = f"park_exit_{i}"
        prefer_ok, bias, w, steer_ok = summarize(tag, data, out_dir)
        if prefer_ok and (best is None or abs(bias) > abs(best[1])):
            best = (tag, bias, w, steer_ok, (x, y, yaw))

    # Also put back on known good asphalt for baseline.
    delete_spawn(node, xml, -0.725347, -0.086613, 1.5708)
    time.sleep(1.5)
    data = sample(node, 6.0)
    summarize("on_asphalt", data, out_dir)

    node.destroy_node()
    rclpy.shutdown()

    if best is None:
        print("RESULT: no clear split view among candidates (vision still running)")
        raise SystemExit(2)
    tag, bias, w, steer_ok, pose = best
    print(
        f"RESULT OK best={tag} pose={pose} bias={bias:+.2f} w={w:+.3f} "
        f"steer_towards_black={steer_ok}"
    )


if __name__ == "__main__":
    main()
