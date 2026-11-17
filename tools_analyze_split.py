#!/usr/bin/env python3
"""Respawn at known split pose and dump RGB/mask stats."""

import math
import time
import os

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from limo_simple.road_vision import segment_road

_DESC_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def main():
    out = "/home/ha054/ros2_ws_simple/captures"
    os.makedirs(out, exist_ok=True)
    rclpy.init()
    node = Node("analyze_split")
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
    time.sleep(2.0)

    img = {"v": None}

    def on_img(msg):
        if img["v"] is None:
            img["v"] = bridge.imgmsg_to_cv2(msg, "bgr8")

    node.create_subscription(Image, "/rgb/image_raw", on_img, 10)
    t0 = time.time()
    while img["v"] is None and time.time() - t0 < 8:
        rclpy.spin_once(node, timeout_sec=0.05)
    if img["v"] is None:
        raise SystemExit("no rgb")

    rgb = img["v"]
    cv2.imwrite(f"{out}/split_rgb.png", rgb)
    mask, samples, usable, top, bias = segment_road(rgb, 0.40, 55, 5)
    cv2.imwrite(f"{out}/split_mask.png", mask)
    print(f"usable={usable} bias={bias:+.3f} samples={len(samples)} top={top}")
    if samples:
        ys = [s[0] for s in samples]
        cxs = [s[1] for s in samples]
        print(f"cx range {min(cxs)}..{max(cxs)} mean={sum(cxs)/len(cxs):.1f}")
        print(f"near cx={samples[-1][1]} far cx={samples[0][1]}")

    # luma / hsv stats in ROI halves
    h, w = rgb.shape[:2]
    band = rgb[top:, :]
    gray = cv2.cvtColor(band, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(band, cv2.COLOR_BGR2HSV)
    mid = w // 2
    y0 = int(band.shape[0] * 0.35)
    for name, sl in [("L", (slice(y0, None), slice(0, mid))), ("R", (slice(y0, None), slice(mid, None)))]:
        g = gray[sl]
        s = hsv[:, :, 1][sl]
        v = hsv[:, :, 2][sl]
        print(
            f"{name}: gray_med={np.median(g):.0f} v_med={np.median(v):.0f} "
            f"s_med={np.median(s):.0f} black%={(g < 55).mean()*100:.1f} "
            f"white%={((s < 60) & (v > 120)).mean()*100:.1f}"
        )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
