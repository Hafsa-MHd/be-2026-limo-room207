#!/usr/bin/env python3
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Float32MultiArray


def main():
    rclpy.init()
    node = rclpy.create_node("cap_vision")
    bridge = CvBridge()
    box = {"o": None, "r": None, "bias": None, "ok": None, "c": None}

    node.create_subscription(
        Image,
        "/vision/overlay",
        lambda m: box.__setitem__("o", bridge.imgmsg_to_cv2(m, "bgr8")),
        10,
    )
    node.create_subscription(
        Image,
        "/rgb/image_raw",
        lambda m: box.__setitem__("r", bridge.imgmsg_to_cv2(m, "bgr8")),
        10,
    )
    node.create_subscription(
        Float32, "/vision/black_bias", lambda m: box.__setitem__("bias", float(m.data)), 10
    )
    node.create_subscription(
        Bool, "/vision/road_ok", lambda m: box.__setitem__("ok", bool(m.data)), 10
    )
    node.create_subscription(
        Float32MultiArray,
        "/vision/center",
        lambda m: box.__setitem__("c", list(m.data)),
        10,
    )

    t0 = time.time()
    while time.time() - t0 < 6.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        if box["o"] is not None and box["r"] is not None and box["bias"] is not None:
            break

    print("bias", box.get("bias"), "ok", box.get("ok"), "center", box.get("c"))
    out = "/home/ha054/ros2_ws_simple/captures"
    if box["r"] is not None:
        cv2.imwrite(f"{out}/live_rgb.png", box["r"])
        print("saved live_rgb.png")
    if box["o"] is not None:
        cv2.imwrite(f"{out}/live_overlay.png", box["o"])
        print("saved live_overlay.png")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
