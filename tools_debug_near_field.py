#!/usr/bin/env python3
"""Diagnose why road_ok is false at parking after near-field gate."""
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from limo_simple.road_vision import segment_road, _near_field_on_black  # noqa: E402


def main():
    rclpy.init()
    n = rclpy.create_node("dbg_near")
    bridge = CvBridge()
    box = {"img": None}

    def cb(msg):
        if box["img"] is None:
            box["img"] = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    n.create_subscription(Image, "/rgb/image_raw", cb, 10)
    t0 = time.time()
    while box["img"] is None and time.time() - t0 < 8:
        rclpy.spin_once(n, timeout_sec=0.1)
    if box["img"] is None:
        raise SystemExit("no camera image")

    image = box["img"]
    mask, samples, usable, top, bias = segment_road(
        image, 0.40, 55, 5, True, True, 120, 60
    )
    h = image.shape[0]
    band_h = h - top
    # Recompute black/white for near-field stats
    band = image[top:, :]
    gray = cv2.GaussianBlur(cv2.cvtColor(band, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    hsv = cv2.cvtColor(band, cv2.COLOR_BGR2HSV)
    black = (gray < 55).astype(np.uint8) * 255
    white = ((hsv[:, :, 1] < 60) & (hsv[:, :, 2] > 120)).astype(np.uint8) * 255
    lane = mask[top:, :]
    near_ok = _near_field_on_black(lane, black, white)
    near = max(4, int(band_h * 0.22))
    print(
        "usable=%s near_ok=%s samples=%d lane_px=%d bias=%.3f"
        % (usable, near_ok, len(samples), int(np.count_nonzero(lane)), bias)
    )
    print(
        "near lane/black/white = %d / %d / %d"
        % (
            int(np.count_nonzero(lane[-near:, :])),
            int(np.count_nonzero(black[-near:, :])),
            int(np.count_nonzero(white[-near:, :])),
        )
    )
    out = "/home/ha054/ros2_ws_simple/captures/near_field_debug.png"
    panel = np.hstack(
        [
            image,
            cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR),
        ]
    )
    cv2.imwrite(out, panel)
    print("saved", out)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
