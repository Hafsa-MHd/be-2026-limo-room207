#!/usr/bin/env python3
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray

SX, SY = 4.88, 6.2
TEX = (
    "/home/ha054/ros2_ws/src/environment/qcar_large_track/models/"
    "qcar_large_track/materials/textures/SDCS_MapLayout.png"
)


def map_gray(img, x, y):
    h, w = img.shape
    col = int(round((x / SX + 0.5) * (w - 1)))
    row = int(round((0.5 - y / SY) * (h - 1)))
    if not (2 <= col < w - 2 and 2 <= row < h - 2):
        return None
    return float(np.median(img[row - 2 : row + 3, col - 2 : col + 3]))


def main():
    rclpy.init()
    n = rclpy.create_node("chk_violation_fix")
    st = {"o": None, "ok": None, "c": None}
    n.create_subscription(Odometry, "/odom", lambda m: st.__setitem__("o", m), 10)
    n.create_subscription(Bool, "/vision/road_ok", lambda m: st.__setitem__("ok", m.data), 10)
    n.create_subscription(
        Float32MultiArray, "/vision/center", lambda m: st.__setitem__("c", len(m.data)), 10
    )
    pub = n.create_publisher(Twist, "/cmd_vel", 10)
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(n, timeout_sec=0.05)
        pub.publish(Twist())

    o = st["o"]
    if o is not None:
        p = o.pose.pose.position
        print("odom=(%.2f,%.2f)" % (p.x, p.y))
        gx = map_gray(cv2.imread(TEX, 0), p.x, p.y)
        print("map_at_odom gray=%.1f on_black=%s" % (gx, gx is not None and gx <= 90))
    print("road_ok", st["ok"], "center_len", st["c"])

    img = cv2.imread(TEX, 0)
    for name, x, y in (
        ("parking", -1.15, 1.45),
        ("island_guess", -0.3, 0.6),
        ("white_block", 0.2, 0.3),
        ("goalish", 1.5, -0.5),
    ):
        g = map_gray(img, x, y)
        print("map %s=(%.2f,%.2f) gray=%s" % (name, x, y, None if g is None else "%.1f" % g))

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
