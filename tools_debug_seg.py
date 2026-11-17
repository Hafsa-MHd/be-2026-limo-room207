#!/usr/bin/env python3
import cv2
import numpy as np

img = cv2.imread("/home/ha054/ros2_ws_simple/captures/split_rgb.png")
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# True visible yellow: decent brightness
for vmin in [5, 20, 40, 60, 80]:
    m = cv2.inRange(hsv, (15, 60, vmin), (40, 255, 255))
    n = np.count_nonzero(m)
    if n:
        ys, xs = np.where(m > 0)
        print(f"V>={vmin}: n={n} x_med={np.median(xs):.0f} y_med={np.median(ys):.0f}")
    else:
        print(f"V>={vmin}: n=0")

# Also show brightest yellow-ish
m = cv2.inRange(hsv, (10, 40, 40), (45, 255, 255))
ys, xs = np.where(m > 0)
print("visible yellowish n", xs.size)
if xs.size:
    # show some pixels
    idx = np.linspace(0, xs.size - 1, num=min(8, xs.size), dtype=int)
    for i in idx:
        x, y = int(xs[i]), int(ys[i])
        print(f"  ({x},{y}) BGR={img[y,x].tolist()} HSV={hsv[y,x].tolist()}")
