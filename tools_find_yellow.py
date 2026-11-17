#!/usr/bin/env python3
import cv2
import numpy as np

img = cv2.imread("/home/ha054/ros2_ws_simple/captures/split_rgb.png")
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# Search for yellowish pixels with various ranges
ranges = [
    ((10, 40, 40), (40, 255, 255)),
    ((15, 50, 50), (45, 255, 255)),
    ((5, 30, 80), (50, 255, 255)),
    ((0, 0, 0), (180, 255, 255)),  # all — just for hue hist of mid band
]
for lo, hi in ranges[:3]:
    m = cv2.inRange(hsv, lo, hi)
    print(f"range {lo}-{hi}: n={np.count_nonzero(m)}")

# Hue histogram where saturation is decent in mid image
band = hsv[180:320, 80:300]
mask = band[:, :, 1] > 40
hues = band[:, :, 0][mask]
print(f"sat>40 pixels in left-mid: {hues.size}")
if hues.size:
    hist, bins = np.histogram(hues, bins=18, range=(0, 180))
    for i, c in enumerate(hist):
        if c > 0:
            print(f"  hue {bins[i]:.0f}-{bins[i+1]:.0f}: {c}")

# Sample a few pixels along expected yellow line area by scanning for local max sat
sat = hsv[:, :, 1]
val = hsv[:, :, 2]
# among dark-ish neighbors, find colored pixels
for y in range(200, 320, 10):
    row_s = sat[y, 50:350]
    row_h = hsv[y, 50:350, 0]
    row_v = val[y, 50:350]
    idx = np.argmax(row_s)
    if row_s[idx] > 30:
        x = 50 + int(idx)
        print(f"y={y} maxS at x={x}: H={hsv[y,x,0]} S={hsv[y,x,1]} V={hsv[y,x,2]} BGR={img[y,x].tolist()}")
