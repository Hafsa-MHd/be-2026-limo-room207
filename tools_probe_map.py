#!/usr/bin/env python3
import cv2

TEX = (
    "/home/ha054/ros2_ws/src/environment/qcar_large_track/models/"
    "qcar_large_track/materials/textures/SDCS_MapLayout.png"
)
img = cv2.imread(TEX, 0)
h, w = img.shape
SX, SY = 4.88, 6.2


def sample(x, y):
    c = int(round((x / SX + 0.5) * (w - 1)))
    r = int(round((0.5 - y / SY) * (h - 1)))
    if not (0 <= c < w and 0 <= r < h):
        return -1
    return int(img[r, c])


# Scan northern corridor y=2.0..2.8, x=-2..2
print("=== north strip ===")
for y in [2.0, 2.2, 2.4, 2.5, 2.6, 2.7, 2.8]:
    row = []
    for x in [-2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.2, 1.4, 1.5, 1.6, 1.8, 2.0]:
        g = sample(x, y)
        row.append("%+.1f:%s" % (x, "B" if g <= 90 else ("W" if g >= 200 else str(g))))
    print("y=%.1f" % y, " ".join(row))

print("=== east descent toward D8 ===")
for y in [2.5, 2.0, 1.5, 1.0, 0.5, 0.0, -0.5, -0.9]:
    row = []
    for x in [0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8]:
        g = sample(x, y)
        row.append("%+.1f:%s" % (x, "B" if g <= 90 else ("W" if g >= 200 else str(g))))
    print("y=%.1f" % y, " ".join(row))
