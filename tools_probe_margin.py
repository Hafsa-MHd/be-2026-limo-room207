#!/usr/bin/env python3
"""How close is current pose to white? Need stronger margin."""
import cv2
import numpy as np

TEX = (
    "/home/ha054/ros2_ws/src/environment/qcar_large_track/models/"
    "qcar_large_track/materials/textures/SDCS_MapLayout.png"
)
SX, SY = 4.88, 6.2
img = cv2.imread(TEX, 0)
h, w = img.shape
black = (img < 90).astype(np.uint8) * 255
bright = (img > 160).astype(np.uint8)
# distance to white in pixels
inv = (bright == 0).astype(np.uint8) * 255
dist = cv2.distanceTransform(inv, cv2.DIST_L2, 5)

for name, x, y in [
    ("now", -0.55, 0.674),
    ("park_old", -1.15, 1.45),
    ("official_spawn", -0.73, -0.09),
    ("island", -0.3, 0.6),
]:
    col = int(round((x / SX + 0.5) * (w - 1)))
    row = int(round((0.5 - y / SY) * (h - 1)))
    g = float(img[row, col])
    d_px = float(dist[row, col])
    # meters per pixel
    mpp = SX / (w - 1)
    d_m = d_px * mpp
    for erode_iter in (1, 2, 3, 4):
        core = cv2.erode(black, np.ones((9, 9), np.uint8), iterations=erode_iter)
        ok = bool(core[row, col] > 0)
        print(
            "%s gray=%.0f dist_white=%.1fpx (%.2fm) erode%d=%s"
            % (name, g, d_px, d_m, erode_iter, ok)
        )
    print("---")
