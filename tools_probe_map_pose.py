#!/usr/bin/env python3
import cv2
import numpy as np

TEX = (
    "/home/ha054/ros2_ws/src/environment/qcar_large_track/models/"
    "qcar_large_track/materials/textures/SDCS_MapLayout.png"
)
SX, SY = 4.88, 6.2
img = cv2.imread(TEX, 0)
h, w = img.shape
print("map", w, h)

for name, x, y in [
    ("now", -0.55, 0.674),
    ("park", -1.15, 1.45),
    ("oldpark", -1.40, 2.90),
    ("island", -0.3, 0.6),
]:
    col = int(round((x / SX + 0.5) * (w - 1)))
    row = int(round((0.5 - y / SY) * (h - 1)))
    patch = img[max(0, row - 3) : row + 4, max(0, col - 3) : col + 4]
    med = float(np.median(patch)) if patch.size else -1
    # eroded black core
    black = (img < 90).astype(np.uint8) * 255
    core = cv2.erode(black, np.ones((9, 9), np.uint8), iterations=2)
    on_core = bool(core[row, col] > 0) if 0 <= row < h and 0 <= col < w else False
    print(
        "%s (%.3f,%.3f) pix=(%d,%d) gray=%.1f on_core=%s"
        % (name, x, y, col, row, med, on_core)
    )
