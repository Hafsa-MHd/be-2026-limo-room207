#!/usr/bin/env python3
"""Find narrow black corridors (tight spots) on the map."""
import cv2
import numpy as np

TEX = (
    "/home/ha054/ros2_ws/src/environment/qcar_large_track/models/"
    "qcar_large_track/materials/textures/SDCS_MapLayout.png"
)
img = cv2.imread(TEX, 0)
h, w = img.shape
SX, SY = 4.88, 6.2
black = (img <= 90).astype(np.uint8)


def sample(x, y):
    c = int(round((x / SX + 0.5) * (w - 1)))
    r = int(round((0.5 - y / SY) * (h - 1)))
    if not (0 <= c < w and 0 <= r < h):
        return -1
    return int(img[r, c])


def width_at(x, y, axis="x"):
    """Approx black half-width in meters along axis."""
    if sample(x, y) > 90:
        return 0.0
    step = 0.02
    a = 0.0
    while a < 0.8 and sample(x - (step if axis == "x" else 0), y - (0 if axis == "x" else step)) <= 90:
        a += step
    b = 0.0
    while b < 0.8 and sample(x + (step if axis == "x" else 0), y + (0 if axis == "x" else step)) <= 90:
        b += step
    return a + b


print("narrow probes (width_x, width_y):")
for y in np.arange(-1.2, 3.0, 0.2):
    for x in np.arange(-2.0, 2.2, 0.2):
        if sample(float(x), float(y)) > 90:
            continue
        wx = width_at(float(x), float(y), "x")
        wy = width_at(float(x), float(y), "y")
        m = min(wx, wy)
        if 0.05 < m < 0.45:
            print(f"  tight ({x:.1f},{y:.1f}) wx={wx:.2f} wy={wy:.2f}")

print("\nlive area around (1.25,2.70):")
for y in [2.4, 2.5, 2.6, 2.7, 2.8, 2.9]:
    row = []
    for x in [0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8]:
        g = sample(x, y)
        row.append(f"{x:.1f}:{'B' if g<=90 else 'W'}")
    print(f"y={y:.1f}", " ".join(row), "isl-ish")
