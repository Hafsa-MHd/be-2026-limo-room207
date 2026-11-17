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


def sample(x, y):
    c = int(round((x / SX + 0.5) * (w - 1)))
    r = int(round((0.5 - y / SY) * (h - 1)))
    patch = img[r - 2 : r + 3, c - 2 : c + 3]
    med = float(np.median(patch))
    ring = img[r - 8 : r + 9, c - 8 : c + 9]
    wf = float((ring > 170).mean())
    return med, wf, c, r


print("scan x=-1.32, y from 0.5 to 2.8:")
for y in np.linspace(0.5, 2.8, 24):
    med, wf, c, r = sample(-1.32, float(y))
    tag = "BLACK" if med < 40 else ("WHITE" if med > 160 else "GRAY")
    print(f"  y={y:.2f} gray={med:5.1f} white_nb={wf:.2f} {tag}")

print("\nscan around diagonal zone:")
for x, y in [
    (-1.32, 2.24),
    (-1.50, 2.10),
    (-1.40, 2.00),
    (-1.20, 1.80),
    (-1.58, 1.96),
    (-0.73, -0.09),
    (-1.00, 1.20),
]:
    med, wf, c, r = sample(x, y)
    print(f"  ({x:.2f},{y:.2f}) gray={med:.0f} white_nb={wf:.2f}")
