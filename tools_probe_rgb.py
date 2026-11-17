#!/usr/bin/env python3
"""Probe luma/HSV on split_rgb.png regions."""

import cv2
import numpy as np

path = "/home/ha054/ros2_ws_simple/captures/split_rgb.png"
img = cv2.imread(path)
h, w = img.shape[:2]
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
print(f"size {w}x{h}")

# Sample windows: left road (yellow lane), right white parking, near bumper
windows = {
    "left_road": (260, 180, 80, 60),  # x,y,w,h in full image
    "right_white": (420, 200, 120, 80),
    "near_center": (280, 380, 100, 60),
    "mid_left": (120, 250, 100, 80),
    "mid_right": (400, 250, 120, 80),
}
for name, (x, y, ww, hh) in windows.items():
    patch_g = gray[y : y + hh, x : x + ww]
    patch_h = hsv[y : y + hh, x : x + ww]
    print(
        f"{name}: gray[min/med/max]={patch_g.min()}/{np.median(patch_g):.0f}/{patch_g.max()} "
        f"V[med]={np.median(patch_h[:,:,2]):.0f} S[med]={np.median(patch_h[:,:,1]):.0f} "
        f"black%={(patch_g < 55).mean()*100:.1f} white%={((patch_h[:,:,1] < 60) & (patch_h[:,:,2] > 120)).mean()*100:.1f}"
    )

# Find brightest low-sat region (likely white parking)
sat = hsv[:, :, 1]
val = hsv[:, :, 2]
white = ((sat < 60) & (val > 120)).astype(np.uint8) * 255
# Restrict to lower 60% of image
white[: int(h * 0.35), :] = 0
ys, xs = np.where(white > 0)
if xs.size:
    print(f"white pixels in lower 65%: n={xs.size} x_mean={xs.mean():.0f} y_mean={ys.mean():.0f}")
    print(f"  x range {xs.min()}..{xs.max()} y {ys.min()}..{ys.max()}")
else:
    print("NO white pixels with current thresholds in lower 65%")
    # relax
    white2 = ((sat < 80) & (val > 90)).astype(np.uint8)
    white2[: int(h * 0.35), :] = 0
    ys, xs = np.where(white2 > 0)
    print(f"relaxed white n={xs.size} x_mean={xs.mean() if xs.size else -1:.0f}")

cv2.imwrite("/home/ha054/ros2_ws_simple/captures/split_white_mask.png", white)
