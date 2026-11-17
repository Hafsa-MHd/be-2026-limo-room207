#!/usr/bin/env python3
import cv2
import numpy as np
from limo_simple.road_vision import segment_road

img = cv2.imread("/home/ha054/ros2_ws_simple/captures/split_rgb.png")
mask, samples, usable, top, bias = segment_road(img, 0.40, 55, 5)
cxs = [s[1] for s in samples]
mean_cx = sum(cxs) / max(1, len(cxs))
print(f"bias={bias:+.3f} usable={usable} n={len(samples)} mean_cx={mean_cx:.1f}")
if samples:
    n = len(samples)
    for label, idx in [("top", 0), ("mid", n // 2), ("bot", n - 1)]:
        y, cx, left, right = samples[idx]
        print(f"  {label}: y={y} cx={cx} L={left} R={right}")

vis = img.copy()
vis[mask > 0] = (0, 180, 0)
cv2.imwrite("/home/ha054/ros2_ws_simple/captures/split_debug.png", vis)
cv2.imwrite("/home/ha054/ros2_ws_simple/captures/split_mask_new.png", mask)

# Success for this known frame: yellow-marked black lane is left-of-center,
# not the far-left white border and not the right parking slab.
ok = 80 <= mean_cx <= 300 and bias < 0.05
print(f"RESULT={'OK prefer yellow/black lane' if ok else 'FAIL'}")
