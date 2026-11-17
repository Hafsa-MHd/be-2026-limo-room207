#!/usr/bin/env python3
"""Pick a point INSIDE a diagonal parking stall (black, near paint)."""
import cv2
import numpy as np

TEX = (
    "/home/ha054/ros2_ws/src/environment/qcar_large_track/models/"
    "qcar_large_track/materials/textures/SDCS_MapLayout.png"
)
SX, SY = 4.88, 6.2


def w2p(x, y, w, h):
    return int(round((x / SX + 0.5) * (w - 1))), int(round((0.5 - y / SY) * (h - 1)))


def p2w(c, r, w, h):
    return ((c / (w - 1) - 0.5) * SX, (0.5 - r / (h - 1)) * SY)


img = cv2.imread(TEX)
h, w = img.shape[:2]
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# Diagonal stall band (upper parking pad)
xs = np.linspace(-1.95, -1.05, 50)
ys = np.linspace(1.55, 2.25, 40)
cands = []
for x in xs:
    for y in ys:
        c, r = w2p(float(x), float(y), w, h)
        patch = gray[r - 3 : r + 4, c - 3 : c + 4]
        if patch.size == 0:
            continue
        med = float(np.median(patch))
        if med > 25:
            continue  # must be black asphalt of stall floor
        ring = gray[r - 12 : r + 13, c - 12 : c + 13]
        wf = float((ring > 170).mean())
        if wf < 0.12:
            continue  # must be near stall paint
        # distance to school spawn — want clearly different
        d_school = ((x + 0.725) ** 2 + (y + 0.087) ** 2) ** 0.5
        d_aisle = ((x + 1.15) ** 2 + (y - 1.45) ** 2) ** 0.5
        cands.append((wf, med, d_school, d_aisle, float(x), float(y), c, r))

cands.sort(key=lambda t: (-t[0], t[1]))
print("IN_STALL candidates:")
chosen = []
for wf, med, ds, da, x, y, c, r in cands:
    if any((x - ox) ** 2 + (y - oy) ** 2 < 0.18**2 for ox, oy in chosen):
        continue
    chosen.append((x, y))
    print(f"  ({x:.3f},{y:.3f}) gray={med:.0f} white_nb={wf:.2f} d_school={ds:.2f} d_aisle={da:.2f}")
    if len(chosen) >= 6:
        break

# Prefer mid stall, facing aisle exit (toward +x / -y roughly)
if chosen:
    # pick one with good white_nb not on edge
    x, y = chosen[min(1, len(chosen) - 1)]
    yaw = -1.20  # face out of stall toward aisle/exit
    print(f"SPAWN_IN_STALL {x:.4f} {y:.4f} {yaw:.4f}")
else:
    print("SPAWN_IN_STALL -1.4500 1.8500 -1.2000")

vis = img.copy()
for x, y in chosen:
    c, r = w2p(x, y, w, h)
    cv2.circle(vis, (c, r), 10, (0, 255, 0), 2)
sc, sr = w2p(-0.725, -0.087, w, h)
cv2.circle(vis, (sc, sr), 12, (0, 0, 255), 2)
ac, ar = w2p(-1.15, 1.45, w, h)
cv2.circle(vis, (ac, ar), 12, (255, 255, 0), 2)
cv2.putText(vis, "SCHOOL", (sc + 8, sr), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
cv2.putText(vis, "AISLE", (ac + 8, ar), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
cv2.imwrite("/home/ha054/ros2_ws_simple/captures/in_stall.png", vis)
print("saved captures/in_stall.png")
