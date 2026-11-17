#!/usr/bin/env python3
"""Pick spawn INSIDE the diagonal white-stripe parking spots (not the lot curve)."""
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

# Diagonal stall band (upper parking pad) — from user screenshot.
xs = np.linspace(-1.95, -1.05, 60)
ys = np.linspace(1.90, 2.55, 50)
cands = []
for x in xs:
    for y in ys:
        c, r = w2p(float(x), float(y), w, h)
        if not (10 <= c < w - 10 and 10 <= r < h - 10):
            continue
        patch = gray[r - 2 : r + 3, c - 2 : c + 3]
        med = float(np.median(patch))
        if med > 20:
            continue  # stall floor must be black
        ring = gray[r - 10 : r + 11, c - 10 : c + 11]
        wf = float((ring > 170).mean())
        if wf < 0.18:  # must sit among diagonal paint
            continue
        # Prefer deeper into stripes (higher y), away from school curve
        score = wf + 0.15 * (y - 1.9) - 0.02 * abs(x + 1.5)
        cands.append((score, med, wf, float(x), float(y), c, r))

cands.sort(reverse=True)
print("DIAGONAL_STALL candidates:")
chosen = []
for score, med, wf, x, y, c, r in cands:
    if any((x - ox) ** 2 + (y - oy) ** 2 < 0.15**2 for ox, oy in chosen):
        continue
    chosen.append((x, y))
    print(f"  ({x:.3f},{y:.3f}) gray={med:.0f} white_nb={wf:.2f} score={score:.2f}")
    if len(chosen) >= 8:
        break

# Compare known poses
for name, x, y in [
    ("school", -0.725, -0.087),
    ("old_curve", -1.58, 1.96),
    ("aisle", -1.15, 1.45),
]:
    c, r = w2p(x, y, w, h)
    med = float(np.median(gray[r - 2 : r + 3, c - 2 : c + 3]))
    ring = gray[r - 10 : r + 11, c - 10 : c + 11]
    wf = float((ring > 170).mean())
    print(f"REF {name}=({x:.3f},{y:.3f}) gray={med:.0f} white_nb={wf:.2f}")

if chosen:
    # Middle of top candidates — yaw face out of stall toward aisle (~south-east)
    x, y = chosen[0]
    yaw = -1.40
    print(f"SPAWN_DIAGONAL {x:.4f} {y:.4f} {yaw:.4f}")
else:
    print("SPAWN_DIAGONAL -1.4900 2.2000 -1.4000")

vis = img.copy()
for x, y in chosen:
    c, r = w2p(x, y, w, h)
    cv2.circle(vis, (c, r), 9, (0, 255, 0), 2)
for name, x, y, col in [
    ("S", -0.725, -0.087, (0, 0, 255)),
    ("C", -1.58, 1.96, (0, 255, 255)),
]:
    c, r = w2p(x, y, w, h)
    cv2.circle(vis, (c, r), 11, col, 2)
    cv2.putText(vis, name, (c + 8, r), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
if chosen:
    c, r = w2p(chosen[0][0], chosen[0][1], w, h)
    cv2.circle(vis, (c, r), 16, (0, 255, 0), 3)
    cv2.putText(vis, "PARK", (c + 12, r - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
cv2.imwrite("/home/ha054/ros2_ws_simple/captures/diagonal_stall.png", vis)
print("saved captures/diagonal_stall.png")
