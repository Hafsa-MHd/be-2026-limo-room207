#!/usr/bin/env python3
"""Pick a black-asphalt goal on the NE roundabout ring."""
import math
from pathlib import Path

import cv2

TEX = Path(
    "/home/ha054/ros2_ws/src/environment/qcar_large_track/models/"
    "qcar_large_track/materials/textures/SDCS_MapLayout.png"
)
if not TEX.is_file():
    try:
        from ament_index_python.packages import get_package_share_directory

        TEX = (
            Path(get_package_share_directory("qcar_large_track"))
            / "models/qcar_large_track/materials/textures/SDCS_MapLayout.png"
        )
    except Exception as exc:  # noqa: BLE001
        raise SystemExit(exc)

img = cv2.imread(str(TEX), 0)
h, w = img.shape
SX, SY = 4.88, 6.2
PARK = (-0.725347, -0.086613)
# White island center of the NE roundabout (from earlier map audit).
CX, CY = 1.44, 2.11


def sample(x, y):
    c = int(round((x / SX + 0.5) * (w - 1)))
    r = int(round((0.5 - y / SY) * (h - 1)))
    if not (0 <= c < w and 0 <= r < h):
        return 999
    return int(img[r, c])


def local_ok(x, y, rad=0.08):
    """Require a small neighborhood mostly black (on the ring, not island edge)."""
    n = 0
    bad = 0
    for dx in (-rad, -rad / 2, 0, rad / 2, rad):
        for dy in (-rad, -rad / 2, 0, rad / 2, rad):
            n += 1
            if sample(x + dx, y + dy) > 160:
                bad += 1
    return bad <= 4


cands = []
for radius in (0.40, 0.48, 0.55, 0.62, 0.70, 0.78):
    for ang in range(0, 360, 5):
        a = math.radians(ang)
        x = CX + radius * math.cos(a)
        y = CY + radius * math.sin(a)
        if sample(x, y) > 90:
            continue
        if not local_ok(x, y):
            continue
        dist = math.hypot(x - PARK[0], y - PARK[1])
        # Prefer a point that forces entering/crossing the ring (not the outer north strip only).
        # South-east / east arc of the circle is a good demo pose.
        cands.append((abs(radius - 0.58), abs(ang - 20) % 360, -dist, radius, ang, x, y))

cands.sort()
print(f"texture={TEX}")
print(f"candidates={len(cands)}")
print("TOP:")
for row in cands[:12]:
    _, _, _, radius, ang, x, y = row
    print(
        f"  r={radius:.2f} ang={ang:3d}  ({x:.3f}, {y:.3f})  "
        f"gray={sample(x, y)}  dist_park={math.hypot(x - PARK[0], y - PARK[1]):.2f}"
    )

# Explicit showcase pick: east/SE of ring center on black asphalt.
showcase = None
for row in cands:
    _, _, _, radius, ang, x, y = row
    # ~east to south-east of center → robot must use the ring corridor.
    if 330 <= ang or ang <= 50:
        showcase = (x, y, radius, ang)
        break
if showcase is None and cands:
    _, _, _, radius, ang, x, y = cands[0]
    showcase = (x, y, radius, ang)

if showcase:
    x, y, radius, ang = showcase
    print(f"\nCHOSEN_ROUNDABOUT_GOAL=({x:.3f}, {y:.3f}) r={radius:.2f} ang={ang}")
