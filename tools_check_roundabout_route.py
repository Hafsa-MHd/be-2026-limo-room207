#!/usr/bin/env python3
"""Check whether TrackPlanner finds a route to the roundabout showcase goal."""
import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "limo_simple"))

from limo_simple.track_planner import TrackPlanner, installed_track_texture  # noqa: E402

TEX = installed_track_texture()
img = cv2.imread(str(TEX), 0)
planner = TrackPlanner(img, 4.88, 6.2)
park = (-0.725347, -0.086613, 1.5708)

# Probe goals around the NE ring.
CX, CY = 1.44, 2.11
cands = [(2.023, 2.322, "current")]
for radius in (0.48, 0.55, 0.62, 0.70):
    for ang in (0, 20, 45, 90, 135, 180, 225, 270, 315):
        a = math.radians(ang)
        x = CX + radius * math.cos(a)
        y = CY + radius * math.sin(a)
        cands.append((x, y, f"r{radius:.2f}_a{ang}"))

print(f"texture={TEX}")
for gx, gy, name in cands:
    on_road = planner._road(gx, gy)
    route = planner.plan(park, (gx, gy))
    if route is None:
        print(f"{name:16s} ({gx:6.3f},{gy:6.3f}) road={on_road} NO_ROUTE")
    else:
        print(f"{name:16s} ({gx:6.3f},{gy:6.3f}) road={on_road} OK n={len(route)}")
