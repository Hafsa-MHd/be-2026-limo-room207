#!/usr/bin/env python3
"""Verify parking + destinations respect black-only consignes, and control schema."""

import ast
import math
import re
from pathlib import Path

import cv2
import numpy as np

TEX = "/home/ha054/ros2_ws/src/environment/qcar_large_track/models/qcar_large_track/materials/textures/SDCS_MapLayout.png"
SX, SY = 4.88, 6.2
ROOT = Path(__file__).resolve().parent


def w2p(x, y, w, h):
    return int(round((x / SX + 0.5) * (w - 1))), int(round((0.5 - y / SY) * (h - 1)))


def sample(gray, x, y, w, h, rad=8):
    c, r = w2p(x, y, w, h)
    win = gray[max(0, r - rad) : r + rad + 1, max(0, c - rad) : c + rad + 1]
    if win.size == 0:
        return 255, 1.0
    return float(np.median(win)), float((win > 160).mean())


def main():
    img = cv2.imread(TEX)
    h, w = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    mission = (ROOT / "tools_mission_destinations.py").read_text(encoding="utf-8")
    m = re.search(r"PARKING\s*=\s*\(([^)]+)\)", mission)
    park = tuple(float(x.strip()) for x in m.group(1).split(","))
    dests = re.findall(
        r'\("([^"]+)",\s*([-\d.]+),\s*([-\d.]+),',
        mission,
    )

    print("=== PARKING ===")
    med, white = sample(gray, park[0], park[1], w, h)
    park_ok = med < 50 and white < 0.12
    print(f"  pose=({park[0]:.2f},{park[1]:.2f},yaw={park[2]:.2f})")
    print(f"  gray_med={med:.1f} white_nb={white:.2f}  => {'OK' if park_ok else 'FAIL'}")
    # Must NOT be the old outer-road point
    if abs(park[0] + 1.40) < 0.05 and abs(park[1] - 2.90) < 0.05:
        print("  FAIL: still using old outer-road parking (-1.40, 2.90)")
        park_ok = False

    print("=== DESTINATIONS (must be deep black) ===")
    dest_ok = True
    for name, xs, ys in dests:
        x, y = float(xs), float(ys)
        med, white = sample(gray, x, y, w, h, rad=10)
        d = math.hypot(x - park[0], y - park[1])
        ok = med < 55 and white < 0.10 and d >= 1.5
        print(
            f"  {name}: ({x:.2f},{y:.2f}) d={d:.2f}m gray={med:.0f} white={white:.2f} => {'OK' if ok else 'FAIL'}"
        )
        dest_ok = dest_ok and ok

    print("=== CONTROL SCHEMA (FIGURE 3) ===")
    polar = (ROOT / "limo_simple/limo_simple/polar_follow.py").read_text(encoding="utf-8")
    checks = {
        "hard_stop_if_not_road_ok": "not self._ok" in polar and "map_ok" in polar,
        "trajectory_from_vision": "_vision_goal_base()" in polar and "dx, dy = self._vision_goal_base()" in polar,
        "mission_only_nudges": "mission_nudge_max_rad" in polar,
        "no_ball_without_mission": "self._mission is not None" in polar
        and "gazebo_goal_ball" in polar,
        "ball_at_mission_not_lookahead": "self._mission[0], self._mission[1]" in polar,
    }
    schema_ok = True
    for k, v in checks.items():
        print(f"  {k}: {'OK' if v else 'FAIL'}")
        schema_ok = schema_ok and v

    # Debug image
    vis = img.copy()
    c, r = w2p(park[0], park[1], w, h)
    cv2.circle(vis, (c, r), 16, (0, 255, 0), 3)
    cv2.putText(vis, "PARK", (c + 10, r), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    for name, xs, ys in dests:
        x, y = float(xs), float(ys)
        c, r = w2p(x, y, w, h)
        cv2.circle(vis, (c, r), 10, (0, 0, 255), -1)
    out = "/home/ha054/ros2_ws_simple/captures/verify_consignes.png"
    cv2.imwrite(out, vis)
    print("saved", out)

    all_ok = park_ok and dest_ok and schema_ok
    print("\nRESULT", "PASS — consignes respected" if all_ok else "FAIL — fix needed")
    raise SystemExit(0 if all_ok else 2)


if __name__ == "__main__":
    main()
