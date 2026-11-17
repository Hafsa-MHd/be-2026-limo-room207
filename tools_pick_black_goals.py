#!/usr/bin/env python3
"""Pick far destination points that lie on BLACK asphalt only (from track texture)."""

import math

import cv2
import numpy as np

TEX = "/home/ha054/ros2_ws/src/environment/qcar_large_track/models/qcar_large_track/materials/textures/SDCS_MapLayout.png"
SX, SY = 4.88, 6.2  # Gazebo plane size (x, y)
PARK = (-1.40, 2.90)


def pix_to_world(col, row, w, h):
    x = (col / (w - 1) - 0.5) * SX
    y = (0.5 - row / (h - 1)) * SY
    return x, y


def world_to_pix(x, y, w, h):
    col = (x / SX + 0.5) * (w - 1)
    row = (0.5 - y / SY) * (h - 1)
    return int(round(col)), int(round(row))


def main():
    img = cv2.imread(TEX)
    h, w = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Black / dark asphalt (not white floor, not bright paint).
    black = (gray < 80).astype(np.uint8) * 255
    # Remove yellow lane paint from "navigable center" preference later — keep as road.
    # Erode so points are strictly inside black, away from white borders.
    # Strong erode: goals must sit deep inside black, far from white edges.
    kernel = np.ones((11, 11), np.uint8)
    core = cv2.erode(black, kernel, iterations=3)

    # Also reject very bright neighbors (near white islands).
    bright = (gray > 160).astype(np.uint8)
    near_white = cv2.dilate(bright, np.ones((21, 21), np.uint8), iterations=2)
    core[near_white > 0] = 0

    ys, xs = np.where(core > 0)
    print(f"texture {w}x{h} black_core_pixels={xs.size}")

    # Candidate seeds: grid sample of core black, keep far from parking.
    rng = np.random.default_rng(7)
    idx = rng.choice(xs.size, size=min(4000, xs.size), replace=False)
    pts = []
    for i in idx:
        x, y = pix_to_world(xs[i], ys[i], w, h)
        d = math.hypot(x - PARK[0], y - PARK[1])
        if d < 2.0:
            continue
        # Must still be black if we check a small window
        c, r = world_to_pix(x, y, w, h)
        if not (5 <= c < w - 5 and 5 <= r < h - 5):
            continue
        patch = gray[r - 3 : r + 4, c - 3 : c + 4]
        if patch.size == 0 or np.median(patch) > 70:
            continue
        if near_white[r, c]:
            continue
        pts.append((d, x, y, c, r))

    pts.sort(reverse=True)  # farthest first
    # Greedy diversity: keep points far from each other
    chosen = []
    for d, x, y, c, r in pts:
        if any(math.hypot(x - cx, y - cy) < 1.1 for _, cx, cy, _, _ in chosen):
            continue
        chosen.append((d, x, y, c, r))
        if len(chosen) >= 8:
            break

    print("PARKING", PARK)
    print("CHOSEN_BLACK_GOALS:")
    for i, (d, x, y, c, r) in enumerate(chosen, 1):
        print(f"  D{i}: ({x:.3f}, {y:.3f}) dist={d:.2f}m pix=({c},{r}) gray={int(gray[r,c])}")

    # Debug image
    vis = img.copy()
    pc, pr = world_to_pix(PARK[0], PARK[1], w, h)
    cv2.circle(vis, (pc, pr), 12, (0, 255, 0), 2)
    for i, (d, x, y, c, r) in enumerate(chosen, 1):
        cv2.circle(vis, (c, r), 10, (0, 0, 255), -1)
        cv2.putText(vis, f"D{i}", (c + 8, r), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    out = "/home/ha054/ros2_ws_simple/captures/black_goals.png"
    cv2.imwrite(out, vis)
    print("saved", out)

    # Emit python list
    print("\nDESTINATIONS = [")
    for i, (d, x, y, c, r) in enumerate(chosen, 1):
        print(
            f'    ("D{i}_black", {x:.3f}, {y:.3f}, "black asphalt {d:.2f}m from parking"),'
        )
    print("]")


if __name__ == "__main__":
    main()
