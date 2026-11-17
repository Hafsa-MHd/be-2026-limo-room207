#!/usr/bin/env python3
"""Locate the 3 rectangular parking stalls (comb) in the parking lot."""
import cv2
import numpy as np

TEX = (
    "/home/ha054/ros2_ws/src/environment/qcar_large_track/models/"
    "qcar_large_track/materials/textures/SDCS_MapLayout.png"
)
SX, SY = 4.88, 6.2


def p2w(c, r, w, h):
    return ((c / (w - 1) - 0.5) * SX, (0.5 - r / (h - 1)) * SY)


def w2p(x, y, w, h):
    return int(round((x / SX + 0.5) * (w - 1))), int(round((0.5 - y / SY) * (h - 1)))


def main():
    img = cv2.imread(TEX)
    h, w = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Parking lot region (left-middle black pad with stalls).
    c0, r0 = w2p(-2.1, 2.4, w, h)
    c1, r1 = w2p(-0.2, 0.2, w, h)
    c0, c1 = sorted([c0, c1])
    r0, r1 = sorted([r0, r1])
    roi = gray[r0:r1, c0:c1]

    black = (roi < 55).astype(np.uint8) * 255
    white = (roi > 170).astype(np.uint8) * 255
    # Stall paint thickened → pockets of black surrounded by white
    white_d = cv2.dilate(white, np.ones((3, 3), np.uint8), iterations=1)
    # Black cells that are near white paint (= inside marked bays)
    near = cv2.bitwise_and(black, cv2.dilate(white_d, np.ones((15, 15), np.uint8), iterations=1))
    near = cv2.erode(near, np.ones((5, 5), np.uint8), iterations=1)

    n, lab, st, cent = cv2.connectedComponentsWithStats(near, 8)
    cands = []
    for i in range(1, n):
        area = int(st[i, cv2.CC_STAT_AREA])
        bw = int(st[i, cv2.CC_STAT_WIDTH])
        bh = int(st[i, cv2.CC_STAT_HEIGHT])
        if area < 80 or area > 5000:
            continue
        # Prefer compact bay pockets (not the whole aisle)
        if max(bw, bh) > 90:
            continue
        cx = int(cent[i][0]) + c0
        cy = int(cent[i][1]) + r0
        x, y = p2w(cx, cy, w, h)
        # Focus on the right-side comb bays (higher x) inside lot
        if not (-1.8 < x < -0.4 and 0.4 < y < 2.2):
            continue
        g = int(gray[cy, cx])
        if g > 40:
            continue
        cands.append((area, x, y, cx, cy, bw, bh))

    cands.sort(reverse=True)
    print("STALL_CANDIDATES:")
    for a, x, y, cx, cy, bw, bh in cands[:12]:
        print(f"  ({x:.3f},{y:.3f}) area={a} box={bw}x{bh} gray={int(gray[cy,cx])}")

    # Pick 3 distinct bays (greedy by distance)
    chosen = []
    for a, x, y, cx, cy, bw, bh in cands:
        if any((x - ox) ** 2 + (y - oy) ** 2 < 0.20**2 for ox, oy, *_ in chosen):
            continue
        chosen.append((x, y, cx, cy, a))
        if len(chosen) >= 3:
            break

    print("CHOSEN_3_BAYS:")
    for i, (x, y, cx, cy, a) in enumerate(chosen, 1):
        print(f"  B{i}: ({x:.3f},{y:.3f})")

    # Preferred start: middle bay, yaw toward aisle exit (roughly +x / south-east)
    if chosen:
        # sort by y and take middle
        chosen_y = sorted(chosen, key=lambda t: t[1])
        mid = chosen_y[len(chosen_y) // 2]
        x, y = mid[0], mid[1]
        yaw = -0.85  # toward exit / road
        print(f"SPAWN_PARKING {x:.4f} {y:.4f} {yaw:.4f}")
    else:
        # fallback known deep bay
        print("SPAWN_PARKING -1.1500 1.4500 -0.8500")

    vis = img.copy()
    for x, y, cx, cy, a in chosen:
        cv2.circle(vis, (cx, cy), 12, (0, 255, 0), 2)
    sc, sr = w2p(-0.725, -0.087, w, h)
    cv2.circle(vis, (sc, sr), 12, (0, 0, 255), 2)
    cv2.putText(vis, "SCHOOL", (sc + 8, sr), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    out = "/home/ha054/ros2_ws_simple/captures/stall_bays.png"
    cv2.imwrite(out, vis)
    print("saved", out)


if __name__ == "__main__":
    main()
