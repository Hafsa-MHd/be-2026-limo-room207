#!/usr/bin/env python3
"""Find the Quanser diagonal parking-bay spawn (not outer road, not bottom bays)."""

import cv2
import numpy as np

TEX = "/home/ha054/ros2_ws/src/environment/qcar_large_track/models/qcar_large_track/materials/textures/SDCS_MapLayout.png"
SX, SY = 4.88, 6.2


def p2w(c, r, w, h):
    return ((c / (w - 1) - 0.5) * SX, (0.5 - r / (h - 1)) * SY)


def w2p(x, y, w, h):
    return int(round((x / SX + 0.5) * (w - 1))), int(round((0.5 - y / SY) * (h - 1)))


def main():
    img = cv2.imread(TEX)
    h, w = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Diagonal parking is in the left-middle black "lot" (not top outer loop, not bottom 4 bays).
    # Approximate from Quanser layout: x in [-2.2,-0.3], y in [0.3, 2.2]
    c0, r0 = w2p(-2.2, 2.2, w, h)
    c1, r1 = w2p(-0.3, 0.3, w, h)
    c0, c1 = sorted([c0, c1])
    r0, r1 = sorted([r0, r1])
    roi = gray[r0:r1, c0:c1]
    # Stall lines: bright
    stall = ((roi > 170) & (roi < 256)).astype(np.uint8) * 255
    # Prefer diagonal strokes: morphological open with diagonal-ish kernel is hard;
    # use connected components of stall and keep elongated ones.
    n, lab, st, _ = cv2.connectedComponentsWithStats(stall, 8)
    mask = np.zeros_like(stall)
    for i in range(1, n):
        ww, hh, area = st[i, cv2.CC_STAT_WIDTH], st[i, cv2.CC_STAT_HEIGHT], st[i, cv2.CC_STAT_AREA]
        if area < 40:
            continue
        # diagonal stalls are elongated
        if max(ww, hh) > 18 and area > 80:
            mask[lab == i] = 255

    # Black asphalt near stalls = parking bay interiors
    black = (roi < 60).astype(np.uint8) * 255
    near = cv2.dilate(mask if mask.any() else stall, np.ones((11, 11), np.uint8), iterations=2)
    bay = cv2.bitwise_and(black, near)
    # erode to center of bay
    bay = cv2.erode(bay, np.ones((5, 5), np.uint8), iterations=1)

    ys, xs = np.where(bay > 0)
    print(f"roi=({c0},{r0})-({c1},{r1}) stall={np.count_nonzero(stall)} bay={xs.size}")
    if xs.size < 50:
        # fallback: all black in roi eroded
        bay = cv2.erode(black, np.ones((9, 9), np.uint8), iterations=2)
        ys, xs = np.where(bay > 0)
        print("fallback bay", xs.size)

    # Cluster: k-means-ish via histogram peaks → several bay centers
    H, xe, ye = np.histogram2d(xs, ys, bins=[12, 12])
    peaks = []
    for _ in range(6):
        ix, iy = np.unravel_index(np.argmax(H), H.shape)
        if H[ix, iy] < 5:
            break
        c = int(0.5 * (xe[ix] + xe[ix + 1])) + c0
        r = int(0.5 * (ye[iy] + ye[iy + 1])) + r0
        x, y = p2w(c, r, w, h)
        peaks.append((H[ix, iy], x, y, c, r))
        H[max(0, ix - 1) : ix + 2, max(0, iy - 1) : iy + 2] = 0

    print("BAY_CENTERS (best first):")
    for count, x, y, c, r in peaks:
        print(f"  ({x:.3f}, {y:.3f}) weight={count:.0f} gray={int(gray[r,c])}")

    # Recommended spawn: bay center closest to lot exit (toward +x / -y from lot)
    # Exit of Quanser parking is typically southeast of the stalls.
    if peaks:
        # pick a bay not on the edge: mid of list or highest weight with y~1.5-2.0
        best = None
        for count, x, y, c, r in peaks:
            if -1.9 < x < -0.7 and 0.8 < y < 2.1:
                best = (x, y, c, r)
                break
        if best is None:
            _, x, y, c, r = peaks[0]
            best = (x, y, c, r)
        print("RECOMMENDED_PARKING", best[0], best[1])
        # yaw toward exit: roughly toward south-east / along aisle
        print("RECOMMENDED_YAW", -0.9, "(toward parking exit / road)")

    vis = img.copy()
    for count, x, y, c, r in peaks:
        cv2.circle(vis, (c, r), 10, (0, 255, 255), -1)
    if peaks:
        cv2.circle(vis, (best[2], best[3]), 16, (0, 255, 0), 3)
        cv2.putText(vis, "PARK", (best[2] + 10, best[3]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    # school road spawn
    sc, sr = w2p(-0.725, -0.087, w, h)
    cv2.circle(vis, (sc, sr), 12, (255, 0, 0), 2)
    cv2.putText(vis, "SCHOOL_ROAD", (sc + 8, sr), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    out = "/home/ha054/ros2_ws_simple/captures/true_parking.png"
    cv2.imwrite(out, vis)
    print("saved", out)


if __name__ == "__main__":
    main()
