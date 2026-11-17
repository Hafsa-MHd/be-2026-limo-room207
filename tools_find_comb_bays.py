#!/usr/bin/env python3
"""Find the 3 horizontal parking bays (comb) next to the diagonal stalls."""
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

    # Grid-sample parking pad; score = black + nearby white paint + compact
    xs = np.linspace(-1.9, -0.35, 40)
    ys = np.linspace(0.35, 2.2, 40)
    hits = []
    for x in xs:
        for y in ys:
            c, r = w2p(x, y, w, h)
            if not (8 <= c < w - 8 and 8 <= r < h - 8):
                continue
            patch = gray[r - 6 : r + 7, c - 6 : c + 7]
            med = float(np.median(patch))
            if med > 35:
                continue
            # white paint nearby (stall lines)
            ring = gray[r - 14 : r + 15, c - 14 : c + 15]
            white_frac = float((ring > 170).mean())
            if white_frac < 0.04 or white_frac > 0.55:
                continue
            hits.append((white_frac, med, x, y, c, r))

    hits.sort(reverse=True)
    print(f"hits={len(hits)}")
    # cluster
    chosen = []
    for wf, med, x, y, c, r in hits:
        if any((x - ox) ** 2 + (y - oy) ** 2 < 0.22**2 for _, _, ox, oy, _, _ in chosen):
            continue
        chosen.append((wf, med, x, y, c, r))
        if len(chosen) >= 10:
            break

    print("TOP_BAY_LIKE:")
    for wf, med, x, y, c, r in chosen:
        print(f"  ({x:.3f},{y:.3f}) white_nb={wf:.2f} gray={med:.0f}")

    # Prefer the comb on the RIGHT of the lot (larger x), mid y
    right = [t for t in chosen if t[2] > -1.2]
    pool = right if len(right) >= 2 else chosen
    # take three closest to a vertical stack (same x band)
    best_spawn = None
    if pool:
        # score: prefer x around -0.7..-1.0 and y around 0.8..1.6
        def score(t):
            wf, med, x, y, c, r = t
            return -((x + 0.9) ** 2) - 0.3 * ((y - 1.2) ** 2) + 0.5 * wf

        pool2 = sorted(pool, key=score, reverse=True)
        best_spawn = pool2[0]
        print(
            "SPAWN_PARKING %.4f %.4f %.4f"
            % (best_spawn[2], best_spawn[3], -0.20)
        )  # yaw ~0 face +x into aisle/exit
    else:
        print("SPAWN_PARKING -1.1500 1.4500 -0.8500")

    # Also print school for reference
    print("SCHOOL_DEFAULT -0.7253 -0.0866 1.5708")

    vis = img.copy()
    for wf, med, x, y, c, r in chosen:
        cv2.circle(vis, (c, r), 8, (0, 255, 255), 1)
    if best_spawn:
        cv2.circle(vis, (best_spawn[4], best_spawn[5]), 14, (0, 255, 0), 3)
        cv2.putText(
            vis,
            "PARK",
            (best_spawn[4] + 10, best_spawn[5]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )
    sc, sr = w2p(-0.725, -0.087, w, h)
    cv2.circle(vis, (sc, sr), 12, (0, 0, 255), 2)
    cv2.putText(vis, "SCHOOL", (sc + 8, sr), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    out = "/home/ha054/ros2_ws_simple/captures/comb_bays.png"
    cv2.imwrite(out, vis)
    print("saved", out)


if __name__ == "__main__":
    main()
