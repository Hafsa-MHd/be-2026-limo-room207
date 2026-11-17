#!/usr/bin/env python3
"""Synthetic half-black / half-white frame: center must prefer black side."""

import sys

from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent / 'limo_simple'))

import cv2
import numpy as np
from limo_simple.road_vision import segment_road


def make_split(black_on_left=True):
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    # sky / wall
    img[:180, :] = (180, 180, 180)
    # ground split
    if black_on_left:
        img[180:, :320] = (25, 25, 25)  # black road
        img[180:, 320:] = (210, 210, 210)  # white parking
    else:
        img[180:, :320] = (210, 210, 210)
        img[180:, 320:] = (25, 25, 25)
    return img


def main():
    left = make_split(True)
    right = make_split(False)
    m1, s1, u1, _, b1 = segment_road(left, 0.35, 55, 5)
    m2, s2, u2, _, b2 = segment_road(right, 0.35, 55, 5)
    cx1 = s1[len(s1) // 5][1] if s1 else -1
    cx2 = s2[len(s2) // 5][1] if s2 else -1
    print(f"black_left: usable={u1} bias={b1:+.2f} cx={cx1}")
    print(f"black_right: usable={u2} bias={b2:+.2f} cx={cx2}")
    ok = True
    if not u1 or not u2:
        print("FAIL: expected usable masks")
        ok = False
    if b1 >= -0.1:
        print("FAIL: expected negative bias when black on left")
        ok = False
    if b2 <= 0.1:
        print("FAIL: expected positive bias when black on right")
        ok = False
    if cx1 > 300:
        print("FAIL: center should be on left half")
        ok = False
    if cx2 < 340:
        print("FAIL: center should be on right half")
        ok = False
    # Save debug panels
    out = "/home/ha054/ros2_ws_simple/captures"
    import os

    os.makedirs(out, exist_ok=True)
    cv2.imwrite(f"{out}/split_black_left.png", left)
    cv2.imwrite(f"{out}/split_mask_left.png", m1)
    cv2.imwrite(f"{out}/split_black_right.png", right)
    cv2.imwrite(f"{out}/split_mask_right.png", m2)
    if ok:
        print("RESULT OK: prefers black side")
    else:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
