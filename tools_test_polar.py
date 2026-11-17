#!/usr/bin/env python3
"""Unit checks for polar law (no ROS)."""

import math
import sys

from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent / 'limo_simple'))

from limo_simple.polar_follow import body_polar, omega_to_gamma, polar_control


def almost(a, b, eps=1e-6):
    return abs(a - b) < eps


def main():
    rho, alpha, beta = body_polar(0.45, 0.0)
    assert almost(rho, 0.45) and almost(alpha, 0.0) and almost(beta, 0.0)
    v, w = polar_control(rho, alpha, beta, -0.6, 1.4, -0.8)
    assert v > 0 and almost(w, 0.0), (v, w)

    rho, alpha, beta = body_polar(0.45, 0.2)  # goal left
    v, w = polar_control(rho, alpha, beta, -0.6, 1.4, -0.8)
    assert alpha > 0 and beta < 0 and w > 0, (alpha, beta, w)  # turn left

    rho, alpha, beta = body_polar(0.45, -0.2)  # goal right
    v, w = polar_control(rho, alpha, beta, -0.6, 1.4, -0.8)
    assert alpha < 0 and w < 0, (alpha, w)  # turn right

    g = omega_to_gamma(0.1, 0.5, 0.2)
    assert almost(g, math.atan(0.2 * 0.5 / 0.1))
    print(
        f"ahead v={polar_control(0.45,0,0,-0.6,1.4,-0.8)[0]:+.3f} "
        f"left w={w if False else polar_control(*body_polar(0.45,0.2),-0.6,1.4,-0.8)[1]:+.3f}"
    )
    print("RESULT OK polar unit tests")


if __name__ == "__main__":
    main()
