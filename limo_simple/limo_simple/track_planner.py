"""Forward-only Ackermann route search on the Room 207 track texture."""

import heapq
import itertools
import math
from pathlib import Path

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory

_STEP = 0.14
_POSITION_BIN = 0.055
_HEADING_BINS = 64
_WHEELBASE = 0.24
_TRACK = 0.168
_TIRE_WIDTH = 0.045
_CURVATURES = (0.0, -0.8, 0.8, -1.5, 1.5, -1.85, 1.85)


def installed_track_texture():
    package = Path(get_package_share_directory("qcar_large_track"))
    return package / "models" / "qcar_large_track" / "materials" / "textures" / "SDCS_MapLayout.png"


class TrackPlanner:
    def __init__(self, texture, size_x, size_y):
        asphalt = (texture <= 160).astype(np.uint8)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        asphalt = cv2.morphologyEx(asphalt, cv2.MORPH_CLOSE, kernel)
        self._distance = cv2.distanceTransform(1 - asphalt, cv2.DIST_L2, 5)
        self._rows, self._cols = asphalt.shape
        self._pixels_x = (self._cols - 1) / size_x
        self._pixels_y = (self._rows - 1) / size_y

    def _pixel(self, x, y):
        return (round((self._rows - 1) / 2 - y * self._pixels_y),
                round((self._cols - 1) / 2 + x * self._pixels_x))

    def _road(self, x, y, tolerance=0.0):
        row, col = self._pixel(x, y)
        return (
            0 <= row < self._rows and 0 <= col < self._cols
            and self._distance[row, col] <= tolerance * self._pixels_x
        )

    @staticmethod
    def _key(x, y, yaw):
        return (round(x / _POSITION_BIN), round(y / _POSITION_BIN),
                round(yaw * _HEADING_BINS / math.tau) % _HEADING_BINS)

    @staticmethod
    def _advance(x, y, yaw, curvature, length):
        heading = yaw + curvature * length
        if curvature == 0:
            return x + length * math.cos(yaw), y + length * math.sin(yaw), yaw
        return (x + (math.sin(heading) - math.sin(yaw)) / curvature,
                y + (math.cos(yaw) - math.cos(heading)) / curvature, heading)

    def _clear(self, x, y, yaw, curvature):
        c, s = math.cos(yaw), math.sin(yaw)
        if not self._road(x + c * _WHEELBASE / 2, y + s * _WHEELBASE / 2):
            return False
        for side in (-1, 1):
            steering = math.atan(_WHEELBASE * curvature / (1 - side * _TRACK * curvature / 2))
            lateral = side * _TRACK / 2
            for axle, angle in ((0.0, 0.0), (_WHEELBASE, steering)):
                for width in (0.0, _TIRE_WIDTH):
                    dx = axle - side * width * math.sin(angle)
                    dy = lateral + side * width * math.cos(angle)
                    if not self._road(x + c * dx - s * dy, y + s * dx + c * dy, 0.005):
                        return False
        return True

    def plan(self, start, goal):
        """Return rear-axle waypoints, or None if the goal has no feasible route."""
        if not all(map(math.isfinite, (*start, *goal))) or not self._road(*goal):
            return None
        sx, sy, yaw = start
        initial = (sx - _WHEELBASE / 2 * math.cos(yaw),
                   sy - _WHEELBASE / 2 * math.sin(yaw), yaw)
        serial = itertools.count()
        nodes = [(initial, None, 0.0, 0.0)]
        queue = [(math.dist((sx, sy), goal), next(serial), 0)]
        best = {self._key(*initial): 0.0}
        while queue and len(nodes) < 115_000:
            _, _, index = heapq.heappop(queue)
            pose, parent, cost, previous = nodes[index]
            if cost > best[self._key(*pose)]:
                continue
            x, y, yaw = pose
            center = (x + _WHEELBASE / 2 * math.cos(yaw), y + _WHEELBASE / 2 * math.sin(yaw))
            distance = math.dist(center, goal)
            if distance <= 0.085 and parent is not None:
                points = []
                while index is not None:
                    state, index, _, _ = nodes[index]
                    points.append(state[:2])
                return tuple(reversed(points))

            lengths = (_STEP / 2, _STEP) if distance < 0.32 else (_STEP,)
            for length in lengths:
                for curvature in _CURVATURES:
                    end = self._advance(x, y, yaw, curvature, length)
                    key = self._key(*end)
                    new_cost = cost + length * (1.0 + 0.12 * abs(curvature)) + 0.035 * abs(curvature - previous)
                    if new_cost >= best.get(key, math.inf):
                        continue
                    mid = self._advance(x, y, yaw, curvature, length / 2)
                    if not self._clear(*end, curvature) or not self._clear(*mid, curvature):
                        continue
                    best[key] = new_cost
                    child = len(nodes)
                    nodes.append((end, index, new_cost, curvature))
                    nx, ny, heading = end
                    hx = goal[0] - nx - _WHEELBASE / 2 * math.cos(heading)
                    hy = goal[1] - ny - _WHEELBASE / 2 * math.sin(heading)
                    heuristic = math.hypot(hx, hy)
                    heapq.heappush(queue, (new_cost + 1.7 * heuristic, next(serial), child))
        return None

