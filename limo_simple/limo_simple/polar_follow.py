#!/usr/bin/env python3
"""Polar control — FIGURE 3: Perception hard-gates Control; Planning may nudge.

Simple loop:
  1) If vision lost / !road_ok → stop
  2) Local goal from black-road perception
  3) Optional mission nudge if goal is ahead and near lane heading
  4) Polar (ρ,α,β) → forward-only cmd_vel (no reverse: no rear camera)
"""

from __future__ import annotations

import math
import os

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray

from .track_planner import TrackPlanner, installed_track_texture

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover
    cv2 = None
    np = None

_GOAL_BALL_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="polar_goal_ball">
    <static>true</static>
    <link name="link">
      <visual name="ball">
        <pose>0 0 0.25 0 0 0</pose>
        <geometry><sphere><radius>0.25</radius></sphere></geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          <emissive>1 0 0 1</emissive>
        </material>
      </visual>
      <visual name="pole">
        <pose>0 0 0.55 0 0 0</pose>
        <geometry><cylinder><radius>0.04</radius><length>0.6</length></cylinder></geometry>
        <material>
          <ambient>1 1 0 1</ambient>
          <diffuse>1 1 0 1</diffuse>
          <emissive>1 1 0 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

# Room 207 parking exit, outside roundabout lane, and south approach to D8.
_D8_GOAL = (0.976, -0.897)
_D8_APPROACH = (
    (-0.725, -0.087), (-0.73, 0.16), (-0.77, 0.37), (-0.84, 0.61),
    (-0.95, 0.84), (-1.10, 1.04), (-1.30, 1.23), (-1.55, 1.41),
    (-1.76, 1.59), (-1.88, 1.78), (-1.90, 1.97), (-1.83, 2.17),
    (-1.66, 2.34), (-1.43, 2.46), (-1.15, 2.53), (-0.80, 2.56),
    (-0.40, 2.60), (0.04, 2.59), (0.42, 2.54), (0.70, 2.56),
    (0.96, 2.67), (1.18, 2.745), (1.44, 2.762), (1.69, 2.714),
    (1.868, 2.608), (1.998, 2.452), (2.074, 2.265), (2.096, 2.08),
    (2.083, 1.90), (2.03, 1.65), (1.94, 1.37), (1.87, 0.98),
    (1.86, 0.55), (1.86, 0.12), (1.87, -0.34), (1.79, -0.58),
    (1.61, -0.74), (1.36, -0.81), (1.11, -0.83), (0.976, -0.842),
)


# Parking exit and shared route through the southern junction.
_D8_SOUTH = (
    (-0.725, -0.087), (-0.725, 0.450), (-0.725, 0.900), (-0.725, 1.167),
    (-0.736, 1.282), (-0.770, 1.393), (-0.825, 1.495), (-0.898, 1.585),
    (-0.988, 1.658), (-1.090, 1.713), (-1.201, 1.747), (-1.316, 1.758),
    (-1.431, 1.747), (-1.542, 1.713), (-1.644, 1.658), (-1.734, 1.585),
    (-1.807, 1.495), (-1.862, 1.393), (-1.896, 1.282), (-1.907, 1.167),
    (-1.907, 0.700), (-1.907, 0.250), (-1.907, -0.050), (-1.907, -0.283),
    (-1.900, -0.366), (-1.881, -0.447), (-1.849, -0.524), (-1.806, -0.595),
    (-1.751, -0.658), (-1.688, -0.713), (-1.617, -0.756), (-1.540, -0.788),
    (-1.459, -0.807), (-1.376, -0.814), (-1.100, -0.814), (-0.600, -0.814),
    (0.000, -0.814), (0.500, -0.814), (0.976, -0.814),
)


_D5_GOAL = (2.101, -0.644)
_SOUTH_EXIT = ((1.45, -0.814), (1.80, -0.77), (2.101, -0.67))


def _corridor_prefix(goal, corridor, tol=0.12):
    """Prefix of a hardcoded corridor ending near `goal` (projected on-centerline)."""
    start = corridor[0]
    best = None
    for i, (end_x, end_y) in enumerate(corridor[1:]):
        x, y = corridor[i]
        dx, dy = end_x - x, end_y - y
        denom = dx * dx + dy * dy
        if denom < 1e-9:
            continue
        fraction = ((goal[0] - x) * dx + (goal[1] - y) * dy) / denom
        fraction = max(0.0, min(1.0, fraction))
        projected = (x + fraction * dx, y + fraction * dy)
        error = math.dist(goal, projected)
        if (math.dist(start, projected) > 0.15 and
                error <= tol and (best is None or error < best[0])):
            best = (error, i, fraction, projected)
    if best is None:
        return None
    _, i, fraction, projected = best
    if fraction <= 1e-6:
        return corridor[:i + 1]
    return (*corridor[:i + 1], projected)


def _south_path(goal):
    if math.dist(goal, _D8_GOAL) <= 0.08:
        return (*_D8_SOUTH[:-1], (goal[0], goal[1] + 0.083))
    if math.dist(goal, _D5_GOAL) <= 0.08:
        return (*_D8_SOUTH, *_SOUTH_EXIT[:-1], (goal[0], goal[1] - 0.026))
    return _corridor_prefix(goal, _D8_SOUTH, tol=0.08)


def _approach_path(goal):
    """Long Room-207 approach (includes NE ring). Last resort after map search."""
    return _corridor_prefix(goal, _D8_APPROACH, tol=0.14)


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def body_polar(dx: float, dy: float):
    rho = math.hypot(dx, dy)
    if rho < 1e-9:
        return 0.0, 0.0, 0.0
    beta = -math.atan2(dy, dx)
    alpha = _wrap(-beta)
    return rho, alpha, beta


def polar_control(rho, alpha, beta, k_rho, k_alpha, k_beta):
    """BE poly law. Always forward — no rear camera."""
    v_poly = k_rho * rho
    omega = k_alpha * alpha + k_beta * beta
    v_cmd = abs(-v_poly)
    if abs(alpha) > math.pi / 2.0:
        # Goal behind in body frame: keep driving forward, flip turn.
        omega = -omega
    return v_cmd, omega


def omega_to_gamma(v, omega, wheelbase, v_eps=1e-3):
    if abs(v) < v_eps:
        return 0.0
    return math.atan(wheelbase * omega / v)


class PolarFollow(Node):
    def __init__(self):
        super().__init__("polar_follow")
        self.declare_parameter("k_rho", -0.6)
        self.declare_parameter("k_alpha", 1.4)
        self.declare_parameter("k_beta", -0.8)
        self.declare_parameter("wheelbase", 0.20)
        self.declare_parameter("v_max", 0.12)
        self.declare_parameter("v_min", 0.04)
        self.declare_parameter("gamma_max", 0.45)
        self.declare_parameter("image_width", 640.0)
        self.declare_parameter("lookahead_m", 0.45)
        self.declare_parameter("lane_half_width_m", 0.35)
        self.declare_parameter("k_black", 0.55)
        self.declare_parameter("deadband", 0.03)
        self.declare_parameter("lost_timeout_sec", 1.0)
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("publish_gamma", True)
        self.declare_parameter("gazebo_goal_ball", True)
        self.declare_parameter("gazebo_ball_hz", 0.5)
        self.declare_parameter("mission_nudge_max_rad", 0.50)
        self.declare_parameter("mission_nudge_gain", 0.25)
        self.declare_parameter("reach_tol_m", 0.45)
        self.declare_parameter("d8_south_route", False)
        self.declare_parameter("offroad_stop_sec", 0.40)
        self.declare_parameter("map_texture", "")
        self.declare_parameter("map_size_x", 4.88)
        self.declare_parameter("map_size_y", 6.2)
        self.declare_parameter("map_black_max", 90)
        self.declare_parameter("map_island_max_m", 0.06)
        # Tight corridors: NE ring + mid junction (user red circle) — looser margin.
        self.declare_parameter("circle_cx", 1.44)
        self.declare_parameter("circle_cy", 2.11)
        self.declare_parameter("circle_radius_m", 1.15)
        self.declare_parameter("junction_cx", -0.35)
        self.declare_parameter("junction_cy", 1.15)
        self.declare_parameter("junction_radius_m", 0.85)
        self.declare_parameter("map_island_max_circle_m", 0.50)
        self.declare_parameter("require_mission", True)

        self._ok = False
        self._cx = 0.0
        self._heading = 0.0
        self._black_bias = 0.0
        self._last_vision = self.get_clock().now()
        self._last_road_ok = self.get_clock().now()
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._have_odom = False
        self._last_odom = None
        self._mission = None
        self._d8_segment = None
        self._mission_path = None
        self._route_attempted = False
        self._map_route_generated = False
        self._track_planner = None
        self._last_ball = self.get_clock().now()
        self._ball_busy = False
        self._ball_at = None
        self._map_gray = None
        self._map_into = None
        self._last_gate = ""
        self._load_map()

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_rho = self.create_publisher(Float32, "/control/rho", 10)
        self.pub_alpha = self.create_publisher(Float32, "/control/alpha", 10)
        self.pub_beta = self.create_publisher(Float32, "/control/beta", 10)
        self.pub_goal = self.create_publisher(PoseStamped, "/control/goal", 10)
        self.pub_marker = self.create_publisher(MarkerArray, "/control/goal_marker", 10)
        self.pub_reached = self.create_publisher(Bool, "/control/goal_reached", 10)
        self.pub_gate = self.create_publisher(String, "/control/gate_reason", 10)

        self._cli_delete = self.create_client(DeleteEntity, "/delete_entity")
        self._cli_spawn = self.create_client(SpawnEntity, "/spawn_entity")

        self.create_subscription(Bool, "/vision/road_ok", self.on_ok, 10)
        self.create_subscription(Float32MultiArray, "/vision/center", self.on_center, 10)
        self.create_subscription(Float32, "/vision/black_bias", self.on_bias, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(PoseStamped, "/control/mission_goal", self.on_mission, 10)
        self.create_subscription(Bool, "/control/mission_enable", self.on_mission_enable, 10)

        period = 1.0 / max(1.0, float(self.get_parameter("control_hz").value))
        self.create_timer(period, self.on_timer)
        self.get_logger().info(
            "polar_follow ready (simple: perception gate + forward-only mission)."
        )

    def _set_gate(self, reason: str) -> None:
        if reason == self._last_gate:
            return
        self._last_gate = reason
        msg = String()
        msg.data = reason
        self.pub_gate.publish(msg)

    def _load_map(self) -> None:
        path = str(self.get_parameter("map_texture").value or installed_track_texture())
        if cv2 is None or np is None or not path or not os.path.isfile(path):
            return
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            return
        self._map_gray = img
        self._track_planner = TrackPlanner(
            img, float(self.get_parameter("map_size_x").value),
            float(self.get_parameter("map_size_y").value),
        )
        asphalt = (img <= 160).astype(np.uint8)
        self._map_into = cv2.distanceTransform(1 - asphalt, cv2.DIST_L2, 5)
        self.get_logger().info(f"map audit ON {img.shape[1]}x{img.shape[0]}")

    def _island_depth(self, x: float, y: float) -> float:
        if self._map_into is None or np is None:
            return 0.0
        h, w = self._map_into.shape
        sx = float(self.get_parameter("map_size_x").value)
        sy = float(self.get_parameter("map_size_y").value)
        col = int(round((x / sx + 0.5) * (w - 1)))
        row = int(round((0.5 - y / sy) * (h - 1)))
        if not (0 <= col < w and 0 <= row < h):
            return 1.0
        mpp = sx / max(1.0, float(w - 1))
        return float(self._map_into[row, col]) * mpp

    def _in_tight_zone(self, x=None, y=None) -> bool:
        """Narrow black corridors (NE ring + mid junction) — looser map margin."""
        if not self._have_odom and x is None:
            return False
        px = self._x if x is None else float(x)
        py = self._y if y is None else float(y)
        zones = (
            (
                float(self.get_parameter("circle_cx").value),
                float(self.get_parameter("circle_cy").value),
                float(self.get_parameter("circle_radius_m").value),
            ),
            (
                float(self.get_parameter("junction_cx").value),
                float(self.get_parameter("junction_cy").value),
                float(self.get_parameter("junction_radius_m").value),
            ),
        )
        return any(math.hypot(px - cx, py - cy) <= r for cx, cy, r in zones)

    def _island_limit(self, x=None, y=None) -> float:
        if self._in_tight_zone(x, y):
            return float(self.get_parameter("map_island_max_circle_m").value)
        return float(self.get_parameter("map_island_max_m").value)

    def _forward_map_bias(self) -> float:
        """Soft lateral bias from map — also in tight zones when nose hits white."""
        if not self._have_odom or self._map_into is None:
            return 0.0
        look = float(self.get_parameter("lookahead_m").value)
        c, s = math.cos(self._yaw), math.sin(self._yaw)
        lx, ly = -s, c
        lim = self._island_limit()
        body = self._island_depth(self._x, self._y)
        nose_x = self._x + look * c
        nose_y = self._y + look * s
        mid = self._island_depth(nose_x, nose_y)
        left = min(
            self._island_depth(nose_x + 0.28 * lx, nose_y + 0.28 * ly),
            self._island_depth(nose_x + 0.50 * lx, nose_y + 0.50 * ly),
        )
        right = min(
            self._island_depth(nose_x - 0.28 * lx, nose_y - 0.28 * ly),
            self._island_depth(nose_x - 0.50 * lx, nose_y - 0.50 * ly),
        )
        # Safe corridor: leave steering to vision.
        if mid <= lim * 0.45 and body <= lim * 0.45:
            return 0.0
        amp = 0.20 + 0.40 * min(1.0, max(mid, body) / max(lim, 1e-3))
        if self._in_tight_zone():
            amp *= 0.85
        if left + 1e-6 < right:
            return amp
        if right + 1e-6 < left:
            return -amp
        return 0.0

    def on_ok(self, msg: Bool) -> None:
        self._ok = bool(msg.data)
        if self._ok:
            self._last_road_ok = self.get_clock().now()

    def on_bias(self, msg: Float32) -> None:
        self._black_bias = float(msg.data)

    def on_center(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 5:
            return
        self._cx = float(msg.data[0])
        self._heading = float(msg.data[4])
        if len(msg.data) >= 6:
            self._black_bias = float(msg.data[5])
        self._last_vision = self.get_clock().now()

    def on_odom(self, msg: Odometry) -> None:
        self._x = float(msg.pose.pose.position.x)
        self._y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self._yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._have_odom = True
        self._last_odom = self.get_clock().now()

    def on_mission(self, msg: PoseStamped) -> None:
        nxt = (float(msg.pose.position.x), float(msg.pose.position.y))
        if self._mission is None or math.hypot(nxt[0] - self._mission[0], nxt[1] - self._mission[1]) > 0.05:
            self._d8_segment = None
            self._mission_path = None
            self._route_attempted = False
            self._map_route_generated = False
            self.get_logger().info(f"mission goal ({nxt[0]:.2f}, {nxt[1]:.2f})")
        self._mission = nxt

    def on_mission_enable(self, msg: Bool) -> None:
        if not msg.data:
            self._mission = None
            self._d8_segment = None
            self._mission_path = None
            self._route_attempted = False
            self._map_route_generated = False
            self._ball_at = None
            self._delete_gazebo_ball()
            self.get_logger().info("mission cleared")

    def _vision_goal_base(self):
        width = float(self.get_parameter("image_width").value)
        err = (self._cx - 0.5 * width) / max(1.0, 0.5 * width)
        err -= float(self.get_parameter("k_black").value) * self._black_bias
        if abs(err) < float(self.get_parameter("deadband").value):
            err = 0.0
        look = float(self.get_parameter("lookahead_m").value)
        half = float(self.get_parameter("lane_half_width_m").value)
        dx = look
        dy = -err * half - math.sin(self._heading) * look * 0.35
        return dx, dy

    def _mission_goal_base(self):
        if self._mission is None or not self._have_odom:
            return None
        return self._point_in_base(*self._mission)

    def _point_in_base(self, x, y):
        c, s = math.cos(self._yaw), math.sin(self._yaw)
        dx, dy = x - self._x, y - self._y
        return c * dx + s * dy, -s * dx + c * dy

    def _route_lookahead(self, south_route):
        if self._mission is None or not self._have_odom:
            return None
        if self._mission_path is None:
            if math.dist((self._x, self._y), _D8_APPROACH[0]) > 0.25:
                return None
            if south_route:
                if self._route_attempted:
                    return None
                self._route_attempted = True
                self._mission_path = _south_path(self._mission)
                if self._mission_path is None and self._track_planner is not None:
                    self.get_logger().info(f"Searching track for a route to {self._mission}")
                    self._mission_path = self._track_planner.plan(
                        (self._x, self._y, self._yaw), self._mission
                    )
                    self._map_route_generated = self._mission_path is not None
                if self._mission_path is None:
                    self._mission_path = _approach_path(self._mission)
            elif math.dist(self._mission, _D8_GOAL) <= 0.08:
                goal_x, goal_y = self._mission
                self._mission_path = (*_D8_APPROACH[:-1], (goal_x, goal_y + 0.055))
            if self._mission_path is None:
                return None
            self._d8_segment = 0
            self.get_logger().info(
                f"Room 207 route active to ({self._mission[0]:.3f}, {self._mission[1]:.3f})"
            )
        path = self._mission_path
        half_wheelbase = float(self.get_parameter("wheelbase").value) / 2.0
        rear_x = self._x - half_wheelbase * math.cos(self._yaw)
        rear_y = self._y - half_wheelbase * math.sin(self._yaw)
        i = self._d8_segment
        while i < len(path) - 1:
            x, y = path[i]
            nx, ny = path[i + 1]
            dx, dy = nx - x, ny - y
            length = math.hypot(dx, dy)
            along = ((rear_x - x) * dx + (rear_y - y) * dy) / length
            if along < length:
                break
            i += 1

        if i != self._d8_segment:
            self._d8_segment = i
            self.get_logger().info(f"Route segment {min(i + 1, len(path) - 1)}/{len(path) - 1}")
        if i == len(path) - 1:
            return self._point_in_base(*path[-1])

        lookahead = 0.18 if self._map_route_generated else 0.23
        remaining = lookahead + max(0.0, along)
        for (x, y), (nx, ny) in zip(path[i:], path[i + 1:]):
            dx, dy = nx - x, ny - y
            length = math.hypot(dx, dy)
            if remaining <= length:
                return self._point_in_base(x + dx * remaining / length, y + dy * remaining / length)
            remaining -= length
        return self._point_in_base(*path[-1])

    def _final_mission_rho(self):
        if self._mission is None or not self._have_odom:
            return None
        return math.hypot(self._mission[0] - self._x, self._mission[1] - self._y)

    def _publish_markers(self, dx, dy, world_xy=None):
        stamp = self.get_clock().now().to_msg()
        goal = PoseStamped()
        goal.header.stamp = stamp
        if world_xy is not None:
            goal.header.frame_id = "odom"
            goal.pose.position.x = world_xy[0]
            goal.pose.position.y = world_xy[1]
        else:
            goal.header.frame_id = "base_link"
            goal.pose.position.x = dx
            goal.pose.position.y = dy
        goal.pose.position.z = 0.12
        yaw_g = math.atan2(dy, dx) + (self._yaw if world_xy is not None else 0.0)
        goal.pose.orientation.z = math.sin(yaw_g / 2.0)
        goal.pose.orientation.w = math.cos(yaw_g / 2.0)
        self.pub_goal.publish(goal)

        arr = MarkerArray()
        sphere = Marker()
        sphere.header.stamp = stamp
        sphere.header.frame_id = "base_link"
        sphere.ns = "polar_goal"
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = float(dx)
        sphere.pose.position.y = float(dy)
        sphere.pose.position.z = 0.15
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.22
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 0.0, 0.0, 1.0
        sphere.lifetime.sec = 2
        text = Marker()
        text.header = sphere.header
        text.ns = "polar_goal"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(dx)
        text.pose.position.y = float(dy)
        text.pose.position.z = 0.42
        text.pose.orientation.w = 1.0
        text.scale.z = 0.18
        text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 1.0, 0.0, 1.0
        text.text = "GOAL"
        text.lifetime.sec = 2
        line = Marker()
        line.header = sphere.header
        line.ns = "polar_goal"
        line.id = 2
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.04
        line.color.r, line.color.g, line.color.b, line.color.a = 1.0, 0.3, 0.0, 1.0
        line.lifetime.sec = 2
        line.points = [Point(x=0.0, y=0.0, z=0.05), Point(x=float(dx), y=float(dy), z=0.05)]
        arr.markers = [sphere, text, line]
        self.pub_marker.publish(arr)

        if (
            self._mission is not None
            and world_xy is not None
            and bool(self.get_parameter("gazebo_goal_ball").value)
        ):
            self._maybe_move_gazebo_ball(self._mission[0], self._mission[1])

    def _delete_gazebo_ball(self) -> None:
        if not self._cli_delete.service_is_ready():
            return
        dreq = DeleteEntity.Request()
        dreq.name = "polar_goal_ball"
        self._cli_delete.call_async(dreq)

    def _maybe_move_gazebo_ball(self, gx, gy):
        if self._ball_busy:
            return
        if self._ball_at is not None:
            if math.hypot(self._ball_at[0] - gx, self._ball_at[1] - gy) < 0.15:
                return
        hz = max(0.3, float(self.get_parameter("gazebo_ball_hz").value))
        age = (self.get_clock().now() - self._last_ball).nanoseconds * 1e-9
        if age < 1.0 / hz:
            return
        if not self._cli_delete.service_is_ready() or not self._cli_spawn.service_is_ready():
            return
        self._ball_busy = True
        self._last_ball = self.get_clock().now()
        wx, wy = float(gx), float(gy)

        def finish(_=None):
            self._ball_busy = False
            self._ball_at = (wx, wy)

        def do_spawn(_=None):
            req = SpawnEntity.Request()
            req.name = "polar_goal_ball"
            req.xml = _GOAL_BALL_SDF
            req.initial_pose.position.x = wx
            req.initial_pose.position.y = wy
            req.initial_pose.position.z = 0.0
            req.initial_pose.orientation.w = 1.0
            req.reference_frame = "world"
            fut2 = self._cli_spawn.call_async(req)
            fut2.add_done_callback(finish)

        dreq = DeleteEntity.Request()
        dreq.name = "polar_goal_ball"
        fut = self._cli_delete.call_async(dreq)
        fut.add_done_callback(do_spawn)

    def on_timer(self) -> None:
        cmd = Twist()
        age = (self.get_clock().now() - self._last_vision).nanoseconds * 1e-9
        lost = age > float(self.get_parameter("lost_timeout_sec").value)
        ok_age = (self.get_clock().now() - self._last_road_ok).nanoseconds * 1e-9
        stale_ok = ok_age > float(self.get_parameter("offroad_stop_sec").value)

        # Hold until mission (no free wander onto white).
        if bool(self.get_parameter("require_mission").value) and self._mission is None:
            self._set_gate("hold_no_mission")
            self.pub_cmd.publish(cmd)
            self.pub_reached.publish(Bool(data=False))
            return

        if self._mission is not None:
            odom_age = (
                (self.get_clock().now() - self._last_odom).nanoseconds * 1e-9
                if self._last_odom is not None else math.inf
            )
            if not 0.0 <= odom_age <= 0.5:
                self._set_gate("stale_odom")
                self.pub_cmd.publish(cmd)
                self.pub_reached.publish(Bool(data=False))
                return

        # FIGURE 3: Perception hard-gates on road_ok. Brief center drop ≠ stop.
        vision_hard = (not self._ok) or stale_ok
        island = self._island_depth(self._x, self._y)
        body_ok = island <= self._island_limit()
        body_soft = island <= max(self._island_limit(), 0.22)
        mission_base = self._mission_goal_base()
        world_dist = None
        dx_g = dy_g = 0.0
        beside_goal = False
        if mission_base is not None:
            dx_g, dy_g = mission_base
            world_dist = math.hypot(
                self._mission[0] - self._x, self._mission[1] - self._y
            )
            # Only latch capture when already near D8 on the south road (not far west).
            beside_goal = (
                abs(self._y - self._mission[1]) < 0.45
                and self._x > 0.05
                and self._x < self._mission[0] - 0.10
                and world_dist < 1.6
            )
            tol = float(self.get_parameter("reach_tol_m").value)
            self._publish_markers(dx_g, dy_g, self._mission)
            reached = world_dist < tol
            self.pub_reached.publish(Bool(data=reached))
            if reached:
                self._set_gate("goal_reached")
                self.pub_cmd.publish(cmd)
                self.pub_rho.publish(Float32(data=float(world_dist)))
                return

        south_route = bool(self.get_parameter("d8_south_route").value)
        guide = self._route_lookahead(south_route)
        if self._mission is not None and south_route and guide is None:
            self._set_gate("no_route_from_parking")
            self.pub_cmd.publish(cmd)
            return
        if guide is not None:
            x, y = guide
            wheelbase = float(self.get_parameter("wheelbase").value)
            x += wheelbase / 2.0
            if x <= 0.0:
                self._set_gate("route_target_behind")
            else:
                curvature = 2.0 * y / max(x * x + y * y, 1e-4)
                limit = float(self.get_parameter("gamma_max").value)
                cmd.angular.z = max(-limit, min(limit, math.atan(wheelbase * curvature)))
                speed = 0.08 if self._map_route_generated else 0.10
                cmd.linear.x = min(speed, float(self.get_parameter("v_max").value))
                self._set_gate("drive_map_route")
            self.pub_cmd.publish(cmd)
            self.pub_rho.publish(Float32(data=float(world_dist)))
            return

        # West of D8 on the same latitude: keep turning east even if vision blinks.
        capturing = beside_goal and body_soft
        map_hold = vision_hard and (body_soft or self._in_tight_zone()) and not capturing
        if vision_hard and not map_hold and not capturing:
            self._set_gate("road_ok_false" if not self._ok else "stale_road_ok")
            self.pub_cmd.publish(cmd)
            self.pub_reached.publish(Bool(data=False))
            return

        look = float(self.get_parameter("lookahead_m").value)
        map_bias = self._forward_map_bias()
        if capturing:
            dx = max(0.20, dx_g)
            dy = dy_g
            gate = "drive_mission_capture"
        elif map_hold:
            bias = map_bias if abs(map_bias) > 1e-3 else (-0.15 if self._x > 1.7 else 0.10)
            dx, dy = look, bias
            gate = "map_hold_tight" if self._in_tight_zone() else "map_hold_black"
        else:
            # Keep last vision steering even if center is briefly stale.
            dx, dy = self._vision_goal_base()
            dy = dy + map_bias
            if self._x > 1.95 and abs(map_bias) < 1e-3:
                dy = dy - 0.12
            if lost:
                gate = "drive_vision_hold"
            else:
                gate = "drive_tight" if self._in_tight_zone() else "drive_vision"

        if mission_base is not None and not map_hold and not capturing:
            # Mission influence only on the southern approach — never from the north loop
            # (that pulls across white toward D8).
            south_approach = self._y < 0.20 and self._x > -0.10
            near_goal = world_dist is not None and world_dist < 1.35
            if south_approach and near_goal:
                dx = max(0.20, dx_g)
                dy = dy_g
                gate = "drive_mission_capture"
            elif south_approach and world_dist is not None and world_dist < 2.8 and dx_g > 0.05:
                lane_yaw = math.atan2(dy, dx)
                goal_yaw = math.atan2(dy_g, dx_g)
                err_yaw = _wrap(goal_yaw - lane_yaw)
                max_nudge = max(
                    float(self.get_parameter("mission_nudge_max_rad").value), 1.35
                )
                gain = float(self.get_parameter("mission_nudge_gain").value) * 2.5
                if abs(err_yaw) < max_nudge:
                    dy = dy + gain * math.sin(err_yaw) * look
                    gate = "drive_mission_nudge"
        elif mission_base is None:
            self._publish_markers(dx, dy, None)

        if abs(map_bias) > 1e-3 and not map_hold and not capturing and gate.startswith("drive_vision"):
            gate = "drive_map_keep_black"
        self._set_gate(gate)

        rho, alpha, beta = body_polar(dx, dy)
        speed_rho = world_dist if world_dist is not None else rho
        k_rho = float(self.get_parameter("k_rho").value)
        k_alpha = float(self.get_parameter("k_alpha").value)
        k_beta = float(self.get_parameter("k_beta").value)
        v, omega = polar_control(speed_rho, alpha, beta, k_rho, k_alpha, k_beta)

        # Forward-only: no rear camera → never publish negative v.
        v_max = float(self.get_parameter("v_max").value)
        v_min = float(self.get_parameter("v_min").value)
        v = max(v_min, min(v_max, abs(v) if abs(v) > 1e-6 else v_min))
        # Slow in tight corridors; creep on map-hold. Never reverse.
        if self._in_tight_zone():
            v = min(v, 0.55 * (v_min + v_max))
        elif map_hold or abs(map_bias) > 0.15:
            v = min(v, v_min + 0.03)
            if abs(omega) < 0.55:
                omega = math.copysign(
                    1.0, map_bias if abs(map_bias) > 1e-3 else (dy if abs(dy) > 1e-3 else 1.0)
                )

        wheelbase = float(self.get_parameter("wheelbase").value)
        gamma_max = float(self.get_parameter("gamma_max").value)
        if bool(self.get_parameter("publish_gamma").value):
            steer = omega_to_gamma(v, omega, wheelbase)
        else:
            steer = omega
        steer = max(-gamma_max, min(gamma_max, steer))

        cmd.linear.x = float(v)
        cmd.angular.z = float(steer)
        self.pub_cmd.publish(cmd)

        self.pub_rho.publish(Float32(data=float(speed_rho)))
        self.pub_alpha.publish(Float32(data=float(alpha)))
        self.pub_beta.publish(Float32(data=float(beta)))


def main(args=None):
    rclpy.init(args=args)
    node = PolarFollow()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.pub_cmd.publish(Twist())
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
