#!/usr/bin/env python3
"""Validate a forward-only Room 207 mission from the parking stall."""

from __future__ import annotations

import argparse
import math
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Bool, Float32, String

from limo_simple.track_planner import installed_track_texture

PARK = (-0.725347, -0.086613)
GOAL = (0.976, -0.897)
TEX = str(installed_track_texture())
SX, SY = 4.88, 6.2
TIMEOUT = 240.0
GOAL_TOLERANCE = 0.12
PARK_EXIT_DISTANCE = 0.40
# The map resolves about 4.8 mm per pixel. Allow two pixels of edge uncertainty.
MAP_EDGE_TOLERANCE = 0.01
WHEEL_FRAMES = {
    "front_left_wheel_link", "front_right_wheel_link",
    "rear_left_wheel_link", "rear_right_wheel_link",
}


class Once(Node):
    def __init__(self):
        super().__init__("test_control_once")
        self.odom = None
        self.road_ok = False
        self.road_frames = 0
        self.valid_road_frames = 0
        self.rho = None
        self.reached = False
        self.gate = ""
        self.cmd_v = 0.0
        self.wheels = {}
        self.max_wheel_intrusion = 0.0
        self.wheel_violation_pose = None
        self.create_subscription(Odometry, "/odom", self._od, 10)
        self.create_subscription(TFMessage, "/tf", self._wheel_transforms, 30)
        self.create_subscription(Bool, "/vision/road_ok", self._ok, 10)
        self.create_subscription(Float32, "/control/rho", self._rho, 10)
        self.create_subscription(Bool, "/control/goal_reached", self._reach, 10)
        self.create_subscription(String, "/control/gate_reason", self._gate, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd, 10)
        self.pub_goal = self.create_publisher(PoseStamped, "/control/mission_goal", 10)
        self.pub_en = self.create_publisher(Bool, "/control/mission_enable", 10)

        img = cv2.imread(TEX, 0)
        if img is None:
            raise RuntimeError(f"Missing track texture: {TEX}")
        asphalt = (img <= 160).astype(np.uint8)
        # Close thin painted line gaps without filling curbs or islands.
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        asphalt = cv2.morphologyEx(asphalt, cv2.MORPH_CLOSE, kernel)
        self.into = cv2.distanceTransform(1 - asphalt, cv2.DIST_L2, 5)
        self.h, self.w = img.shape

    def _od(self, m):
        self.odom = m
        if not WHEEL_FRAMES.issubset(self.wheels):
            return
        pose = m.pose.pose
        q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        for wheel in self.wheels.values():
            q = wheel.rotation
            # Transform the cylinder's outward local +Y axis into base_link.
            axis_x = 2 * (q.x * q.y - q.w * q.z)
            axis_y = 1 - 2 * (q.x * q.x + q.z * q.z)
            for width in (0.0, 0.0225, 0.045):
                x = wheel.translation.x + width * axis_x
                y = wheel.translation.y + width * axis_y
                intrusion = self.island(
                    pose.position.x + c * x - s * y,
                    pose.position.y + s * x + c * y,
                )
                if intrusion > self.max_wheel_intrusion:
                    self.max_wheel_intrusion = intrusion
                    self.wheel_violation_pose = (pose.position.x, pose.position.y)

    def _wheel_transforms(self, msg):
        for t in msg.transforms:
            # Gazebo publishes direct base-to-wheel transforms. The URDF
            # publisher routes the front wheels through the steering links.
            if t.header.frame_id == "base_link" and t.child_frame_id in WHEEL_FRAMES:
                self.wheels[t.child_frame_id] = t.transform

    def _ok(self, m):
        self.road_ok = bool(m.data)
        self.road_frames += 1
        self.valid_road_frames += self.road_ok

    def _rho(self, m):
        self.rho = float(m.data)

    def _reach(self, m):
        self.reached = bool(m.data)

    def _gate(self, m):
        self.gate = str(m.data)

    def _cmd(self, m):
        self.cmd_v = float(m.linear.x)

    def island(self, x, y):
        c = int(round((x / SX + 0.5) * (self.w - 1)))
        r = int(round((0.5 - y / SY) * (self.h - 1)))
        if not (0 <= c < self.w and 0 <= r < self.h):
            return 1.0
        return float(self.into[r, c]) * (SX / (self.w - 1))

    def spin(self, dt=0.05):
        rclpy.spin_once(self, timeout_sec=dt)

    def set_goal(self, x, y):
        g = PoseStamped()
        g.header.frame_id = "odom"
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = x
        g.pose.position.y = y
        g.pose.orientation.w = 1.0
        self.pub_goal.publish(g)
        self.pub_en.publish(Bool(data=True))

    def clear(self):
        self.pub_en.publish(Bool(data=False))


def main():
    rclpy.init()
    n = Once()
    t0 = time.time()
    while (n.odom is None or not WHEEL_FRAMES.issubset(n.wheels)) and time.time() - t0 < 10:
        n.spin()
    if n.odom is None:
        raise SystemExit("no odom")
    if not WHEEL_FRAMES.issubset(n.wheels):
        raise SystemExit("missing Gazebo wheel transforms")

    p = n.odom.pose.pose.position
    print("start odom=(%.3f,%.3f) road_ok=%s island=%.3f" % (p.x, p.y, n.road_ok, n.island(p.x, p.y)))
    if math.hypot(p.x - PARK[0], p.y - PARK[1]) > 0.12:
        raise SystemExit("test did not start in the parking stall")
    discovery_deadline = time.monotonic() + 5.0
    while True:
        publishers = n.get_publishers_info_by_topic("/cmd_vel")
        if len(publishers) == 1 and publishers[0].node_name == "polar_follow":
            break
        if time.monotonic() >= discovery_deadline:
            names = ", ".join(p.node_name for p in publishers) or "none"
            raise SystemExit(f"expected polar_follow as the only /cmd_vel publisher; found {names}")
        n.spin(0.05)
    print("mission GOAL=(%.3f,%.3f)" % GOAL)

    viol = 0
    samples = 0
    max_island = 0.0
    reverse_n = 0
    exited_parking = False
    stuck_t = None
    last = (p.x, p.y)
    t0 = time.time()
    last_log = 0.0
    last_goal = 0.0
    while time.time() - t0 < TIMEOUT:
        if time.time() - last_goal > 0.5:
            n.set_goal(*GOAL)
            last_goal = time.time()
        n.spin(0.05)
        if n.odom is None:
            continue
        p = n.odom.pose.pose.position
        isl = n.island(p.x, p.y)
        samples += 1
        max_island = max(max_island, isl)
        exited_parking |= math.dist((p.x, p.y), PARK) >= PARK_EXIT_DISTANCE
        if isl > MAP_EDGE_TOLERANCE:
            viol += 1
        if n.cmd_v < -1e-3:
            reverse_n += 1
        if math.hypot(p.x - last[0], p.y - last[1]) < 0.025:
            if stuck_t is None:
                stuck_t = time.time()
        else:
            stuck_t = None
            last = (p.x, p.y)
        goal_error = math.dist((p.x, p.y), GOAL)
        if n.reached and goal_error <= GOAL_TOLERANCE:
            print(f"REACHED goal_error={goal_error:.3f} m")
            break
        if stuck_t and time.time() - stuck_t > 20:
            print("STUCK at (%.3f,%.3f) gate=%s" % (p.x, p.y, n.gate))
            break
        if time.time() - last_log > 2.0:
            last_log = time.time()
            print(
                "t=%.0f odom=(%.2f,%.2f) v=%.3f road_ok=%s rho=%s gate=%s isl=%.3f"
                % (
                    time.time() - t0,
                    p.x,
                    p.y,
                    n.cmd_v,
                    n.road_ok,
                    None if n.rho is None else round(n.rho, 2),
                    n.gate,
                    isl,
                )
            )

    n.clear()
    if n.road_frames:
        valid_pct = 100 * n.valid_road_frames / n.road_frames
        print(f"road_ok_frames={n.valid_road_frames}/{n.road_frames} ({valid_pct:.1f}%)")
    p = n.odom.pose.pose.position if n.odom else None
    viol_frac = viol / max(1, samples)
    print("---")
    print(
        "end=%s reached=%s max_island=%.3f viol_frac=%.3f reverse=%d/%d"
        % (
            None if p is None else (round(p.x, 3), round(p.y, 3)),
            n.reached,
            max_island,
            viol_frac,
            reverse_n,
            samples,
        )
    )
    goal_error = math.dist((p.x, p.y), GOAL)
    print(f"parking_exit={exited_parking} goal_error={goal_error:.3f} m")
    print(f"max_wheel_intrusion={n.max_wheel_intrusion:.3f} m pose={n.wheel_violation_pose}")
    clear = viol == 0 and n.max_wheel_intrusion <= MAP_EDGE_TOLERANCE
    ok = exited_parking and n.reached and goal_error <= GOAL_TOLERANCE and clear and reverse_n == 0
    print("RESULT", "PASS" if ok else ("FAIL_WHITE" if not clear else "FAIL_MISSION"))
    n.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--goal", nargs=2, type=float, metavar=("X", "Y"), default=GOAL)
    parser.add_argument("--timeout", type=float, default=TIMEOUT)
    args = parser.parse_args()
    if not all(map(math.isfinite, args.goal)) or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("the goal must be finite and the timeout must be positive and finite")
    GOAL = tuple(args.goal)
    TIMEOUT = args.timeout
    main()
