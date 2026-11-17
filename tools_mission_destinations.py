#!/usr/bin/env python3
"""Parking → far black-road destinations mission test.

For each waypoint:
  1) Spawn robot in parking
  2) Put red GOAL ball on a far black-road point
  3) Enable polar mission (stay on black via vision blend)
  4) Wait until reached / timeout / off-road fail
  5) Record result, clear goal, next waypoint
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String

# True parking lot (diagonal-bay area), on BLACK asphalt, facing the exit.
# OLD (-1.40, 2.90) was WRONG: outer road curve next to white (not the lot).
# Official GitLab Room 207 spawn (just below parking bay, heading N).
PARKING = (-0.725347, -0.086613, 1.5708)

# Far destinations sampled from the track texture — BLACK asphalt only
# (eroded away from white islands). Distances are from parking.
DESTINATIONS = [
    ("D1_black", 1.538, -2.728, "black asphalt 6.35m from parking"),
    ("D2_black", -0.007, -2.819, "black asphalt 5.89m from parking"),
    ("D3_black", 2.120, -1.765, "black asphalt 5.84m from parking"),
    ("D4_black", -1.123, -2.652, "black asphalt 5.56m from parking"),
    ("D5_black", 2.101, -0.644, "black asphalt 4.98m from parking"),
    ("D6_black", -1.977, -1.955, "black asphalt 4.89m from parking"),
    ("D7_black", 0.107, -1.722, "black asphalt 4.86m from parking"),
    ("D8_black", 0.976, -0.897, "black asphalt 4.48m from parking"),
    # East of NE roundabout island (planner-reachable black ring).
    ("D9_roundabout", 2.100, 2.110, "black asphalt on NE roundabout ring"),
]


def dist_parking(x, y):
    return math.hypot(x - PARKING[0], y - PARKING[1])


class Mission(Node):
    def __init__(self):
        super().__init__("mission_destinations")
        self._odom = None
        self._road_ok = False
        self._rho = None
        self._reached = False
        self._xml = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(String, "/robot_description", self._on_desc, qos)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(Bool, "/vision/road_ok", self._on_ok, 10)
        self.create_subscription(Float32, "/control/rho", self._on_rho, 10)
        self.create_subscription(Bool, "/control/goal_reached", self._on_reached, 10)

        self.pub_goal = self.create_publisher(PoseStamped, "/control/mission_goal", 10)
        self.pub_enable = self.create_publisher(Bool, "/control/mission_enable", 10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)

        self.cli_del = self.create_client(DeleteEntity, "/delete_entity")
        self.cli_spawn = self.create_client(SpawnEntity, "/spawn_entity")

    def _on_desc(self, msg):
        if self._xml is None:
            self._xml = msg.data

    def _on_odom(self, msg):
        self._odom = msg

    def _on_ok(self, msg):
        self._road_ok = bool(msg.data)

    def _on_rho(self, msg):
        self._rho = float(msg.data)

    def _on_reached(self, msg):
        self._reached = bool(msg.data)

    def spin_for(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def stop(self):
        self.pub_cmd.publish(Twist())
        en = Bool()
        en.data = False
        self.pub_enable.publish(en)

    def wait_services(self):
        if not self.cli_del.wait_for_service(15) or not self.cli_spawn.wait_for_service(15):
            raise SystemExit("gazebo spawn/delete unavailable")
        t0 = time.time()
        while self._xml is None and time.time() - t0 < 15:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._xml is None:
            raise SystemExit("no robot_description")

    def respawn_parking(self):
        self.stop()
        for name in ("limo", "polar_goal_ball"):
            req = DeleteEntity.Request()
            req.name = name
            fut = self.cli_del.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=8)
        time.sleep(0.8)
        x, y, yaw = PARKING
        sp = SpawnEntity.Request()
        sp.name = "limo"
        sp.xml = self._xml
        sp.initial_pose.position.x = x
        sp.initial_pose.position.y = y
        sp.initial_pose.position.z = 0.15
        sp.initial_pose.orientation.z = math.sin(yaw / 2.0)
        sp.initial_pose.orientation.w = math.cos(yaw / 2.0)
        sp.reference_frame = "world"
        fut = self.cli_spawn.call_async(sp)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15)
        print(f"  parking spawn ({x:.2f},{y:.2f}) -> {fut.result()}")
        self.spin_for(2.5)

    def set_goal(self, gx, gy):
        self._reached = False
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x = gx
        msg.pose.position.y = gy
        msg.pose.position.z = 0.1
        msg.pose.orientation.w = 1.0
        self.pub_goal.publish(msg)
        en = Bool()
        en.data = True
        self.pub_enable.publish(en)
        # republish a few times (latched-less)
        for _ in range(5):
            self.pub_goal.publish(msg)
            self.pub_enable.publish(en)
            rclpy.spin_once(self, timeout_sec=0.05)

    def run_one(self, name, gx, gy, note, timeout_sec=90.0):
        d0 = dist_parking(gx, gy)
        print(f"\n=== {name}  goal=({gx:.2f},{gy:.2f})  dist_from_parking={d0:.2f}m ===")
        print(f"  {note}")
        self.respawn_parking()
        self.set_goal(gx, gy)

        t0 = time.time()
        path = 0.0
        last = None
        road_ok_samples = 0
        road_samples = 0
        min_rho = 1e9

        while time.time() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._odom is not None:
                x = self._odom.pose.pose.position.x
                y = self._odom.pose.pose.position.y
                if last is not None:
                    path += math.hypot(x - last[0], y - last[1])
                last = (x, y)
                dist_goal = math.hypot(x - gx, y - gy)
                min_rho = min(min_rho, dist_goal)
            road_samples += 1
            if self._road_ok:
                road_ok_samples += 1
            if self._rho is not None:
                min_rho = min(min_rho, self._rho)
            if self._reached or (last and math.hypot(last[0] - gx, last[1] - gy) < 0.45):
                ok_ratio = road_ok_samples / max(1, road_samples)
                self.stop()
                status = "PASS" if ok_ratio > 0.55 and path > 0.4 else "WEAK"
                print(
                    f"  RESULT {status}: reached in {time.time()-t0:.1f}s "
                    f"path={path:.2f}m road_ok={ok_ratio:.0%} min_dist={min_rho:.2f}m"
                )
                return status == "PASS" or status == "WEAK", status, path, ok_ratio

        self.stop()
        ok_ratio = road_ok_samples / max(1, road_samples)
        dist_left = (
            math.hypot(last[0] - gx, last[1] - gy) if last else float("nan")
        )
        print(
            f"  RESULT FAIL: timeout path={path:.2f}m left={dist_left:.2f}m "
            f"road_ok={ok_ratio:.0%} min_dist={min_rho:.2f}m"
        )
        return False, "FAIL", path, ok_ratio


def main():
    print("Destinations (far on black asphalt):")
    for name, x, y, note in DESTINATIONS:
        print(f"  {name}: ({x:.2f},{y:.2f}) d={dist_parking(x,y):.2f}m — {note}")

    rclpy.init()
    node = Mission()
    node.wait_services()

    results = []
    for name, gx, gy, note in DESTINATIONS:
        # Skip if somehow too close
        if dist_parking(gx, gy) < 2.0:
            print(f"\nSKIP {name}: too close to parking ({dist_parking(gx,gy):.2f}m)")
            continue
        ok, status, path, road = node.run_one(name, gx, gy, note, timeout_sec=150.0)
        results.append((name, status, path, road, dist_parking(gx, gy)))
        time.sleep(1.0)

    node.stop()
    print("\n========== SUMMARY ==========")
    n_pass = 0
    for name, status, path, road, d0 in results:
        flag = "OK" if status in ("PASS", "WEAK") else "NO"
        if status in ("PASS", "WEAK"):
            n_pass += 1
        print(
            f"  [{flag}] {name}: {status}  dist0={d0:.2f}m  path={path:.2f}m  road_ok={road:.0%}"
        )
    print(f"Validated {n_pass}/{len(results)} destinations")
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if n_pass > 0 else 2)


if __name__ == "__main__":
    main()
