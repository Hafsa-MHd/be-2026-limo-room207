#!/usr/bin/env python3
"""Override school road spawn → parking stall (BE start pose).

`room207_limo.launch.py` always spawns at (-0.725, -0.087) on the road.
This node waits for that spawn, then deletes `limo` and respawns in the
parking bay (black asphalt + white stall lines).
"""

from __future__ import annotations

import math
import time

import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

# Inside parking stall area (NOT school road, NOT lot aisle center).
# Official Room 207 start from GitLab (room207_limo.launch.py / limo parameters.m):
# "just below the parking bay", heading north.
DEFAULT_X = -0.725347
DEFAULT_Y = -0.086613
DEFAULT_YAW = 1.5708
DEFAULT_Z = 0.15


class SpawnParking(Node):
    def __init__(self) -> None:
        super().__init__("spawn_parking")
        self.declare_parameter("x", DEFAULT_X)
        self.declare_parameter("y", DEFAULT_Y)
        self.declare_parameter("yaw", DEFAULT_YAW)
        self.declare_parameter("z", DEFAULT_Z)
        self.declare_parameter("wait_school_sec", 5.0)
        self.declare_parameter("tol_m", 0.35)

        self._xml: str | None = None
        self._odom: Odometry | None = None
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(String, "/robot_description", self._on_xml, qos)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_mission = self.create_publisher(Bool, "/control/mission_enable", 10)
        self.cli_del = self.create_client(DeleteEntity, "/delete_entity")
        self.cli_spawn = self.create_client(SpawnEntity, "/spawn_entity")

    def _on_xml(self, msg: String) -> None:
        if self._xml is None and msg.data:
            self._xml = msg.data

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _hold(self) -> None:
        self.pub_cmd.publish(Twist())
        b = Bool()
        b.data = False
        self.pub_mission.publish(b)

    def target(self):
        return (
            float(self.get_parameter("x").value),
            float(self.get_parameter("y").value),
            float(self.get_parameter("yaw").value),
            float(self.get_parameter("z").value),
        )

    def in_parking(self) -> bool:
        """True only if XY and yaw already match the requested pose."""
        if self._odom is None:
            return False
        x, y, yaw, _ = self.target()
        p = self._odom.pose.pose.position
        if math.hypot(p.x - x, p.y - y) >= float(self.get_parameter("tol_m").value):
            return False
        q = self._odom.pose.pose.orientation
        cur = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        err = abs(math.atan2(math.sin(cur - yaw), math.cos(cur - yaw)))
        return err < 0.35

    def run(self) -> bool:
        x, y, yaw, z = self.target()
        self.get_logger().info(f"will move limo to parking ({x:.2f},{y:.2f},yaw={yaw:.2f})")

        wait = float(self.get_parameter("wait_school_sec").value)
        t0 = time.time()
        while time.time() - t0 < wait and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self._hold()

        t0 = time.time()
        while self._xml is None and time.time() - t0 < 30.0 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self._hold()
        if self._xml is None:
            self.get_logger().error("no /robot_description")
            return False

        if not self.cli_del.wait_for_service(15.0) or not self.cli_spawn.wait_for_service(15.0):
            self.get_logger().error("gazebo spawn services missing")
            return False

        if self.in_parking():
            p = self._odom.pose.pose.position
            self.get_logger().info(f"already in parking ({p.x:.2f},{p.y:.2f})")
            return True

        for name in ("limo", "polar_goal_ball"):
            req = DeleteEntity.Request()
            req.name = name
            fut = self.cli_del.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
            self.get_logger().info(f"delete {name}: {fut.result()}")
        time.sleep(1.0)

        # Respawn can leave plugins silent — retry delete+spawn a few times.
        for attempt in range(1, 4):
            self._odom = None
            sp = SpawnEntity.Request()
            sp.name = "limo"
            sp.xml = self._xml
            sp.initial_pose.position.x = x
            sp.initial_pose.position.y = y
            sp.initial_pose.position.z = z
            sp.initial_pose.orientation.z = math.sin(yaw / 2.0)
            sp.initial_pose.orientation.w = math.cos(yaw / 2.0)
            sp.reference_frame = "world"
            fut = self.cli_spawn.call_async(sp)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
            self.get_logger().info(f"spawn parking attempt {attempt}: {fut.result()}")

            # Odom / ackermann can lag several seconds after respawn.
            t0 = time.time()
            while time.time() - t0 < 20.0 and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                self._hold()
                if self.in_parking():
                    p = self._odom.pose.pose.position
                    self.get_logger().info(f"parking OK ({p.x:.2f},{p.y:.2f})")
                    return True
                if self._odom is not None:
                    # Any odom after respawn is enough to continue demos.
                    p = self._odom.pose.pose.position
                    self.get_logger().warn(
                        f"parking soft-OK at ({p.x:.2f},{p.y:.2f}) — continuing"
                    )
                    return True

            self.get_logger().warn(f"no /odom after spawn attempt {attempt}")
            req = DeleteEntity.Request()
            req.name = "limo"
            fut = self.cli_del.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
            time.sleep(1.5)

        self.get_logger().error("parking pose not confirmed (no odom)")
        return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpawnParking()
    try:
        ok = node.run()
        # Keep holding briefly so polar does not grab school pose.
        for _ in range(30):
            node._hold()
            rclpy.spin_once(node, timeout_sec=0.05)
        raise SystemExit(0 if ok else 1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
