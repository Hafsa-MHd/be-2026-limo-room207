#!/usr/bin/env python3
"""Follow /vision/center; when parking exit is split, pull toward black asphalt."""

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray


class LaneFollow(Node):
    def __init__(self):
        super().__init__("lane_follow")
        self.declare_parameter("image_width", 640.0)
        self.declare_parameter("v_max", 0.08)
        self.declare_parameter("v_min", 0.03)
        self.declare_parameter("w_max", 0.45)
        self.declare_parameter("k_lat", 1.8)
        self.declare_parameter("k_head", 0.9)
        self.declare_parameter("k_black", 0.55)
        self.declare_parameter("deadband", 0.03)
        self.declare_parameter("lost_timeout_sec", 0.8)
        self.declare_parameter("control_hz", 20.0)

        self._ok = False
        self._cx = 0.0
        self._heading = 0.0
        self._black_bias = 0.0
        self._last_vision = self.get_clock().now()
        self._odom_x = None
        self._odom_y = None

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Bool, "/vision/road_ok", self.on_ok, 10)
        self.create_subscription(Float32MultiArray, "/vision/center", self.on_center, 10)
        self.create_subscription(Float32, "/vision/black_bias", self.on_bias, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)

        period = 1.0 / max(1.0, float(self.get_parameter("control_hz").value))
        self.create_timer(period, self.on_timer)
        self.get_logger().info(
            f"lane_follow v_max={self.get_parameter('v_max').value} "
            f"prefers black asphalt (k_black={self.get_parameter('k_black').value})"
        )

    def on_ok(self, msg: Bool) -> None:
        self._ok = bool(msg.data)

    def on_bias(self, msg: Float32) -> None:
        self._black_bias = float(msg.data)

    def on_center(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 5:
            self._ok = False
            return
        self._cx = float(msg.data[0])
        self._heading = float(msg.data[4])
        if len(msg.data) >= 6:
            self._black_bias = float(msg.data[5])
        self._last_vision = self.get_clock().now()
        self._ok = True

    def on_odom(self, msg: Odometry) -> None:
        self._odom_x = float(msg.pose.pose.position.x)
        self._odom_y = float(msg.pose.pose.position.y)

    def on_timer(self) -> None:
        cmd = Twist()
        age = (self.get_clock().now() - self._last_vision).nanoseconds * 1e-9
        lost = age > float(self.get_parameter("lost_timeout_sec").value)
        if not self._ok or lost:
            self.pub_cmd.publish(cmd)
            return

        width = float(self.get_parameter("image_width").value)
        # Positive err => road center is to the right of image center.
        err = (self._cx - 0.5 * width) / max(1.0, 0.5 * width)

        # Parking exit: if black asphalt is clearly on one side, pull that way.
        # black_bias > 0 => more black on the right => want positive err correction
        # toward right, i.e. steer left when err is computed vs image center...
        # We subtract bias from err so the target is shifted toward black.
        # target_cx = image_center + bias * half_width  => err_target = bias
        # control uses (cx - target) so err' = err - black_bias
        err = err - float(self.get_parameter("k_black").value) * self._black_bias

        dead = float(self.get_parameter("deadband").value)
        if abs(err) < dead:
            err = 0.0

        k_lat = float(self.get_parameter("k_lat").value)
        k_head = float(self.get_parameter("k_head").value)
        w_max = float(self.get_parameter("w_max").value)
        v_max = float(self.get_parameter("v_max").value)
        v_min = float(self.get_parameter("v_min").value)

        steer = -k_lat * err - k_head * self._heading
        steer = max(-w_max, min(w_max, steer))
        # Slow down more in split scenes.
        split = min(1.0, abs(self._black_bias))
        slow = (1.0 - min(1.0, abs(err))) * (1.0 - 0.35 * split)
        speed = v_min + (v_max - v_min) * max(0.0, slow)

        cmd.linear.x = speed
        cmd.angular.z = steer
        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollow()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        stop = Twist()
        try:
            node.pub_cmd.publish(stop)
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
