#!/usr/bin/env python3
"""Force-spawn a huge bright GOAL marker in Gazebo ahead of the robot."""

import math
import time

import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# Very large, bright red so it is obvious in top-down Gazebo view.
SDF = """<?xml version="1.0"?>
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


def main():
    rclpy.init()
    node = Node("force_goal_ball")
    box = {"odom": None, "cx": 320.0}

    node.create_subscription(Odometry, "/odom", lambda m: box.__setitem__("odom", m), 10)
    node.create_subscription(
        Float32MultiArray,
        "/vision/center",
        lambda m: box.__setitem__("cx", float(m.data[0])) if len(m.data) >= 1 else None,
        10,
    )

    t0 = time.time()
    while box["odom"] is None and time.time() - t0 < 8:
        rclpy.spin_once(node, timeout_sec=0.1)
    if box["odom"] is None:
        raise SystemExit("no odom")

    o = box["odom"]
    x = o.pose.pose.position.x
    y = o.pose.pose.position.y
    q = o.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    err = (box["cx"] - 320.0) / 320.0
    dx, dy = 0.55, -err * 0.35
    c, s = math.cos(yaw), math.sin(yaw)
    gx = x + c * dx - s * dy
    gy = y + s * dx + c * dy
    print(f"robot=({x:.2f},{y:.2f}) yaw={yaw:.2f} goal=({gx:.2f},{gy:.2f})")

    delete = node.create_client(DeleteEntity, "/delete_entity")
    spawn = node.create_client(SpawnEntity, "/spawn_entity")
    delete.wait_for_service(10)
    spawn.wait_for_service(10)

    for name in ("polar_goal_ball",):
        d = DeleteEntity.Request()
        d.name = name
        fut = delete.call_async(d)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5)
        print("delete", fut.result())
    time.sleep(0.4)

    sreq = SpawnEntity.Request()
    sreq.name = "polar_goal_ball"
    sreq.xml = SDF
    sreq.initial_pose.position.x = gx
    sreq.initial_pose.position.y = gy
    sreq.initial_pose.position.z = 0.0
    sreq.initial_pose.orientation.w = 1.0
    sreq.reference_frame = "world"
    fut = spawn.call_async(sreq)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
    res = fut.result()
    print("spawn", res)
    if not res or not res.success:
        raise SystemExit(2)
    print("LOOK IN GAZEBO: big RED ball + YELLOW pole ahead of the robot")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
