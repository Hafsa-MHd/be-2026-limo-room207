#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
python3 - <<'PY'
import rclpy
from gazebo_msgs.srv import DeleteEntity
from std_msgs.msg import Bool
import time
rclpy.init()
n = rclpy.create_node("clr")
cli = n.create_client(DeleteEntity, "/delete_entity")
cli.wait_for_service(5)
req = DeleteEntity.Request(); req.name = "polar_goal_ball"
fut = cli.call_async(req)
rclpy.spin_until_future_complete(n, fut, timeout_sec=5)
print("delete_ball", fut.result())
pub = n.create_publisher(Bool, "/control/mission_enable", 10)
time.sleep(0.2)
b = Bool(); b.data = False; pub.publish(b)
time.sleep(0.2)
n.destroy_node(); rclpy.shutdown()
PY
