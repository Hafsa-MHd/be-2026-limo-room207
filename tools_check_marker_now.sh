#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash

echo "=== procs ==="
pgrep -af gzserver | head -2 || echo no_gz
pgrep -af road_vision | head -2 || echo no_vision
pgrep -af polar_follow | head -2 || echo no_polar
pgrep -af rviz2 | head -2 || echo no_rviz

echo "=== topics ==="
ros2 topic list 2>/dev/null | grep -E 'control|vision|cmd_vel|rgb' || true

echo "=== road_ok ==="
timeout 4 ros2 topic echo /vision/road_ok --once || echo NO_OK
echo "=== center ==="
timeout 4 ros2 topic echo /vision/center --once || echo NO_CENTER
echo "=== goal ==="
timeout 4 ros2 topic echo /control/goal --once || echo NO_GOAL
echo "=== marker ns ==="
timeout 4 ros2 topic echo /control/goal_marker --once 2>&1 | head -40 || echo NO_MARKER
echo "=== logs ==="
tail -30 /tmp/drive_polar.log 2>/dev/null || true
tail -20 /tmp/polar.log 2>/dev/null || true
