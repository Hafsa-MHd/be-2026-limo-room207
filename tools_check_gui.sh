#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws_simple/install/setup.bash 2>/dev/null || true
echo "=== drive log ==="
tail -50 /tmp/limo_full/drive.log || true
echo "=== procs ==="
pgrep -af 'gzserver|gzclient' || echo none
echo "=== odom ==="
timeout 4 ros2 topic echo /odom --once 2>/dev/null | head -18 || echo NO_ODOM
