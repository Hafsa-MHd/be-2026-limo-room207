#!/usr/bin/env bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set -eo pipefail
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash

python3 "$ROOT/tools_respawn_on_road.py" 2>&1 | tail -20

echo "=== after respawn ==="
timeout 5 ros2 topic echo /vision/road_ok --once || true
timeout 5 ros2 topic echo /control/rho --once || true
timeout 5 ros2 topic echo /control/alpha --once || true
timeout 5 ros2 topic echo /cmd_vel --once || true
timeout 3 ros2 topic hz /cmd_vel 2>&1 | head -6 || true
