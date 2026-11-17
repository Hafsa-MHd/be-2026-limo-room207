#!/usr/bin/env bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash
sed -i 's/\r$//' "$ROOT/tools_analyze_split.py"
python3 "$ROOT/tools_analyze_split.py"
