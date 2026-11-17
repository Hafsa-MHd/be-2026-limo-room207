#!/usr/bin/env bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set -eo pipefail
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash

pkill -f 'limo_simple.lane_follow|lane_follow.py' 2>/dev/null || true
sleep 0.4

echo "=== procs ==="
pgrep -af 'polar_follow|road_vision|lane_follow' || echo none

# Keep a single polar if duplicates
POLARS=$(pgrep -f 'limo_simple.polar_follow|polar_follow.py' || true)
COUNT=$(echo "$POLARS" | grep -c . || true)
if [ "${COUNT:-0}" -gt 2 ]; then
  echo "too many polar procs — restarting one"
  pkill -f 'limo_simple.polar_follow|polar_follow.py' 2>/dev/null || true
  sleep 1
  bash "$ROOT/tools_restart_polar_only.sh"
fi

python3 "$ROOT/tools_check_violation_fix.py"
