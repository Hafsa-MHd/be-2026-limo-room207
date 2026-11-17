#!/usr/bin/env bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source ~/ros2_ws_simple/install/setup.bash
CFG="$ROOT/limo_simple/config/polar_follow.yaml"
pkill -f 'limo_simple.polar_follow|polar_follow.py' 2>/dev/null || true
sleep 1
nohup ros2 run limo_simple polar_follow --ros-args --params-file "$CFG" >/tmp/polar.log 2>&1 &
disown || true
sleep 3
pgrep -af polar_follow || echo NO_POLAR
tail -8 /tmp/polar.log
