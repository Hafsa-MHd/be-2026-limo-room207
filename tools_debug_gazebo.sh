#!/usr/bin/env bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set -e
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

bash "$ROOT/tools_kill_gazebo.sh"

export DISPLAY="${DISPLAY:-:0}"
world="$HOME/ros2_ws/install/qcar_large_track/share/qcar_large_track/worlds/qcar_large_track.world"
rm -f /tmp/gz_out.txt
gzserver "$world" -s libgazebo_ros_init.so -s libgazebo_ros_factory.so > /tmp/gz_out.txt 2>&1 &
GPID=$!
echo "GPID=$GPID"
sleep 12
if kill -0 "$GPID" 2>/dev/null; then
  echo GZSERVER_ALIVE
else
  echo GZSERVER_DEAD
  wait "$GPID" || echo "EXIT=$?"
fi
echo "==== stdout ===="
tail -60 /tmp/gz_out.txt || true
kill "$GPID" 2>/dev/null || true
pkill -x gzserver 2>/dev/null || true
fuser -k 11345/tcp 2>/dev/null || true
echo DEBUG_DONE
