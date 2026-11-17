#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
mkdir -p "$HOME/ros2_ws_simple/src" "$HOME/ros2_ws_simple/captures"
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/limo_simple"
ln -sfn "$SRC" "$HOME/ros2_ws_simple/src/limo_simple"
ls -l "$HOME/ros2_ws_simple/src/"
test -f "$HOME/ros2_ws_simple/src/limo_simple/package.xml"
cd "$HOME/ros2_ws_simple"
colcon build --packages-select limo_simple --symlink-install
source install/setup.bash
ros2 pkg prefix limo_simple
ros2 pkg executables limo_simple
echo BUILD_OK
