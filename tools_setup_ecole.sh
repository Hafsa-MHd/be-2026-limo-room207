#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash

pkill -x gzserver 2>/dev/null || true
pkill -x gzclient 2>/dev/null || true
pkill -x rviz2 2>/dev/null || true
pkill -f '/limo_simple/' 2>/dev/null || true
pkill -f 'spawn_entity.py' 2>/dev/null || true
sleep 2

cd "$HOME/ros2_ws/src"

# Ignore Foxy copies when building on Humble
touch limo/ros2_foxy/COLCON_IGNORE
# room207 package lives under ros2_foxy folder name but builds on Humble too
# (school layout); keep it buildable.

ln -sfn limo/ros2_humble/limo_ros2 limo_ros2

cd "$HOME/ros2_ws"
rm -rf build/room207_limo install/room207_limo 2>/dev/null || true

echo "==== colcon build ===="
colcon build --symlink-install \
  --packages-up-to room207_limo ensem_room207 qcar_large_track agilex_sand_table limo_car teleoperation recording

source install/setup.bash
echo "==== pkg prefixes ===="
for p in room207_limo ensem_room207 qcar_large_track limo_car teleoperation recording; do
  if ros2 pkg prefix "$p" >/dev/null 2>&1; then
    echo "OK $p -> $(ros2 pkg prefix "$p")"
  else
    echo "MISSING $p"
  fi
done

# Show which launch is installed
echo "==== room207 launches ===="
ros2 pkg prefix room207_limo
ls "$(ros2 pkg prefix room207_limo)/share/room207_limo/launch" 2>/dev/null || true

echo SETUP_ECOLE_OK
