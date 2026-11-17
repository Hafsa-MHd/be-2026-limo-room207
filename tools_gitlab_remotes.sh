#!/usr/bin/env bash
set -e
for d in environment room207-limo limo recording teleoperation group-informations; do
  echo "==== $d ===="
  p="$HOME/ros2_ws/src/$d"
  if [ -d "$p/.git" ] || [ -f "$p/.git" ]; then
    git -C "$p" remote -v
    git -C "$p" rev-parse --short HEAD
    git -C "$p" branch -vv | head -3
  elif [ -d "$p" ]; then
    echo "present but not a git repo"
    ls "$p" | head
  else
    echo MISSING
  fi
done
echo "==== room207 README clone section ===="
sed -n '1,120p' "$HOME/ros2_ws/src/room207-limo/README.md"
echo "==== environment README ===="
sed -n '1,80p' "$HOME/ros2_ws/src/environment/README.md"
