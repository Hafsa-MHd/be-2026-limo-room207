#!/usr/bin/env bash
# Full live demo restore: relaunch sim if needed, then cycle goals (incl. roundabout).
set -eo pipefail
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
sed -i 's/\r$//' "$ROOT"/tools_*.sh "$ROOT"/tools_test_control_once.py 2>/dev/null || true
mkdir -p /tmp/limo_full

echo "=== stopping old control/test (keep or restart Gazebo) ==="
# Do not match this script name (tools_rerun_live_demo) or we kill ourselves.
pkill -f 'tools_test_control_once' 2>/dev/null || true
pkill -f 'tools_run_multi_goals' 2>/dev/null || true
pkill -f 'limo_simple/polar_follow' 2>/dev/null || true
pkill -f 'limo_simple/road_vision' 2>/dev/null || true
pkill -f 'limo_simple/spawn_parking' 2>/dev/null || true
sleep 1

need_launch=0
if ! pgrep -f 'gzserver.*qcar_large_track' >/dev/null; then
  need_launch=1
fi
# If robot_description is missing, stack is broken → full relaunch.
if ! timeout 3 ros2 topic echo /robot_description --once >/dev/null 2>&1; then
  echo "robot_description missing → full relaunch"
  need_launch=1
fi

if [ "$need_launch" = 1 ]; then
  echo "=== relaunching room207_drive (d8_south_route) ==="
  pkill -9 -f 'gzserver|gzclient|rviz2|room207_drive|joint_state_publisher' 2>/dev/null || true
  sleep 2
  nohup ros2 launch limo_simple room207_drive.launch.py d8_south_route:=true \
    >/tmp/limo_full/live-demo-launch.log 2>&1 &
  echo "waiting for Gazebo + robot..."
  for i in $(seq 1 60); do
    if timeout 2 ros2 topic echo /odom --once >/dev/null 2>&1 \
       && timeout 2 ros2 topic echo /robot_description --once >/dev/null 2>&1; then
      echo "sim ready after ${i}s"
      break
    fi
    sleep 2
  done
else
  echo "=== Gazebo OK — starting polar/vision ==="
  nohup ros2 run limo_simple road_vision --ros-args \
    --params-file "$ROOT/limo_simple/config/road_vision.yaml" \
    -p use_sim_time:=true >/tmp/limo_full/vision.log 2>&1 &
  sleep 1
  nohup ros2 run limo_simple polar_follow --ros-args \
    --params-file "$ROOT/limo_simple/config/polar_follow.yaml" \
    -p d8_south_route:=true -p use_sim_time:=true >/tmp/limo_full/map-planner-controller.log 2>&1 &
  sleep 4
fi

# Wait until polar is the cmd_vel publisher.
for i in $(seq 1 30); do
  if timeout 2 ros2 topic info /cmd_vel 2>/dev/null | grep -q polar_follow; then
    break
  fi
  sleep 1
done

echo "=== force parking ==="
bash "$ROOT/tools_force_parking_now.sh" || true
sleep 2

# Verify odom before demo; if spawn ate the robot, full relaunch once.
if ! timeout 5 ros2 topic echo /odom --once >/dev/null 2>&1; then
  echo "WARNING: no /odom after park — full relaunch recovery"
  pkill -9 -f 'gzserver|gzclient|rviz2|room207_drive|joint_state_publisher|limo_simple' 2>/dev/null || true
  sleep 3
  nohup ros2 launch limo_simple room207_drive.launch.py d8_south_route:=true \
    >/tmp/limo_full/live-demo-launch.log 2>&1 &
  for i in $(seq 1 90); do
    if timeout 2 ros2 topic echo /odom --once >/dev/null 2>&1 \
       && timeout 2 ros2 topic echo /robot_description --once >/dev/null 2>&1; then
      echo "sim recovered after ${i}s"
      break
    fi
    sleep 2
  done
  bash "$ROOT/tools_force_parking_now.sh" || true
  sleep 2
fi

if ! timeout 5 ros2 topic echo /odom --once >/dev/null 2>&1; then
  echo "ERROR: still no /odom — cannot demo"
  exit 1
fi
echo "odom OK — starting multi-goal loop"
exec bash "$ROOT/tools_run_multi_goals.sh"
