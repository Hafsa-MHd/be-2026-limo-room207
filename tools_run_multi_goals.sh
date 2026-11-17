#!/usr/bin/env bash
# Cycle destinations: park → goal → next (same style as the ChatGPT general-planner run).
set -eo pipefail
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
sed -i 's/\r$//' "$ROOT"/tools_*.sh "$ROOT"/tools_test_control_once.py 2>/dev/null || true

RUN_DIR="/tmp/limo_full/general-planner-rerun-$(date +%H%M%S)"
mkdir -p "$RUN_DIR"
echo "$RUN_DIR" > /tmp/limo_full/general-planner-latest

# 02 = NE roundabout black ring (early so the demo shows circle mastery even if a later spawn fails).
cat > "$RUN_DIR/points.txt" <<'EOF'
01 1.391 -2.561
02 2.100 2.110
03 0.479 2.709
04 -1.863 1.550
05 -1.925 0.272
06 -0.322 2.542
07 -1.453 2.761
08 0.432 -2.738
09 -2.058 -1.312
10 -1.620 -0.987
11 2.001 -1.192
EOF

have_odom() {
  timeout 5 ros2 topic echo /odom --once >/dev/null 2>&1
}

have_desc() {
  timeout 5 ros2 topic echo /robot_description --once >/dev/null 2>&1
}

park_robot() {
  local log=$1
  # If odom already alive, teleport is enough; do not thrash delete/spawn.
  if have_odom; then
    bash "$ROOT/tools_force_parking_now.sh" >>"$log" 2>&1 || true
    sleep 1
    if have_odom; then
      return 0
    fi
  fi
  local attempt
  for attempt in 1 2 3; do
    if ! have_desc; then
      echo "wait robot_description (attempt $attempt)..." | tee -a "$log"
      sleep 2
      continue
    fi
    bash "$ROOT/tools_force_parking_now.sh" >>"$log" 2>&1 || true
    sleep 2
    if have_odom; then
      return 0
    fi
    echo "park left no /odom — retry $attempt" | tee -a "$log"
    sleep 2
  done
  return 1
}

pkill -f 'tools_test_control_once' 2>/dev/null || true
sleep 1

if ! pgrep -f 'gzserver' >/dev/null; then
  echo "ERROR: Gazebo is not running."
  exit 1
fi

if ! pgrep -f 'limo_simple/road_vision' >/dev/null; then
  nohup ros2 run limo_simple road_vision --ros-args \
    --params-file "$ROOT/limo_simple/config/road_vision.yaml" \
    -p use_sim_time:=true >/tmp/limo_full/vision.log 2>&1 &
fi
if ! pgrep -f 'limo_simple/polar_follow' >/dev/null; then
  nohup ros2 run limo_simple polar_follow --ros-args \
    --params-file "$ROOT/limo_simple/config/polar_follow.yaml" \
    -p d8_south_route:=true -p use_sim_time:=true >/tmp/limo_full/map-planner-controller.log 2>&1 &
fi
sleep 3

if ! have_odom; then
  echo "ERROR: no /odom — robot missing. Use tools_rerun_live_demo.sh"
  exit 1
fi

echo "=== multi-goal demo → $RUN_DIR ==="
echo "Watch Gazebo: robot should leave parking toward the red GOAL ball."
echo "Point 02 is the NE roundabout showcase (2.100, 2.110)."
: > "$RUN_DIR/summary.txt"

while read -r idx gx gy; do
  echo ""
  echo "======== POINT $idx  GOAL=($gx, $gy) ========"
  if ! park_robot "$RUN_DIR/point-${idx}-spawn.log"; then
    echo "ERROR: lost /odom after spawn — trying one stack nudge"
    # Restart polar/vision only; do not kill Gazebo.
    pkill -f 'limo_simple/polar_follow' 2>/dev/null || true
    pkill -f 'limo_simple/road_vision' 2>/dev/null || true
    sleep 1
    nohup ros2 run limo_simple road_vision --ros-args \
      --params-file "$ROOT/limo_simple/config/road_vision.yaml" \
      -p use_sim_time:=true >/tmp/limo_full/vision.log 2>&1 &
    nohup ros2 run limo_simple polar_follow --ros-args \
      --params-file "$ROOT/limo_simple/config/polar_follow.yaml" \
      -p d8_south_route:=true -p use_sim_time:=true >/tmp/limo_full/map-planner-controller.log 2>&1 &
    sleep 4
    if ! park_robot "$RUN_DIR/point-${idx}-spawn-retry.log"; then
      echo "$idx $gx $gy RESULT NO_ODOM" | tee -a "$RUN_DIR/summary.txt"
      echo "continuing to next point..."
      continue
    fi
  fi
  set +e
  PYTHONUNBUFFERED=1 python3 -u "$ROOT/tools_test_control_once.py" \
    --goal "$gx" "$gy" --timeout 175 \
    | tee "$RUN_DIR/point-${idx}.log"
  set -e
  result=$(grep -E '^RESULT ' "$RUN_DIR/point-${idx}.log" | tail -1 | awk '{print $2}')
  echo "$idx $gx $gy RESULT ${result:-UNKNOWN}" | tee -a "$RUN_DIR/summary.txt"
  sleep 1
done < "$RUN_DIR/points.txt"

echo ""
echo "=== DONE ==="
cat "$RUN_DIR/summary.txt"
