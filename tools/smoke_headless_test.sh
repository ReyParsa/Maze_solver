#!/usr/bin/env bash
set -euo pipefail

# Smoke test for headless Gazebo run
# - launches the bringup in headless mode
# - waits for planner node and robot creation
# - checks for /tf and /cmd_vel (if planner produces commands)
# - exits 0 on success, non-zero on failure

WORKSPACE_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOGDIR="/tmp/robot_maze_smoke"
mkdir -p "$LOGDIR"
LAUNCH_LOG="$LOGDIR/launch.log"
SMOKE_LOG="$LOGDIR/smoke.log"
PIDFILE="$LOGDIR/launch.pid"

# Environment (adjust if your ROS 2 distro path differs)
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="$WORKSPACE_ROOT/install/setup.bash"

echo "[smoke] workspace: $WORKSPACE_ROOT" | tee "$SMOKE_LOG"
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
# Some ROS setup scripts reference variables that may be unset when this script
# runs with 'set -u'. Temporarily disable 'nounset' while sourcing them.
set +u
source "$ROS_SETUP"
source "$WS_SETUP"
set -u

# Kill stray gz processes
pkill -f gz || true
sleep 0.5

# Start launch in background
ros2 launch robot_maze_simulation bringup_launch.py \
  maze_seed:=777 maze_rows:=6 maze_cols:=6 cell_size:=0.3 maze_single_mesh:=true headless:=true planner:=astar \
  &> "$LAUNCH_LOG" &
LAUNCH_PID=$!
echo $LAUNCH_PID > "$PIDFILE"

echo "[smoke] started launch (pid=$LAUNCH_PID), logging -> $LAUNCH_LOG" | tee -a "$SMOKE_LOG"

# Helper: poll for condition with timeout
poll_for() {
  local timeout_secs=$1; shift
  local cmd="$@"
  local waited=0
  local interval=1
  while [ $waited -lt $timeout_secs ]; do
    if bash -lc "$cmd" &> /dev/null; then
      return 0
    fi
    sleep $interval
    waited=$((waited + interval))
  done
  return 1
}

# Wait for planner node to appear
echo "[smoke] waiting for planner node (timeout 30s)" | tee -a "$SMOKE_LOG"
if poll_for 30 "ros2 node list | grep -i maze_planner || ros2 node list | grep -i planner"; then
  echo "[smoke] planner node present" | tee -a "$SMOKE_LOG"
else
  echo "[smoke] planner node NOT found within timeout" | tee -a "$SMOKE_LOG"
  cat "$LAUNCH_LOG" | tail -n 200 >> "$SMOKE_LOG"
  kill $LAUNCH_PID 2>/dev/null || true
  pkill -f gz || true
  exit 2
fi

# Wait for /tf to publish
echo "[smoke] waiting for /tf (timeout 10s)" | tee -a "$SMOKE_LOG"
if poll_for 10 "ros2 topic list | grep -x '/tf'"; then
  echo "[smoke] /tf present" | tee -a "$SMOKE_LOG"
  # grab one message (non-blocking)
  timeout 3 ros2 topic echo -n 1 /tf > "$LOGDIR/tf_sample.log" || true
else
  echo "[smoke] /tf not present" | tee -a "$SMOKE_LOG"
fi

# Check for cmd_vel (planner output) and capture a sample
if ros2 topic list | grep -x '/cmd_vel' &> /dev/null; then
  echo "[smoke] /cmd_vel topic present; capturing sample" | tee -a "$SMOKE_LOG"
  timeout 3 ros2 topic echo -n 1 /cmd_vel > "$LOGDIR/cmd_vel_sample.log" || true
else
  echo "[smoke] /cmd_vel not present (planner may not publish immediately)" | tee -a "$SMOKE_LOG"
fi

# Summary
echo "[smoke] logs: $LAUNCH_LOG, $LOGDIR" | tee -a "$SMOKE_LOG"

# Clean up: stop launch and gz
kill $LAUNCH_PID 2>/dev/null || true
sleep 0.5
pkill -f gz || true

# Print short summary for user
echo "--- SMOKE SUMMARY ---" | tee -a "$SMOKE_LOG"
cat "$SMOKE_LOG" | tail -n 200

# Determine pass/fail: at minimum planner node and /tf present
if ros2 node list | grep -i maze_planner &> /dev/null && ros2 topic list | grep -x '/tf' &> /dev/null; then
  echo "[smoke] PASS" | tee -a "$SMOKE_LOG"
  exit 0
else
  echo "[smoke] FAIL" | tee -a "$SMOKE_LOG"
  exit 3
fi
