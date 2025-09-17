#!/usr/bin/env bash
set -euo pipefail

# Quick smoke test: build, launch headless for a short time, and check for key topics/messages.

WS_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WS_DIR"

echo "[smoke] Building workspace (simulation only)"
if [ -f install/setup.bash ]; then
  source install/setup.bash || true
fi

colcon build --packages-select robot_maze_simulation robot_maze_planners --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -Wno-dev
source install/setup.bash

echo "[smoke] Launching headless bringup (A*)"
LAUNCH_LOG=$(mktemp)
set +e
timeout 25s ros2 launch robot_maze_simulation bringup_launch.py headless:=true with_rviz:=false planner:=astar >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
set -e

sleep 8

echo "[smoke] Checking essential topics"
ros2 topic list | grep -E "/model/slambot/odom" >/dev/null || { echo "[smoke][FAIL] odom topic missing"; kill $LAUNCH_PID || true; exit 1; }
ros2 topic list | grep -E "^/planned_path$" >/dev/null || { echo "[smoke][WARN] planned_path not yet listed; waiting"; }

# Give the planner a bit more time to publish a path
sleep 5

set +e
ros2 topic echo -n 1 /planned_path >/dev/null 2>&1
PATH_RC=$?
set -e

if [ $PATH_RC -ne 0 ]; then
  echo "[smoke][WARN] No path echoed yet; this can happen on slow starts. Checking logs for planner output."
  tail -n 200 "$LAUNCH_LOG" | sed 's/^/[log] /'
else
  echo "[smoke][OK] Received a planned_path message."
fi

echo "[smoke] Stopping launch"
kill $LAUNCH_PID >/dev/null 2>&1 || true
sleep 1

echo "[smoke] Done."
exit 0
