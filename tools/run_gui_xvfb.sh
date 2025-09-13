#!/usr/bin/env bash
set -eu

# Lightweight helper to attempt running the GUI inside Xvfb on :99
# - does not install packages (no sudo)
# - sets software GL env and launches the GUI for a configurable timeout

TIMEOUT_SECS=${1:-30}
WORKSPACE_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOGFILE="/tmp/gz_gui_xvfb_helper.log"

echo "[gui-helper] workspace: $WORKSPACE_ROOT" | tee "$LOGFILE"

# Check prerequisites
command -v Xvfb >/dev/null || { echo "[gui-helper] Xvfb not found in PATH. Install: sudo apt install xvfb" | tee -a "$LOGFILE"; exit 2; }
command -v xwd >/dev/null || { echo "[gui-helper] xwd not found in PATH. Install: sudo apt install x11-apps" | tee -a "$LOGFILE"; exit 2; }

# Start Xvfb on :99
rm -f /tmp/.X99-lock /tmp/.X99-unix || true
Xvfb :99 -screen 0 1280x720x24 & XVFB_PID=$!
echo "[gui-helper] started Xvfb (pid=$XVFB_PID)" | tee -a "$LOGFILE"
sleep 0.8

# configure display + software GL
export DISPLAY=:99
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=swrast
export QT_QPA_PLATFORM=xcb

# Safely source ROS workspace/setup scripts (some may reference unset variables).
set +u
cd "$WORKSPACE_ROOT"
source /opt/ros/jazzy/setup.bash || true
source install/setup.bash || true
set -u

pkill -f gz || true

echo "[gui-helper] launching GUI (timeout=${TIMEOUT_SECS}s)" | tee -a "$LOGFILE"
timeout ${TIMEOUT_SECS}s ros2 launch robot_maze_simulation bringup_launch.py maze_seed:=777 maze_rows:=6 maze_cols:=6 cell_size:=0.3 maze_single_mesh:=true headless:=false planner:=astar 2>&1 | tee -a "$LOGFILE" || true

sleep 1
# screenshot
xwd -root -display :99 -silent -out /tmp/gz_xvfb_helper.xwd || true
[ -f /tmp/gz_xvfb_helper.xwd ] && convert /tmp/gz_xvfb_helper.xwd /tmp/gz_xvfb_helper.png || true

# cleanup
pkill -f gz || true
kill $XVFB_PID 2>/dev/null || true

echo "[gui-helper] finished, logs: $LOGFILE, screenshot: /tmp/gz_xvfb_helper.png" | tee -a "$LOGFILE"
