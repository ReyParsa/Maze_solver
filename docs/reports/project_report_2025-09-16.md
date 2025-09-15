# Maze Solver Project — Comprehensive Report (as of 2025-09-16)

Authoring assistant: GitHub Copilot
Workspace: /home/rey/Desktop/Maze Solver/robot_maze_ws
Repository: Maze_solver (branch: master)

## 1) Executive Summary

This project implements a ROS 2 Jazzy + Gazebo simulation where a differential-drive robot navigates a maze using the A* planner (with optional RRT/RRT*), relying only on odometry. The final presentation requirement is a reproducible demo that simultaneously shows Gazebo GUI and RViz. Maze generation is decoupled: the launch must only load a pre-generated static world at `worlds/maze.world` and never generate on launch.

Key outcomes:
- End-to-end pipeline: maze publisher → A* planner → path follower → cmd_vel to Gazebo via ros_gz_bridge.
- GUI reliability: hardened Gazebo GUI for VM use with software rendering and minimal plugins.
- Strict world source: launch fails fast unless `src/robot_maze_simulation/worlds/maze.world` (or installed share) exists.

## 2) Project Timeline & History

- Phase 1: Core pipeline
  - Implemented `maze_publisher_node`, centered occupancy grid (±10 m, 0.4 m cells), and origin semantics.
  - A* planner with Manhattan heuristic and post-processing (deduplication, collinearity pruning, densification). Reduced replan churn with gating.
  - Path follower publishes Twist from odom-only pose: lookahead control, angular-aware linear throttling, turn-in-place for large heading error.
  - Odom→Pose republisher provides `PoseStamped` on `/robot_pose` with source locking.

- Phase 2: Stability & recovery
  - Start-cell clearing to avoid immediate collision.
  - Recovery behaviors: no-progress detection (timeout + min delta), alternating arc-turn recovery, waypoint skip on recovery, nearest-index snapping.

- Phase 3: Gazebo GUI hardening
  - VM-safe environment: enforce software rendering (llvmpipe), disable DRI3, set Qt/RHI to software, cap GL/GLSL versions.
  - Minimal GUI plugin config to reduce load and avoid EGL/ZINK issues.
  - Headless mode validated for development; later disabled for final presentation.

- Phase 4: Decouple maze generation
  - Removed any on-launch generation; launch now strictly loads a static world from `worlds/maze.world`.
  - Deleted placeholder worlds to remain content-agnostic.
  - Added fail-fast preflight if `maze.world` is missing with clear remediation instructions.

- Phase 5: Dual visualization & polish
  - RViz layout added for `/planned_path`, `/maze_occupancy`, robot model, and `/robot_pose`.
  - Launch orchestration reworked for GUI + RViz, with timed startup to avoid race conditions.

## 3) Current Architecture

- Simulation & orchestration: `src/robot_maze_simulation/launch/bringup_launch.py`
  - Resolves `worlds/maze.world` from source or installed share; errors out if absent.
  - Starts Gazebo server and GUI; spawns robot (URDF xacro from `robot_maze_description/urdf/maze_generate/slambot.urdf.xacro`).
  - Bridges `/cmd_vel` and `/odom` via `ros_gz_bridge`.
  - Launches planner, follower, odom→pose republisher, and maze publisher with staggered timers.
  - Starts RViz2 with `config/maze_demo.rviz`.

- Robot description: `src/robot_maze_description/urdf/maze_generate/slambot.urdf.xacro` (from directory listing)

- Configs: `src/robot_maze_simulation/config/`
  - `gui_minimal.config`: minimal GUI plugin set for stability.
  - `maze_demo.rviz`: visualization layout for RViz2.

- Worlds: `src/robot_maze_simulation/worlds/`
  - Present: `maze_world.sdf`, `maze_world_fixed.sdf`
  - Required for launch: `maze.world` (must be generated/copied by user’s external tool). Not currently present.

## 4) Components Overview

- maze_publisher_node
  - Publishes occupancy grid centered at origin. Parameters: rows, cols, cell_size. Topic: `/maze_occupancy`.

- planner_astar_node (default)
  - Parameters: goal_x=9.6, goal_y=9.6; default_start from lower-left corner; plan_on_timer at 1 Hz.
  - Post-process path to improve smoothness and reduce oscillations.
  - Publishes `nav_msgs/Path` on `/planned_path`.

- path_follower_node
  - Parameters tuned for VM stability: lookahead=0.4, ang thresholds, speed limits.
  - Recovery behaviors and waypoint skipping when stuck.
  - Publishes `/cmd_vel` which is bridged to Gazebo.

- odom_to_pose_node
  - Converts `/odom` → `/robot_pose` (`geometry_msgs/PoseStamped`), with source locking.

- ros_gz_bridge parameter_bridge
  - Bridges model and generic topics for cmd_vel and odometry; optional `/tf`.

## 5) Launch & Runbook

Prerequisites:
- Place a valid `maze.world` at `src/robot_maze_simulation/worlds/maze.world` (preferred), or install location.
- Build and source the workspace.

Quick run:
- GUI + RViz (recommended):
  - `ros2 launch robot_maze_simulation bringup_launch.py with_rviz:=true headless:=false force_software_rendering:=true`
- Headless server (dev/testing):
  - `ros2 launch robot_maze_simulation bringup_launch.py headless:=true`

Notes:
- If `maze.world` is missing, the launch throws a RuntimeError with the candidate paths to fix.
- RViz fixed frame may need switching between `map` and `odom` depending on TF.

## 6) Decisions & Rationale

- Decouple generation from launch to guarantee reproducibility and align with the user’s external tooling.
- Force software rendering in VM contexts to avoid EGL/ZINK crashes; provide a minimal GUI for responsiveness.
- Fixed goal (9.6, 9.6) to match a 0.4 m cell grid across ±10 m coverage; simplifies evaluation.
- Start with A* as default for predictability; keep RRT/RRT* switches as optional.

## 7) Known Issues and Resolutions

- Gazebo GUI crash on VM: mitigated via software rendering and minimal plugins.
- Missing world file leads to Gazebo server exit: addressed with fail-fast check and explicit instructions.
- README contains outdated args (`use_static_world`, `static_world_path`): should be updated to reflect strict `worlds/maze.world` behavior.

## 8) Current Status (2025-09-16)

- Build: OK for `robot_maze_simulation` (recent build logs showed success). Other packages previously built successfully.
- Launch: Fails early if `worlds/maze.world` is absent (current repo state). Place the file to proceed.
- Worlds present: `maze_world.sdf`, `maze_world_fixed.sdf` (not used by bringup). Required file missing: `maze.world`.

## 9) Requirements Coverage

- “Solve maze via A* using odom only” — Implemented and integrated.
- “Centered grid ±10 m, 0.4 m cells; goal (9.6, 9.6)” — Implemented.
- “Robust follower with recovery” — Implemented.
- “Gazebo GUI + RViz simultaneously” — Implemented; GUI hardened for VMs.
- “Decouple generation; launch must only load worlds/maze.world” — Implemented; enforced with preflight error.

## 10) Quality Gates (latest pass)

- Build: PASS for simulation pkg (RelWithDebInfo). No compile errors reported.
- Lint/Typecheck: Not fully enforced; minor unused imports may exist.
- Unit tests: None present. Behavior validated via manual runs.
- Smoke test: Headless prior to GUI hardening passed; GUI runs when a valid world exists, given VM-safe env.

## 11) Next Steps

- Provide or copy `src/robot_maze_simulation/worlds/maze.world` from your external generator and relaunch.
- Update `README.md` to remove deprecated args and document the strict world path workflow.
- Optionally add minimal unit tests for planner and follower logic.
- Consider parameterizing goal or loading from world metadata if needed.

## 12) Appendix

- Launch file excerpt references (current): `robot_maze_simulation/launch/bringup_launch.py` resolves `worlds/maze.world`, sets VM-safe env vars, starts Gazebo processes, spawns robot, bridges topics, and launches planner/follower/RViz with timers.
- Config files: `config/gui_minimal.config`, `config/maze_demo.rviz`.
- Worlds directory currently contains `maze_world.sdf`, `maze_world_fixed.sdf`; required `maze.world` missing.
