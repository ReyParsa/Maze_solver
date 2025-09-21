This workspace contains a ROS 2 + Gazebo simulation of a maze-solving robot and several planner implementations (A*, RRT, RRT*).

Be concise and actionable: the file below highlights the project's structure, important files, developer workflows, and concrete examples an AI coding agent should follow to make safe, small, and correct changes.

Key points to know
- Top-level packages: `robot_maze_description` (URDF/xacro, meshes), `robot_maze_simulation` (worlds, launch, tools), `robot_maze_planners` (planner implementations, nodes, utils).
- Main launch: `src/robot_maze_simulation/launch/bringup_launch.py` — use this to run the full system (has planner choice, headless/gui flags, maze sizing).
- Planners live in `src/robot_maze_planners/planners/` (files: `astar_planner.py`, `rrt_planner.py`, `rrt_star_planner.py`). Nodes live in `src/robot_maze_planners/nodes/` (e.g. `path_follower_node.py`).

What to change and how
- When adding or modifying a planner: implement a class under `robot_maze_planners/planners/` with a `plan()` method that returns a list of (x, y) world points. Keep world↔cell math consistent with `MazeGrid` in `robot_maze_planners/utils/maze_helpers.py`.
- Node entry points must be registered as console scripts in the Python package (see existing entry points in `setup.py` / `package.xml` under `robot_maze_planners`). For CMake packages, follow existing `install(PROGRAMS ...)` patterns in the simulation package.
- Update `bringup_launch.py` to add new planner options. Prefer minimal, backward-compatible changes to the launch arguments and default behavior.

Build, run, and debug (quick commands)
- Build workspace: `colcon build --symlink-install` from repo root; then `source install/setup.bash`.
- Launch full sim (A* with GUI):
  - `ros2 launch robot_maze_simulation bringup_launch.py planner:=astar headless:=false`
- Headless CI/quick run: `ros2 launch robot_maze_simulation bringup_launch.py planner:=astar headless:=true`
- Inspect runtime params/topics (examples):
  - `ros2 param list /maze_planner`
  - `ros2 topic echo -n 1 /planned_path`

Project-specific conventions
- The maze grid is centered at world origin. Calculations use `origin_x = - (cols * cell_size) / 2`. Use `MazeGrid` helpers for conversions; do not reimplement conversions ad-hoc.
- `maze_occupancy` publishes wall segments as pairs of poses in a `nav_msgs/Path`; code expects this format and uses `parse_maze` to rasterize walls.
- Planners output `nav_msgs/Path` in the `map` frame with densified/pruned waypoints. Path follower expects a polyline in `map`.
- Planner parameters are declared on the node (e.g., `goal_x`, `cell_size`, `inflation_radius`). Use `ros2 param` to inspect and confirm defaults when changing behavior.

Testing and CI hints
- Unit tests live under `src/robot_maze_planners/test/` and use pytest. Run `pytest -q` from workspace root after sourcing the workspace environment.
- Use `tools/smoke_headless_test.sh` for a quick headless integration smoke check — it builds sim + planners and asserts basic topics.

Integration points and external deps
- Requires ROS 2 Jazzy and Gazebo Harmonic with `ros_gz_*` bridge packages. The launch file uses `ros_gz_sim` and `ros_gz_bridge` to spawn the robot and bridge topics.
- Optional Nav2 components (`nav2_map_server`) are used if present — do not hard-rely on them in changes.

Small edits policy (for AI agents)
- Prefer minimal, focused edits with unit tests added or updated. When touching planner logic, also update or add a small unit test in `src/robot_maze_planners/test/` that asserts world↔cell conversions or a short plan output for a tiny grid.
- Avoid wide refactors across packages in a single change. If adding new CLI/launch args, keep defaults unchanged and document new args in `bringup_launch.py`.

Files to inspect when changing behavior
- `src/robot_maze_simulation/launch/bringup_launch.py` (launch args, node wiring)
- `src/robot_maze_simulation/tools/generate_maze_sdf.py` (maze generation format and flags)
- `src/robot_maze_planners/utils/maze_helpers.py` (MazeGrid and parse_maze)
- `src/robot_maze_planners/planners/astar_planner.py` (A* implementation pattern)
- `src/robot_maze_planners/nodes/path_follower_node.py` (controller expectations)

If anything is unclear or you need examples to modify specific files, ask for that file and I'll provide focused guidance or make a small, tested change.
