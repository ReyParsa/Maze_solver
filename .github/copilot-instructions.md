Project: Robot Maze Solver (ROS2 + Gazebo)

Goal: Help AI coding agents become productive quickly in this ROS2 workspace by highlighting the architecture, key files, build/test workflows, conventions, and integration points.

Key components (big picture)
- `src/robot_maze_simulation`: Gazebo worlds, launch files and tools. Primary entry: `launch/bringup_launch.py` (starts Gazebo, spawns robot, launches planner and follower).
- `src/robot_maze_planners`: Python planner implementations and nodes. Core planners:
  - `planners/astar_planner.py` (A* implementation)
  - `planners/rrt_planner.py` (RRT)
  - `planners/rrt_star_planner.py` (RRT*)
  Nodes live under `nodes/` (e.g., `path_follower_node.py`). These are installed as console scripts.
- `src/robot_maze_description`: robot URDF/xacro and meshes used by simulation.
- `src/*/tests` and `tools/`: unit tests and smoke scripts (see `tools/smoke_headless_test.sh`).

Important files to inspect when making changes
- `launch/bringup_launch.py` — wiring of simulation, planner selection (planner arg values: `astar`, `rrt`, `rrt_star`), RViz and Gazebo GUI flags.
- `planners/*_planner.py` — implement `plan()` semantics that return world-space list of (x,y) points. Follow A* shape/params.
- `utils/maze_helpers.py` — MazeGrid, world↔cell math, and `parse_maze` used to rasterize `maze_occupancy` Path messages.
- `config/maze_nav_demo.rviz` — RViz layout and topics used (e.g. `/maze_occupancy`, `/planned_path`, `/robot_pose`).

Data flows and topics (quick reference)
- Maze input: `maze_occupancy` (nav_msgs/Path) — wall segments as pose pairs; parsed by `maze_helpers.parse_maze` into `MazeGrid`.
- Robot state: `/model/slambot/odom` (Gazebo odom); optional `robot_pose` (PoseStamped) is supported.
- Planner output: `planned_path` (nav_msgs/Path) — world-frame polyline of the plan.
- Controller input/output: `planned_path` -> `path_follower_node` -> publishes `/model/slambot/cmd_vel` and `cmd_vel`.

Build / test / run workflows (exact commands)
- Build the workspace (recommended):
  - cd to repo root and run: `colcon build --symlink-install`
  - Source: `source install/setup.bash`
- Quick build for sim+planners (used in VS Code task):
  - `colcon build --packages-select robot_maze_simulation robot_maze_planners --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -Wno-dev`
- Launch bringup (A* + GUI):
  - `ros2 launch robot_maze_simulation bringup_launch.py planner:=astar headless:=false`
- Headless launch (CI / smoke test):
  - `ros2 launch robot_maze_simulation bringup_launch.py planner:=astar headless:=true`
- Run smoke script (quick verification): `tools/smoke_headless_test.sh` (inspects topics and nodes briefly).

Project-specific conventions and patterns
- Grid math: `MazeGrid` centers the maze around world origin. Use `origin_x = - (cols * cell_size)/2` — important when converting world↔cell.
- `maze_occupancy` encoding: walls are provided as Path with paired poses (start,end). See `generate_maze_sdf.py` for generation and expected format.
- Planner interface: planner classes should expose a `plan()` method and accept `MazeGrid`, start/goal in world coords. Return a list of (x,y) or a `nav_msgs/Path`-compatible polyline. See `astar_planner.py` for exact behavior:
  - Inflates obstacles via `inflation_radius` before searching.
  - If goal cell occupied, planner searches for nearest free cell.
- Node parameterization: Planner nodes declare ROS2 params (goal_x/y, step_size, cell_size, grid_rows/cols, plan_on_timer). Use `ros2 param list /maze_planner` to inspect at runtime.

Integration and external dependencies
- ROS 2 Jazzy (desktop) and Gazebo Harmonic are required. Also `ros_gz_*` bridge packages for Gazebo↔ROS topics.
- Optional: Nav2 `map_server` and lifecycle manager; the launch has flags to enable optional map server wiring.

Editing and safe changes guidance
- When changing planner logic, add unit tests under `src/robot_maze_planners/test/` — existing tests check world↔cell conversions and inflation behavior.
- For simulation/world edits, regenerate SDF with `generate_maze_sdf.py` and test in headless mode before enabling GUI.
- Preserve topic names and frames: RViz and nodes expect `map` frame and static `map->odom` identity transform in many places.

Examples to look at (concrete code pointers)
- A* planner: `src/robot_maze_planners/robot_maze_planners/planners/astar_planner.py`
- Launch wiring: `src/robot_maze_simulation/robot_maze_simulation/launch/bringup_launch.py`
- Maze generator: `src/robot_maze_simulation/robot_maze_simulation/tools/generate_maze_sdf.py`
- RViz config: `src/robot_maze_simulation/config/maze_nav_demo.rviz`

Troubleshooting hints for agents
- If no `planned_path` appears: verify `maze_occupancy` is published and `goal_x/goal_y` are inside grid bounds.
- If planners report unreachable goals: try increasing `inflation_radius` or check that `maze_occupancy` parsing returns expected free/occupied grid cells.
- GUI issues: use `headless:=true` or `force_software_rendering:=true` when launching.

What not to change without extra checks
- Topic names (`/model/slambot/odom`, `/planned_path`, `/maze_occupancy`) and TF frames (`map`) — changing these requires updates in RViz config and multiple nodes.
- `MazeGrid` coordinate math — many planners rely on that exact conversion; update both planner tests and nodes if modifying.

If you need more context
- Read the top-level `README.md` for high-level diagrams and exact parameter names.
- Use `ros2 param list /maze_planner` and `ros2 topic echo -n 1 /maze_occupancy` during runtime to confirm behavior.

If this file should be merged with an existing agent instruction, provide that file path and the preferred sections to preserve.

---
If you'd like, I can iterate on wording or add short command snippets for common edits (e.g., running a single planner node locally). Please tell me which areas to expand.
