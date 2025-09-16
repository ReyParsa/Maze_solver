# Robot Maze Solver (ROS 2 Jazzy + Gazebo Harmonic)

An end‑to‑end, learning‑friendly ROS 2 workspace that simulates a robot solving mazes using A*, RRT, and RRT*. This README doubles as a mini‑wiki: it explains ROS 2 concepts used here, the algorithms, how the system hangs together, and how you can build your own.

If you follow along, you’ll learn to:
- Build and run a ROS 2 multi‑package workspace with Gazebo Harmonic
- Understand nodes, topics, parameters, TF frames, and launch files by example
- Implement and compare A*, RRT, and RRT* planners on a grid maze
- Visualize and debug paths and robot motion in RViz2 and Gazebo

## Repository layout

```
robot_maze_ws/
├── src/
│   ├── robot_maze_description/   # Robot URDF (xacro) and meshes
│   ├── robot_maze_simulation/    # Gazebo worlds, launch, configs, tools
│   └── robot_maze_planners/      # Planner nodes (A*, RRT, RRT*), path follower, helpers
├── build/ install/ log/          # Colcon artifacts (generated)
└── README.md                     # You are here
```

Key files to know:
- Simulation bring‑up: `src/robot_maze_simulation/launch/bringup_launch.py`
- World assets: `src/robot_maze_simulation/worlds/*`
- Maze generator: `src/robot_maze_simulation/tools/generate_maze_sdf.py`
- Planner nodes: `src/robot_maze_planners/robot_maze_planners/nodes/*`
- Planners (core):
  - A*: `robot_maze_planners/planners/astar_planner.py` (full implementation)
  - RRT: `robot_maze_planners/planners/rrt_planner.py` (implemented)
  - RRT*: `robot_maze_planners/planners/rrt_star_planner.py` (implemented)
- Utilities: `robot_maze_planners/utils/maze_helpers.py` (MazeGrid, parsing)

## Prerequisites

- ROS 2 Jazzy (desktop)
- Gazebo Harmonic and ROS–Gazebo bridge (`ros_gz_*`)
- Python 3.10+
- Colcon build tools
- Optional: Nav2 packages for map server (`nav2_map_server`, `nav2_lifecycle_manager`) used if available

On Ubuntu, install ROS 2 Jazzy and Gazebo Harmonic per their official guides, then ensure your environment is sourced before building this workspace.

## Quick start

```bash
cd "/home/rey/Desktop/Maze Solver/robot_maze_ws"
colcon build --symlink-install
source install/setup.bash

# Launch with A* and GUI (RViz + Gazebo); VM‑friendly software GL is enabled by default
ros2 launch robot_maze_simulation bringup_launch.py planner:=astar headless:=false

# Headless (no GUI)
ros2 launch robot_maze_simulation bringup_launch.py planner:=astar headless:=true
```

Switch planners at launch:

```bash
ros2 launch robot_maze_simulation bringup_launch.py planner:=rrt
ros2 launch robot_maze_simulation bringup_launch.py planner:=rrt_star
```

## System at a glance (diagram)

```mermaid
graph TD
  subgraph Simulation
    GZ[Gazebo Harmonic\n(worlds/maze.world)]
    ODOM[/model/slambot/odom\n(nav_msgs/Odometry)]
    CMD[/model/slambot/cmd_vel\n(geometry_msgs/Twist)]
    RSP[robot_state_publisher]
    JSP[joint_state_publisher]
    TF[Static TF: map -> odom]
  end

  subgraph Planning
    PL[planner_*_node\n(A*/RRT/RRT*)\nParams: goal_x/y, step_size, etc.]
    PF[path_follower_node\nPure pursuit + recovery\nParams: gains, lookahead]
  end

  subgraph Map & Pose
    MAZE[maze_occupancy\n(nav_msgs/Path)]
    POSE[robot_pose\n(geometry_msgs/PoseStamped)\nOptional]
    MAP[map_server\nOptional Nav2]
  end

  subgraph Visualization
    RVIZ[RViz2\n(config/maze_nav_demo.rviz)]
  end

  GZ -->|Odometry| ODOM
  ODOM --> PL
  MAZE --> PL
  PL -->|planned_path| PF
  PF -->|cmd_vel| CMD
  CMD --> GZ
  RSP --> TF
  JSP --> TF
  TF --> RVIZ
  PL --> RVIZ
  PF --> RVIZ
  MAP --> RVIZ
```

## What runs under the hood

When you launch `bringup_launch.py`, the following components start:

- Gazebo server (and GUI if `headless:=false`), loading `worlds/maze.world`
- Robot spawn (`ros_gz_sim create`) using `robot_maze_description/urdf/.../slambot.urdf.xacro`
- `robot_state_publisher` + `joint_state_publisher`
- Static TF publisher: `map -> odom` identity
- Planner node (one of):
  - `planner_astar_node`, `planner_rrt_node`, or `planner_rrtstar_node` (launched as `maze_planner`)
- Path follower: `path_follower_node` (publishes velocity commands)
- Optional (if installed): `nav2_map_server` + lifecycle manager for static maps
- RViz2 with a preconfigured layout

### Topics (most relevant)

- Sensor/pose and control
  - `/model/slambot/odom` (nav_msgs/Odometry) — robot odometry from Gazebo
  - `robot_pose` (geometry_msgs/PoseStamped) — optional; planners also read odom directly
  - `/model/slambot/cmd_vel` (geometry_msgs/Twist) — velocity commands to robot
  - `cmd_vel` (geometry_msgs/Twist) — generic duplicate command topic

- Planning I/O
  - `maze_occupancy` (nav_msgs/Path) — wall segments encoded as paired poses
  - `planned_path` (nav_msgs/Path) — computed path in the `map` frame

Frames: RViz uses `map`. A static identity transform `map -> odom` is provided so you can view either.

### Launch arguments (bringup_launch.py)

- `planner`: astar | rrt | rrt_star (default: astar)
- `with_rviz`: Launch RViz2 (default: true)
- `headless`: Run without GUI (default: false)
- `force_software_rendering`: Force software GL for GUI (default: true)
- `maze_rows`, `maze_cols`: Maze size in cells (default: 10, 10)
- `cell_size`: Cell size (m) (default: 0.4)
- `maze_compact`: Use compact model output (default: true)
- `maze_seed`: Deterministic generator seed (default: empty/rand)
- `maze_force_regen`, `maze_single_mesh`: kept for compatibility; no‑ops here
- `minimal_gui`: Use minimal Gazebo GUI config (default: true)
- `inflation_radius`: Safety margin for A* occupancy (m) (default: 0.2)

Planner node parameters (subset):
- A* `planner_astar_node`:
  - `goal_x`, `goal_y` (computed from maze size by default)
  - `cell_size` (resolution), `inflation_radius`
  - `plan_on_timer` (bool), `plan_rate_hz`
  - `allow_start_default`, `default_start_x`, `default_start_y`
  - `grid_rows`, `grid_cols` (internal occupancy size)
- RRT / RRT* nodes:
  - `step_size`, `max_iter`, and for RRT* also `radius`

Inspect parameters at runtime:

```bash
ros2 param list /maze_planner
ros2 param get /maze_planner goal_x
ros2 param get /maze_planner goal_y
```

## How the maze and grid coordinate system work

The maze is centered at world origin (0, 0). If the grid has `rows` x `cols` cells of size `cell_size`:
- Lower‑left world coordinate of cell (0,0):
  - `origin_x = - (cols * cell_size) / 2`
  - `origin_y = - (rows * cell_size) / 2`
- `MazeGrid` stores: a binary occupancy array, `origin_x`, `origin_y`, and `cell_size` for consistent world↔cell conversions.

`maze_occupancy` topic carries wall segments as a Path with pairs of poses (start, end). `maze_helpers.parse_maze` rasterizes these into `MazeGrid`.

## Algorithms, explained

### A* (grid‑based) — implemented

File: `robot_maze_planners/planners/astar_planner.py`

Highlights:
- Converts world start/goal to cells using the centered grid origin
- Optional obstacle inflation (`inflation_radius`) to keep a clearance from walls
- 4‑connected neighbors by default (toggle to 8 if needed)
- Manhattan heuristic, uniform cost per move
- If goal cell is occupied, it searches the nearest free cell
- Returns a world‑space polyline through cell centers; then the node prunes collinear points and densifies for smooth following

Pseudocode:

```
function a_star(grid, start_world, goal_world):
  start = world_to_cell(start_world)
  goal  = world_to_cell(goal_world)
  frontier = priority_queue()
  frontier.push(start, 0)
  came_from = {}
  cost_so_far = {start: 0}
  while frontier not empty:
    current = frontier.pop_lowest()
    if current == goal: break
    for nb in neighbors(current):
      new_cost = cost_so_far[current] + 1
      if nb not in cost_so_far or new_cost < cost_so_far[nb]:
        cost_so_far[nb] = new_cost
        priority = new_cost + heuristic(nb, goal)
        frontier.push(nb, priority)
        came_from[nb] = current
  if goal not reached: return [start_world, goal_world]
  cells_path = backtrack(came_from, goal)
  return [cell_center(c) for c in cells_path]
```

Practical tuning:
- Increase `inflation_radius` to avoid skimming walls
- Use smaller `cell_size` for finer paths (with higher compute cost)

### RRT — implemented

File: `robot_maze_planners/planners/rrt_planner.py`

RRT grows a tree from the start by sampling random points and extending toward the nearest tree node, avoiding obstacles until it reaches near the goal. This repo’s implementation includes goal bias, step‑size steering, collision sampling against `MazeGrid`, and densified path output.

Pseudocode:

```
for i in range(max_iter):
  q_rand = sample_with_goal_bias(goal)
  q_near = nearest(tree, q_rand)
  q_new  = steer(q_near, q_rand, step_size)
  if collision_free(q_near, q_new, grid):
    add_to_tree(q_new, parent=q_near)
    if dist(q_new, goal) < goal_threshold: break
```

Use planner params: `step_size`, `max_iter`.

RRT flow:

```mermaid
flowchart LR
  A[Start node] --> B{Sample q_rand\n(goal bias p)}
  B --> C[Find nearest node q_near]
  C --> D[Steer toward q_rand by step_size\n-> q_new]
  D --> E{Collision-free\n(q_near -> q_new)?}
  E -- no --> B
  E -- yes --> F[Add q_new to tree\nparent = q_near]
  F --> G{dist(q_new, goal) < thresh?}
  G -- yes --> H[Reconstruct path]
  G -- no --> B
```

### RRT* — implemented

File: `robot_maze_planners/planners/rrt_star_planner.py`

RRT* improves RRT by rewiring to optimize path cost inside a neighborhood radius. This implementation selects the best parent among neighbors within `radius` and rewires neighbors when it lowers cost, while checking collisions against `MazeGrid`.

Params: `step_size`, `max_iter`, `radius`.

RRT* flow:

```mermaid
flowchart LR
  A[Start node] --> B{Sample q_rand\n(goal bias p)}
  B --> C[Find q_near]
  C --> D[Steer to q_new]
  D --> E{Collision-free?}
  E -- no --> B
  E -- yes --> F[Find neighbors within radius]
  F --> G[Choose best parent\n(min cost + edge free)]
  G --> H[Insert q_new with cost]
  H --> I[Rewire neighbors if cheaper\n(and edge free)]
  I --> J{dist(q_new, goal) < thresh?}
  J -- yes --> K[Reconstruct best path]
  J -- no --> B
```

## Tests and smoke checks

- Unit tests (pytest): `src/robot_maze_planners/test/`
  - `test_astar_utils.py` validates world↔cell conversions and inflation behavior
- Headless smoke test script: `tools/smoke_headless_test.sh`
  - Builds the sim + planners, launches headless A* briefly, and checks topics
  - Useful for quick verification or CI

### A* conversion flow (centered grid)

```mermaid
flowchart LR
  A[Start/Goal in world (x,y)] --> B[Compute origin_x, origin_y\nfrom rows, cols, cell_size]
  B --> C[world_to_cell: (x - origin_x)/cell_size]
  C --> D[A* search on 0/1 grid\n(with optional inflation)]
  D --> E[cell_to_world: center of cell]
  E --> F[planned_path]
```

## Path following

File: `robot_maze_planners/nodes/path_follower_node.py`

- Subscribes: `planned_path`, odom (`/model/slambot/odom` and `/odom` as fallback)
- Publishes: `/model/slambot/cmd_vel` and `cmd_vel`
- Uses look‑ahead pure‑pursuit‑style logic with yaw error limits and a recovery behavior if progress stalls
- Useful params: `linear_gain`, `angular_gain`, `lookahead_distance`, `goal_tolerance`, speed limits, and recovery knobs

Path follower logic:

```mermaid
flowchart TD
  A[Receive planned_path\n(nav_msgs/Path)] --> B[Receive odom\n(nav_msgs/Odometry)]
  B --> C[Extract current pose\n(x, y, yaw)]
  C --> D[Find lookahead point\non path ahead by distance]
  D --> E[Compute yaw error\nto lookahead point]
  E --> F[Compute angular velocity\n= angular_gain * yaw_error]
  F --> G[Compute linear velocity\n= min(max_linear_speed, linear_gain * dist_to_goal)]
  G --> H{Progress stalled?\n(no_progress_timeout)}
  H -- yes --> I[Recovery: rotate in place\nor reverse briefly]
  H -- no --> J[Publish cmd_vel\n(linear, angular)]
  I --> J
  J --> K{At goal?\n(goal_tolerance)}
  K -- no --> B
  K -- yes --> L[Stop robot\n(cmd_vel = 0)]
```

## Simulation and world generation

Worlds live in `src/robot_maze_simulation/worlds/` and are loaded by Gazebo. The default `maze.world` is expected to exist. You can generate new static worlds with the provided tool:

```bash
python3 src/robot_maze_simulation/tools/generate_maze_sdf.py \
  --rows 10 --cols 10 --cell_size 0.4 \
  --seed 12345 --compact \
  --output /tmp/maze_world_generated.sdf
```

Flags:
- `--compact`: single model with many links (fast to load)
- `--single-mesh`: export STL for visuals; collisions remain primitive boxes

Tip: The launch file also computes default spawn and goal at the lower‑left and upper‑right corners of the centered maze.

## RViz2 visualization

The launch will start RViz2 (unless `with_rviz:=false`) using `config/maze_nav_demo.rviz`. Displayed:
- Robot model
- `planned_path` (green polyline)
- TF frames (Fixed Frame `map`)

If you only see `odom`, you can switch RViz Fixed Frame to `odom`.

## Hands‑on: verify the system

```bash
# After launching bringup
ros2 node list
ros2 topic echo -n 1 /model/slambot/odom
ros2 topic echo -n 1 /maze_occupancy
ros2 topic echo -n 1 /planned_path
ros2 topic info /planned_path
ros2 param list /maze_planner
```

If you also run an odom→pose republisher (optional), verify:

```bash
ros2 topic echo -n 1 /robot_pose
```

## Troubleshooting

GUI doesn’t appear / crashes:
- Use headless mode: `headless:=true`
- Or force software GL (default when GUI is enabled): `force_software_rendering:=true`
- On VMs, enable 3D acceleration if possible

No odom / no robot_pose:
- Check the Gazebo bridge topics exist: `/model/slambot/odom`
- The planners can use odom directly; `robot_pose` is optional

No path published:
- Echo `maze_occupancy` to ensure it’s published
- Increase `inflation_radius` if the robot is too close to walls
- Confirm `goal_x`, `goal_y` are reachable (planner params)

Robot stuck / oscillating:
- Reduce `max_linear_speed`, increase `angular_gain`
- Increase `lookahead_distance` slightly for smoother pursuit
- The follower has a built‑in recovery; tune `no_progress_timeout` and `recovery_duration`

Clean rebuild:

```bash
rm -rf build/ install/ log/
colcon build --symlink-install
```

## Build your own planner in this project

1) Create a new class in `robot_maze_planners/planners/` with a `plan()` method that returns a list of `(x, y)` world points.

2) Add a ROS 2 node in `robot_maze_planners/nodes/` that:
- Subscribes to `robot_pose` and/or `maze_occupancy`
- Publishes `planned_path`
- Declares parameters you need (e.g., step size, iterations)

3) Wire it in `bringup_launch.py` by adding a `planner` option or a new executable target.

You can reuse `MazeGrid` and `parse_maze` to convert walls to an occupancy grid with consistent world↔cell math.

## Node catalog (what each node does)

- planner_astar_node
  - Inputs: `robot_pose` (optional), `/model/slambot/odom` (Odometry), `maze_occupancy` (Path)
  - Output: `planned_path` (Path)
  - Params: `goal_x`, `goal_y`, `cell_size`, `inflation_radius`, `plan_on_timer`, `plan_rate_hz`, `allow_start_default`, `default_start_x`, `default_start_y`, `grid_rows`, `grid_cols`

- planner_rrt_node / planner_rrtstar_node
  - Inputs/outputs same as A*; core planner params: `step_size`, `max_iter`, (RRT*: `radius`)

- path_follower_node
  - Inputs: `planned_path`, odom topics `/model/slambot/odom` and `/odom`
  - Outputs: `/model/slambot/cmd_vel`, `cmd_vel`
  - Params: controller gains, lookahead, speed limits, recovery behavior

- maze_publisher_node (optional)
  - Generates a maze procedurally at runtime and publishes wall segments on `maze_occupancy`
  - Useful for planner testing outside full simulation

- odom_to_pose_node (optional)
  - Converts Odometry to `PoseStamped` on `robot_pose` when needed by downstream nodes

## From-scratch: scaffold a similar ROS 2 project

These steps assume ROS 2 Jazzy is installed and sourced.

```bash
# 1) Create a workspace
mkdir -p ~/dev/maze_ws/src
cd ~/dev/maze_ws

# 2) Create packages
# Simulation (C++ or Python). Here: Python for planners, and a separate sim package for launch/worlds.
ros2 pkg create robot_maze_planners --build-type ament_python --dependencies rclpy nav_msgs geometry_msgs std_msgs tf2_ros
ros2 pkg create robot_maze_simulation --build-type ament_cmake
ros2 pkg create robot_maze_description --build-type ament_cmake

# 3) Add your code and launch files similar to this repo structure
#    - planners under robot_maze_planners/robot_maze_planners/
#    - launch under robot_maze_simulation/launch/
#    - URDF under robot_maze_description/urdf/

# 4) Build and source
colcon build --symlink-install
source install/setup.bash

# 5) Launch your bringup
ros2 launch robot_maze_simulation bringup_launch.py planner:=astar
```

Tips:
- For Python packages, ensure `setup.py`, `setup.cfg`, and `package.xml` declare entry points for console scripts (your nodes)
- Use `install(PROGRAMS ... DESTINATION lib/<pkg>)` for CMake packages, and export share directories for resources (worlds, configs)
- Start small: publish a dummy `planned_path` first, then integrate odom and maze occupancy

Launch sequence (bringup_launch.py):

```mermaid
flowchart TD
  A[Launch bringup_launch.py\nwith args: planner, headless, etc.] --> B[Set env vars\n(GZ_GUI_CONFIG, software GL)]
  B --> C[Start Gazebo server\n(and GUI if not headless)]
  C --> D[Spawn robot\n(ros_gz_sim create)]
  D --> E[Start robot_state_publisher\n+ joint_state_publisher]
  E --> F[Publish static TF\n(map -> odom identity)]
  F --> G[Start planner node\n(depending on planner arg)]
  G --> H[Start path_follower_node]
  H --> I[Optional: maze_publisher_node\n(if no static maze)]
  I --> J[Optional: nav2 map_server\n(if installed)]
  J --> K[Start ros_gz_bridge\n(for cmd_vel, odom)]
  K --> L[Start RViz2\n(if with_rviz)]
  L --> M[System ready\n(nodes publishing/subscribing)]
```