# Robot Maze Solver - ROS 2 Jazzy Project

A professional-level ROS 2 workspace implementing maze navigation using three distinct path-planning algorithms: A*, RRT, and RRT* in Gazebo simulation.

## Project Structure

```
robot_maze_ws/
└── src/
    ├── robot_maze_description/     # Robot URDF and environment models
    ├── robot_maze_simulation/      # Gazebo worlds and simulation launch files
    └── robot_maze_planners/        # Path planning algorithms and ROS 2 nodes
```

## Quick Start

1. **Build the workspace:**
   ```bash
   cd robot_maze_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

2. **Launch simulation with A* planner:**
   ```bash
   ros2 launch robot_maze_simulation bringup_launch.py planner:=astar
   ```

3. **Launch simulation with RRT planner:**
   ```bash
   ros2 launch robot_maze_simulation bringup_launch.py planner:=rrt
   ```

4. **Launch simulation with RRT* planner:**
   ```bash
   ros2 launch robot_maze_simulation bringup_launch.py planner:=rrt_star
   ```

## Available Planners

- **A* (astar)**: Optimal grid-based pathfinding
- **RRT (rrt)**: Rapidly-exploring Random Tree
- **RRT* (rrt_star)**: Optimal version of RRT

## Visualization

Launch RViz2 to visualize the planned paths:
```bash
rviz2 -d src/robot_maze_simulation/config/maze_visualization.rviz
```

## Dependencies

- ROS 2 Jazzy
- Gazebo Harmonic
- Nav2 stack
- Python 3.10+

## Author

Built following ROS 2 best practices and REP-2004 quality guidelines.

---

## Static Maze & Dual Visualization (Gazebo GUI + RViz2)

To make demos stable and reproducible, the launch system now supports a pre-generated static maze world plus simultaneous RViz2 visualization.

### Key Launch Arguments
| Argument | Default | Purpose |
|----------|---------|---------|
| `use_static_world` | `true` | Skip on-launch maze generation and use a static SDF. |
| `static_world_path` | `worlds/maze_static.sdf` | Path to the static maze world. |
| `with_rviz` | `true` | Start RViz2 alongside Gazebo. |
| `force_software_rendering` | `true` | Force llvmpipe (helpful in VMs without GPU). |

### Quick Demo (Static World + GUI + RViz)
```bash
source install/setup.bash
ros2 launch robot_maze_simulation bringup_launch.py \
   use_static_world:=true \
   static_world_path:=$(pwd)/src/robot_maze_simulation/worlds/maze_static.sdf \
   with_rviz:=true headless:=false force_software_rendering:=true
```

### Generate a New Maze Manually
```bash
python3 src/robot_maze_simulation/tools/generate_maze_sdf.py \
   --rows 10 --cols 10 --cell_size 0.4 \
   --seed 12345 \
   --output src/robot_maze_simulation/worlds/maze_seed12345.sdf --compact

ros2 launch robot_maze_simulation bringup_launch.py use_static_world:=true \
   static_world_path:=$(pwd)/src/robot_maze_simulation/worlds/maze_seed12345.sdf
```
Use `--single-mesh` for an STL visual (keeps primitive collisions for physics).

### RViz Layout
The included `maze_demo.rviz` displays:
* Grid (frame: map)
* Robot model
* `/planned_path` (green)
* `/maze_occupancy` (occupancy grid)
* `/robot_pose` (arrow)

If TF only provides `odom`, switch the Fixed Frame inside RViz from `map` to `odom`.

### Tips
* Keep `force_software_rendering:=true` inside virtualization environments.
* Static world avoids per-launch Python maze generation latency and instability.
* Record important topics for evidence: `/planned_path`, `/robot_pose`, `/odom`, `/cmd_vel`.

---


commands
# 0) From workspace root
cd "/home/rey/Desktop/Maze Solver/robot_maze_ws"

# 1) Build (ignore build/install/log if stale)
colcon build --symlink-install

# 2) Source overlay
source install/setup.bash

# 3) Launch (forces regen of maze, A* planner)
ros2 launch robot_maze_simulation bringup_launch.py planner:=astar headless:=false maze_force_regen:=true

# (Optional headless)
# ros2 launch robot_maze_simulation bringup_launch.py planner:=astar headless:=true

# 4) Inspect running nodes
ros2 node list

# 5) Check planner parameters
ros2 param list /maze_planner
ros2 param get /maze_planner goal_x
ros2 param get /maze_planner goal_y
ros2 param get /maze_planner allow_start_default

# 6) Verify robot pose (may need a few seconds after launch)
ros2 topic echo -n 1 /robot_pose

# 7) Verify odom bridge
ros2 topic echo -n 1 /model/slambot/odom

# 8) Inspect maze occupancy (wall segments as paired poses)
ros2 topic echo -n 1 /maze_occupancy

# 9) Check planned path
ros2 topic echo -n 1 /planned_path

# 10) See velocity commands sent to Gazebo model
ros2 topic echo -n 5 /model/slambot/cmd_vel

# 11) List topic publishers/subscribers (ensure connections)
ros2 topic info /planned_path
ros2 topic info /maze_occupancy
ros2 topic info /robot_pose

# 12) TF frames (if needed)
ros2 run tf2_tools view_frames
# (Generates frames.pdf after a short delay)

# 13) RQT graph (optional GUI)
rqt_graph

# 14) Re-run only the planner (if you change planners) without full sim restart
ros2 run robot_maze_planners planner_astar_node
# or
# ros2 run robot_maze_planners planner_rrtnode
# ros2 run robot_maze_planners planner_rrtstarnode

# 15) Clean build artifacts (only if necessary)
rm -rf build/ install/ log/ && colcon build --symlink-install