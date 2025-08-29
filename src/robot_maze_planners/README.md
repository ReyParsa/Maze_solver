# robot_maze_planners

This package implements three path-planning algorithms for maze solving:
- A* (astar)
- RRT (rrt)
- RRT* (rrt_star)

## Contents
- `planners/`: Modular Python classes for each planner
- `nodes/`: ROS 2 nodes for each planner
- `utils/maze_helpers.py`: Helper functions for maze parsing
- `launch/planners_launch.py`: Launch file to select planner

## Build

From the workspace root:
```bash
colcon build --symlink-install
source install/setup.bash
```

## Run Planners

Launch a planner node:
```bash
ros2 launch robot_maze_planners planners_launch.py planner:=astar
ros2 launch robot_maze_planners planners_launch.py planner:=rrt
ros2 launch robot_maze_planners planners_launch.py planner:=rrt_star
```

## Parameters
- `resolution` (A*)
- `step_size`, `max_iter` (RRT)
- `step_size`, `max_iter`, `radius` (RRT*)

## License
Apache-2.0
