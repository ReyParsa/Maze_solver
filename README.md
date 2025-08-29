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
