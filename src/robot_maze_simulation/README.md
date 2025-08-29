# robot_maze_simulation

Gazebo simulation package for the robot maze solver project.

## Contents
- `worlds/maze_world.sdf`: Maze world definition for Gazebo
- `launch/sim_launch.py`: Launch Gazebo with maze and robot
- `launch/bringup_launch.py`: Launch simulation and planner node
- `config/gazebo_params.yaml`: Gazebo tuning parameters

## Build

From the workspace root:
```bash
colcon build --symlink-install
source install/setup.bash
```

## Run Simulation

Launch Gazebo with the maze world and robot:
```bash
ros2 launch robot_maze_simulation sim_launch.py
```

Launch simulation with a planner (A*, RRT, RRT*):
```bash
ros2 launch robot_maze_simulation bringup_launch.py planner:=astar
ros2 launch robot_maze_simulation bringup_launch.py planner:=rrt
ros2 launch robot_maze_simulation bringup_launch.py planner:=rrt_star
```

## License
Apache-2.0
