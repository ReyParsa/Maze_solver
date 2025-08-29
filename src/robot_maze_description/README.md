# robot_maze_description

This package contains the robot and maze environment description for the maze solver project.

## Contents
- `urdf/robot.urdf.xacro`: Differential drive robot definition
- `launch/description_launch.py`: Launch robot state publisher and joint state publisher
- `meshes/`: (Optional) Meshes for robot parts

## Usage

To launch the robot description:
```bash
ros2 launch robot_maze_description description_launch.py
```

## Build

From the workspace root:
```bash
colcon build --symlink-install
source install/setup.bash
```

## License
Apache-2.0
