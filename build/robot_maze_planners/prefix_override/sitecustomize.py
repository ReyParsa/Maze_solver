import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rey/Desktop/Maze Solver/robot_maze_ws/install/robot_maze_planners'
