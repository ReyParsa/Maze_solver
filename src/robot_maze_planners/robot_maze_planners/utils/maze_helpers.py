import numpy as np

def parse_maze(msg):
    """
    Parse maze occupancy data from a ROS message.
    Returns a 2D numpy array representing the maze grid.
    """
    # Placeholder: actual parsing depends on message type
    # For demonstration, return a simple 5x5 grid
    grid = np.zeros((5, 5))
    # Example: set some obstacles
    grid[1, 2] = 1
    grid[2, 2] = 1
    grid[3, 1] = 1
    return grid
