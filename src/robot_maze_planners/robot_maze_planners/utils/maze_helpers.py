import numpy as np

def parse_maze(msg, *, rows: int = 20, cols: int = 20):
    """Parse maze occupancy data.

    Currently still a stub but returns a larger zero grid with a few obstacle lines
    to allow A* to demonstrate multi-waypoint paths.
    0 = free, 1 = obstacle.
    """
    grid = np.zeros((rows, cols), dtype=np.uint8)
    # add some synthetic walls (ensure within bounds)
    if rows > 5 and cols > 5:
        grid[3, 2:cols-2] = 1
        grid[rows//2, 1:cols-1:2] = 1  # dashed wall
        grid[rows-4:rows-2, cols//3] = 1
    return grid
