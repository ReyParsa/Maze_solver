import numpy as np
from nav_msgs.msg import Path

def parse_maze(msg: Path, *, rows: int = 20, cols: int = 20, cell_size: float = 0.4):
    """Convert a Path of segment endpoints into an occupancy grid.

    Encoding expected (published by maze_publisher_node): each *pair* of consecutive poses
    represents the start and end endpoints of a maze wall segment in the 'map' frame.

    We rasterize each wall segment onto a discrete grid of size (rows x cols) centered
    at the origin (matching the maze generation frame). A cell is marked occupied (1)
    if a segment passes through or sufficiently close to the cell center or its boundary.

    Fallback: if msg is None or malformed, return an empty (all-free) grid so planner can still run.
    """
    grid = np.zeros((rows, cols), dtype=np.uint8)
    if msg is None or not isinstance(msg, Path) or len(msg.poses) < 2:
        return grid

    # derive grid origin so indices map: cell (0,0) at lower-left of maze spanning rows*cell_size
    width = cols * cell_size
    height = rows * cell_size
    origin_x = -width / 2.0
    origin_y = -height / 2.0

    def world_to_index(x: float, y: float):
        # translate to origin, then divide
        cx = (x - origin_x) / cell_size
        cy = (y - origin_y) / cell_size
        return int(np.floor(cy)), int(np.floor(cx))  # row (y), col (x)

    def clamp_rc(r, c):
        return 0 <= r < rows and 0 <= c < cols

    # Iterate over pairs of poses
    poses = msg.poses
    for i in range(0, len(poses) - 1, 2):
        p1 = poses[i].pose.position
        p2 = poses[i + 1].pose.position
        x1, y1 = p1.x, p1.y
        x2, y2 = p2.x, p2.y

        # Bresenham-like sampling along the segment in world coords, step ~ half cell
        seg_len = max(1e-6, np.hypot(x2 - x1, y2 - y1))
        step = cell_size * 0.4  # finer than cell for coverage
        steps = int(np.ceil(seg_len / step))
        for s in range(steps + 1):
            t = s / max(1, steps)
            xs = x1 + (x2 - x1) * t
            ys = y1 + (y2 - y1) * t
            r, c = world_to_index(xs, ys)
            if clamp_rc(r, c):
                grid[r, c] = 1

        # Also mark bounding box corners to avoid gaps
        for (wx, wy) in [(x1, y1), (x2, y2)]:
            r, c = world_to_index(wx, wy)
            if clamp_rc(r, c):
                grid[r, c] = 1

    return grid
