import numpy as np
from nav_msgs.msg import Path

class MazeGrid:
    """Wrapper for occupancy grid holding origin and cell size metadata.

    Attributes:
      data: 2D numpy array (rows x cols) with 0=free, 1=occupied
      origin_x, origin_y: world coords of lower-left corner of cell (0,0)
      cell_size: resolution (meters)
    """
    __slots__ = ("data", "origin_x", "origin_y", "cell_size")
    def __init__(self, data: np.ndarray, origin_x: float, origin_y: float, cell_size: float):
        self.data = data.astype(np.uint8)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.cell_size = float(cell_size)
    @property
    def shape(self):
        return self.data.shape
    def __getitem__(self, key):
        return self.data[key]

def parse_maze(msg: Path, *, rows: int = 20, cols: int = 20, cell_size: float = 0.4) -> MazeGrid:
    """Convert maze wall segments (Path) to MazeGrid using CENTERED origin.

    Centered origin: maze spans (cols*cell_size, rows*cell_size) and is centered at (0,0).
    Lower-left (cell 0,0) world coordinates:
        origin_x = - (cols * cell_size)/2
        origin_y = - (rows * cell_size)/2

    If msg invalid, returns empty grid with requested rows/cols.
    """
    origin_x = - (cols * cell_size) / 2.0
    origin_y = - (rows * cell_size) / 2.0
    grid = np.zeros((rows, cols), dtype=np.uint8)

    if msg is None or not isinstance(msg, Path) or len(msg.poses) < 2:
        return MazeGrid(grid, origin_x, origin_y, cell_size)

    def world_to_index(x: float, y: float):
        cx = (x - origin_x) / cell_size
        cy = (y - origin_y) / cell_size
        return int(np.floor(cy + 1e-6)), int(np.floor(cx + 1e-6))  # row, col

    def clamp_rc(r, c):
        return 0 <= r < rows and 0 <= c < cols

    poses = msg.poses
    for i in range(0, len(poses) - 1, 2):
        p1 = poses[i].pose.position
        p2 = poses[i + 1].pose.position
        x1, y1 = p1.x, p1.y
        x2, y2 = p2.x, p2.y
        seg_len = max(1e-6, np.hypot(x2 - x1, y2 - y1))
        step = cell_size * 0.4
        steps = int(np.ceil(seg_len / step))
        for s in range(steps + 1):
            t = s / max(1, steps)
            xs = x1 + (x2 - x1) * t
            ys = y1 + (y2 - y1) * t
            r, c = world_to_index(xs, ys)
            if clamp_rc(r, c):
                grid[r, c] = 1
        for (wx, wy) in [(x1, y1), (x2, y2)]:
            r, c = world_to_index(wx, wy)
            if clamp_rc(r, c):
                grid[r, c] = 1

    return MazeGrid(grid, origin_x, origin_y, cell_size)
