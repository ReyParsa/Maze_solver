import math
import numpy as np
import pytest


# Try to import MazeGrid; if ROS msg types are unavailable, skip these tests gracefully
try:
    from robot_maze_planners.utils.maze_helpers import MazeGrid
except Exception:  # pragma: no cover - environment without ROS Python overlays
    pytest.skip("ROS msg types unavailable for MazeGrid import; skipping grid tests.", allow_module_level=True)

from robot_maze_planners.planners.astar_planner import AStarPlanner


def make_centered_grid(rows: int, cols: int, cell_size: float) -> MazeGrid:
    origin_x = - (cols * cell_size) / 2.0
    origin_y = - (rows * cell_size) / 2.0
    data = np.zeros((rows, cols), dtype=np.uint8)
    return MazeGrid(data, origin_x, origin_y, cell_size)


def test_world_cell_conversions_centered_origin():
    rows, cols, cell_size = 10, 10, 0.4
    grid = make_centered_grid(rows, cols, cell_size)
    planner = AStarPlanner(grid, start_world=(0.0, 0.0), goal_world=(0.0, 0.0), cell_size=cell_size)

    # Lower-left near corner should map to (0,0)
    ll_world = (grid.origin_x + 0.1, grid.origin_y + 0.1)
    cell_ll = planner._world_to_cell(ll_world)
    assert cell_ll == (0, 0)

    # Upper-right near corner should map to (cols-1, rows-1)
    ur_world = (
        -grid.origin_x - 0.1,  # origin_x is negative; upper-right x ~ +extent - epsilon
        -grid.origin_y - 0.1,
    )
    cell_ur = planner._world_to_cell(ur_world)
    assert cell_ur == (cols - 1, rows - 1)

    # Round-trip: (0,0) cell center world coords
    w00 = planner._cell_to_world((0, 0))
    assert math.isclose(w00[0], grid.origin_x + 0.5 * cell_size, rel_tol=0, abs_tol=1e-9)
    assert math.isclose(w00[1], grid.origin_y + 0.5 * cell_size, rel_tol=0, abs_tol=1e-9)

    # Round-trip: (cols-1, rows-1) cell center world coords
    w_ur = planner._cell_to_world((cols - 1, rows - 1))
    assert math.isclose(w_ur[0], grid.origin_x + (cols - 0.5) * cell_size, rel_tol=0, abs_tol=1e-9)
    assert math.isclose(w_ur[1], grid.origin_y + (rows - 0.5) * cell_size, rel_tol=0, abs_tol=1e-9)


def test_inflation_disk_radius_one_cell_cross():
    # Build a 7x7 grid with a single occupied cell in the center
    rows, cols, cell_size = 7, 7, 1.0
    grid = make_centered_grid(rows, cols, cell_size)
    cy, cx = rows // 2, cols // 2
    grid.data[cy, cx] = 1

    # Inflation radius = 1.0 m => r_cells = ceil(1.0/1.0) = 1
    # The implemented disk (<= r^2) includes 4-neighbors (cross), excludes diagonals for r=1
    planner = AStarPlanner(grid, start_world=(0.0, 0.0), goal_world=(0.0, 0.0), cell_size=cell_size, inflation_radius=1.0)
    occ = planner._occ

    assert occ[cy, cx] == 1  # center stays occupied
    # Cardinal neighbors are inflated
    assert occ[cy - 1, cx] == 1
    assert occ[cy + 1, cx] == 1
    assert occ[cy, cx - 1] == 1
    assert occ[cy, cx + 1] == 1
    # Diagonals at r=1 should remain 0 with the disk formulation
    assert occ[cy - 1, cx - 1] == 0
    assert occ[cy - 1, cx + 1] == 0
    assert occ[cy + 1, cx - 1] == 0
    assert occ[cy + 1, cx + 1] == 0
