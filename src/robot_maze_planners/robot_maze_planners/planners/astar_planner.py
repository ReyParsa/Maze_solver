import logging
import heapq
from typing import List, Tuple, Optional


class AStarPlanner:
    """Grid-based A* path planner.

        Coordinates:
            - World coordinates (meters) provided for start & goal.
            - Grid is a 2D array (H,W) with 0 = free, >0 = obstacle.
            - Maze / grid is assumed CENTERED at the world origin (0,0) such that the
                lower-left corner (cell 0,0) has world coords:
                        origin_x = - (W * cell_size) / 2
                        origin_y = - (H * cell_size) / 2
            - Conversions account for this origin; previously negative world coords were
                clamped to 0, collapsing large parts of the maze. This fix keeps full extent.
    """

    def __init__(self, grid, start_world: Tuple[float, float], goal_world: Tuple[float, float], *, cell_size: float = 0.4, allow_diagonal: bool = False):
        self.grid = grid
        self.start_world = start_world
        self.goal_world = goal_world
        self.cell_size = cell_size
        self.allow_diagonal = allow_diagonal
        # Pre-compute origin (lower-left) based on centered maze assumption
        if grid is not None:
            # MazeGrid wrapper expected
            if hasattr(grid, 'origin_x') and hasattr(grid, 'origin_y'):
                self.origin_x = grid.origin_x
                self.origin_y = grid.origin_y
            else:  # fallback to centered assumption
                h, w = grid.shape
                self.origin_x = - (w * cell_size) / 2.0
                self.origin_y = - (h * cell_size) / 2.0
        else:
            self.origin_x = 0.0
            self.origin_y = 0.0
        logging.info(
            f"AStarPlanner init start={start_world} goal={goal_world} cell_size={cell_size} grid_shape={getattr(grid,'shape',None)} origin=({getattr(self,'origin_x',0):.2f},{getattr(self,'origin_y',0):.2f})"
        )

    # ---------------- internal helpers ----------------
    def _world_to_cell(self, pt: Tuple[float, float]) -> Tuple[int, int]:
        x_rel = (pt[0] - self.origin_x) / self.cell_size
        y_rel = (pt[1] - self.origin_y) / self.cell_size
        return (int(x_rel // 1), int(y_rel // 1))

    def _cell_to_world(self, cell: Tuple[int, int]) -> Tuple[float, float]:
        # Return CENTER of cell for smoother following
        return (
            self.origin_x + (cell[0] + 0.5) * self.cell_size,
            self.origin_y + (cell[1] + 0.5) * self.cell_size,
        )

    def _neighbors(self, cell: Tuple[int, int]):
        (x, y) = cell
        dirs4 = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        dirs8 = dirs4 + [(1, 1), (1, -1), (-1, 1), (-1, -1)]
        for dx, dy in (dirs8 if self.allow_diagonal else dirs4):
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.grid.shape[1] and 0 <= ny < self.grid.shape[0]:
                if self.grid[ny, nx] == 0:  # free
                    yield (nx, ny)

    def _heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        # Manhattan (works fine even if diagonals allowed)
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def _reconstruct(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    # ---------------- public API ----------------
    def plan(self) -> List[Tuple[float, float]]:
        if self.grid is None:
            logging.warning("No grid provided to AStarPlanner; returning direct path.")
            return [self.start_world, self.goal_world]

        start_cell = self._world_to_cell(self.start_world)
        goal_cell = self._world_to_cell(self.goal_world)
        # Clamp inside grid (after conversion respecting origin)
        max_x = self.grid.shape[1] - 1
        max_y = self.grid.shape[0] - 1
        start_cell = (max(0, min(start_cell[0], max_x)), max(0, min(start_cell[1], max_y)))
        goal_cell = (max(0, min(goal_cell[0], max_x)), max(0, min(goal_cell[1], max_y)))
        logging.info(
            f"A* converted start_world={self.start_world} -> cell={start_cell}; goal_world={self.goal_world} -> cell={goal_cell}"
        )

        if self.grid[start_cell[1], start_cell[0]] != 0:
            logging.warning(f"Start cell {start_cell} is occupied; searching nearby free cell.")
            start_cell = self._find_nearest_free(start_cell)
        else:
            logging.info(f"Start cell {start_cell} is free.")
        if self.grid[goal_cell[1], goal_cell[0]] != 0:
            logging.warning(f"Goal cell {goal_cell} is occupied; searching nearby free cell.")
            goal_cell = self._find_nearest_free(goal_cell)
        else:
            logging.info(f"Goal cell {goal_cell} is free.")

        # Neighborhood diagnostic around start
        sx, sy = start_cell
        neigh_lines = []
        for ry in range(max(0, sy-1), min(self.grid.shape[0], sy+2)):
            row_vals = []
            for rx in range(max(0, sx-1), min(self.grid.shape[1], sx+2)):
                row_vals.append(str(int(self.grid[ry, rx])))
            neigh_lines.append(' '.join(row_vals))
        logging.debug(f"Start 3x3 occupancy (y rows top->bottom): {' | '.join(neigh_lines)}")
        # 5x5 diagnostic around start for deeper insight
        sxr = range(max(0, sy-2), min(self.grid.shape[0], sy+3))
        sxcs = range(max(0, sx-2), min(self.grid.shape[1], sx+3))
        diag_rows = []
        for ry in sxr:
            row_vals = []
            for rx in sxcs:
                row_vals.append(str(int(self.grid[ry, rx])))
            diag_rows.append(''.join(row_vals))
        logging.debug(f"Start 5x5 block (row order top->bottom): {' / '.join(diag_rows)}")

        frontier = []
        heapq.heappush(frontier, (0, start_cell))
        came_from = {}
        cost_so_far = {start_cell: 0}

        expanded = 0
        while frontier:
            _, current = heapq.heappop(frontier)
            expanded += 1
            if current == goal_cell:
                break
            for nb in self._neighbors(current):
                new_cost = cost_so_far[current] + 1  # uniform cost per move
                if nb not in cost_so_far or new_cost < cost_so_far[nb]:
                    cost_so_far[nb] = new_cost
                    priority = new_cost + self._heuristic(nb, goal_cell)
                    heapq.heappush(frontier, (priority, nb))
                    came_from[nb] = current

        if goal_cell not in cost_so_far:
            logging.warning("A* failed: goal unreachable; returning start->goal direct line.")
            return [self.start_world, self.goal_world]

        cells_path = self._reconstruct(came_from, goal_cell)
        world_path = [self._cell_to_world(c) for c in cells_path]
        logging.info(f"A* success nodes={len(cells_path)} expanded={expanded}")
        return world_path

    def _find_nearest_free(self, cell: Tuple[int, int]) -> Tuple[int, int]:
        if self.grid[cell[1], cell[0]] == 0:
            return cell
        # BFS ring search
        from collections import deque
        visited = set([cell])
        q = deque([cell])
        while q:
            x, y = q.popleft()
            for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                nx, ny = x+dx, y+dy
                if 0 <= nx < self.grid.shape[1] and 0 <= ny < self.grid.shape[0] and (nx, ny) not in visited:
                    if self.grid[ny, nx] == 0:
                        return (nx, ny)
                    visited.add((nx, ny))
                    q.append((nx, ny))
        return cell  # fallback
