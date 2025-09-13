import logging
import heapq
from typing import List, Tuple, Optional


class AStarPlanner:
    """Grid-based A* path planner.

    Coordinates:
      - World coordinates (meters) provided for start & goal.
      - Grid is a 2D array (H,W) with 0 = free, >0 = obstacle.
      - cell_size maps grid indices to world: world_x = ix * cell_size.
    """

    def __init__(self, grid, start_world: Tuple[float, float], goal_world: Tuple[float, float], *, cell_size: float = 0.4, allow_diagonal: bool = False):
        self.grid = grid
        self.start_world = start_world
        self.goal_world = goal_world
        self.cell_size = cell_size
        self.allow_diagonal = allow_diagonal
        logging.info(
            f"AStarPlanner init start={start_world} goal={goal_world} cell_size={cell_size} grid_shape={getattr(grid,'shape',None)}"
        )

    # ---------------- internal helpers ----------------
    def _world_to_cell(self, pt: Tuple[float, float]) -> Tuple[int, int]:
        return (max(0, int(round(pt[0] / self.cell_size))), max(0, int(round(pt[1] / self.cell_size))))

    def _cell_to_world(self, cell: Tuple[int, int]) -> Tuple[float, float]:
        return (cell[0] * self.cell_size, cell[1] * self.cell_size)

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

        # Clamp inside grid
        max_x = self.grid.shape[1] - 1
        max_y = self.grid.shape[0] - 1
        start_cell = (min(start_cell[0], max_x), min(start_cell[1], max_y))
        goal_cell = (min(goal_cell[0], max_x), min(goal_cell[1], max_y))

        if self.grid[start_cell[1], start_cell[0]] != 0:
            logging.warning(f"Start cell {start_cell} is occupied; searching nearby free cell.")
            start_cell = self._find_nearest_free(start_cell)
        if self.grid[goal_cell[1], goal_cell[0]] != 0:
            logging.warning(f"Goal cell {goal_cell} is occupied; searching nearby free cell.")
            goal_cell = self._find_nearest_free(goal_cell)

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
