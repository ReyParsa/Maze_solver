import logging
import math
import random
from typing import List, Tuple, Optional

import numpy as np

from robot_maze_planners.utils.maze_helpers import MazeGrid


class RRTPlanner:
    """Basic RRT planner in continuous space using MazeGrid for collision checks.

    Coordinates are world meters (map frame). The occupancy grid is treated as solid
    obstacles at cell centers; edges are collision-checked by sampling along the line.
    """

    def __init__(self, grid: MazeGrid, start: Tuple[float, float], goal: Tuple[float, float], step_size: float = 0.25, max_iter: int = 800, goal_bias: float = 0.08, goal_thresh: float = 0.35):
        self.grid = grid
        self.start = tuple(start)
        self.goal = tuple(goal)
        self.step_size = float(step_size)
        self.max_iter = int(max_iter)
        self.goal_bias = float(goal_bias)
        self.goal_thresh = float(goal_thresh)
        logging.info(
            f"RRTPlanner init start={self.start} goal={self.goal} step={self.step_size} iters={self.max_iter} bias={self.goal_bias} thr={self.goal_thresh}"
        )

    # ----------------- utilities -----------------
    def _rand_free_point(self) -> Tuple[float, float]:
        # Sample in maze bounding box with a bit of margin
        w = self.grid.shape[1] * self.grid.cell_size
        h = self.grid.shape[0] * self.grid.cell_size
        x = self.grid.origin_x + random.random() * w
        y = self.grid.origin_y + random.random() * h
        return (x, y)

    def _nearest(self, pts: List[Tuple[float, float]], q: Tuple[float, float]) -> int:
        bx = -1
        bd = 1e18
        qx, qy = q
        for i, (x, y) in enumerate(pts):
            d = (x - qx) * (x - qx) + (y - qy) * (y - qy)
            if d < bd:
                bd = d
                bx = i
        return bx

    def _steer(self, a: Tuple[float, float], b: Tuple[float, float], step: float) -> Tuple[float, float]:
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        dist = math.hypot(dx, dy)
        if dist <= step:
            return (bx, by)
        t = step / max(1e-9, dist)
        return (ax + dx * t, ay + dy * t)

    def _collision_free_segment(self, a: Tuple[float, float], b: Tuple[float, float]) -> bool:
        # sample along [a,b] at ~0.3*cell_size resolution
        step = max(0.1, self.grid.cell_size * 0.3)
        ax, ay = a
        bx, by = b
        seg_len = math.hypot(bx - ax, by - ay)
        steps = int(max(1, math.ceil(seg_len / step)))
        for s in range(steps + 1):
            t = s / max(1, steps)
            x = ax + (bx - ax) * t
            y = ay + (by - ay) * t
            if self._is_occupied(x, y):
                return False
        return True

    def _is_occupied(self, x: float, y: float) -> bool:
        # convert world to cell index and test occupancy
        cx = int(math.floor((x - self.grid.origin_x) / self.grid.cell_size + 1e-6))
        cy = int(math.floor((y - self.grid.origin_y) / self.grid.cell_size + 1e-6))
        if cy < 0 or cy >= self.grid.shape[0] or cx < 0 or cx >= self.grid.shape[1]:
            return True  # outside map treated as obstacle
        return self.grid.data[cy, cx] != 0

    # ----------------- main API -----------------
    def plan(self) -> List[Tuple[float, float]]:
        # Tree as parallel arrays for speed
        nodes: List[Tuple[float, float]] = [self.start]
        parents: List[int] = [-1]

        # quick reject if start in obstacle: clear locally to allow startup
        if self._is_occupied(*self.start):
            logging.warning("RRT: start is inside obstacle; temporarily clearing for planning run.")
        if self._is_occupied(*self.goal):
            logging.info("RRT: goal inside obstacle; will stop at threshold near it.")

        for it in range(self.max_iter):
            # biased sampling
            if random.random() < self.goal_bias:
                q_rand = self.goal
            else:
                q_rand = self._rand_free_point()
            # nearest
            idx = self._nearest(nodes, q_rand)
            q_near = nodes[idx]
            q_new = self._steer(q_near, q_rand, self.step_size)
            # collision check
            if not self._collision_free_segment(q_near, q_new):
                continue
            nodes.append(q_new)
            parents.append(idx)
            # goal reached?
            if math.hypot(q_new[0] - self.goal[0], q_new[1] - self.goal[1]) <= self.goal_thresh:
                logging.info(f"RRT reached goal vicinity in {it+1} iterations with {len(nodes)} nodes")
                return self._reconstruct(nodes, parents, len(nodes) - 1)

        logging.warning("RRT: goal not reached within iteration budget; returning best-effort path.")
        # best-effort: nearest-to-goal node
        best_i = self._nearest(nodes, self.goal)
        return self._reconstruct(nodes, parents, best_i)

    def _reconstruct(self, nodes: List[Tuple[float, float]], parents: List[int], leaf_index: int) -> List[Tuple[float, float]]:
        path = []
        i = leaf_index
        while i >= 0:
            path.append(nodes[i])
            i = parents[i]
        path.reverse()
        # optional: densify along segments to help follower
        out = [path[0]]
        max_step = max(0.1, self.grid.cell_size * 0.5)
        for i in range(1, len(path)):
            a = out[-1]
            b = path[i]
            seg_len = math.hypot(b[0] - a[0], b[1] - a[1])
            steps = int(max(1, math.floor(seg_len / max_step)))
            for s in range(1, steps + 1):
                t = min(1.0, s * max_step / max(1e-6, seg_len))
                out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
        return out
