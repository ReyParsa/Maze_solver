import logging
import math
import random
from typing import List, Tuple

import numpy as np

from robot_maze_planners.utils.maze_helpers import MazeGrid


class RRTStarPlanner:
    """RRT* planner using MazeGrid for collisions, with simple radius-based rewiring."""

    def __init__(self, grid: MazeGrid, start: Tuple[float, float], goal: Tuple[float, float], step_size: float = 0.25, max_iter: int = 1000, radius: float = 0.7, goal_bias: float = 0.08, goal_thresh: float = 0.35):
        self.grid = grid
        self.start = tuple(start)
        self.goal = tuple(goal)
        self.step_size = float(step_size)
        self.max_iter = int(max_iter)
        self.radius = float(radius)
        self.goal_bias = float(goal_bias)
        self.goal_thresh = float(goal_thresh)
        logging.info(
            f"RRT* init start={self.start} goal={self.goal} step={self.step_size} iters={self.max_iter} r={self.radius} bias={self.goal_bias} thr={self.goal_thresh}"
        )

    # ---------- utilities shared with RRT ----------
    def _rand_free_point(self) -> Tuple[float, float]:
        w = self.grid.shape[1] * self.grid.cell_size
        h = self.grid.shape[0] * self.grid.cell_size
        x = self.grid.origin_x + random.random() * w
        y = self.grid.origin_y + random.random() * h
        return (x, y)

    def _is_occupied(self, x: float, y: float) -> bool:
        cx = int(math.floor((x - self.grid.origin_x) / self.grid.cell_size + 1e-6))
        cy = int(math.floor((y - self.grid.origin_y) / self.grid.cell_size + 1e-6))
        if cy < 0 or cy >= self.grid.shape[0] or cx < 0 or cx >= self.grid.shape[1]:
            return True
        return self.grid.data[cy, cx] != 0

    def _collision_free_segment(self, a: Tuple[float, float], b: Tuple[float, float]) -> bool:
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

    def _steer(self, a: Tuple[float, float], b: Tuple[float, float], step: float) -> Tuple[float, float]:
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        dist = math.hypot(dx, dy)
        if dist <= step:
            return (bx, by)
        t = step / max(1e-9, dist)
        return (ax + dx * t, ay + dy * t)

    # ---------- main ----------
    def plan(self) -> List[Tuple[float, float]]:
        nodes: List[Tuple[float, float]] = [self.start]
        parents: List[int] = [-1]
        costs: List[float] = [0.0]  # cumulative cost from start

        if self._is_occupied(*self.start):
            logging.warning("RRT*: start in obstacle; temporarily allowing for this plan.")

        goal_index = None

        for it in range(self.max_iter):
            q_rand = self.goal if random.random() < self.goal_bias else self._rand_free_point()
            # nearest
            idx_near = self._nearest(nodes, q_rand)
            q_new = self._steer(nodes[idx_near], q_rand, self.step_size)
            if not self._collision_free_segment(nodes[idx_near], q_new):
                continue

            # choose best parent among neighbors within radius
            neighs = self._neighbors_within(nodes, q_new, self.radius)
            best_parent = idx_near
            best_cost = costs[idx_near] + self._dist(nodes[idx_near], q_new)
            for j in neighs:
                c = costs[j] + self._dist(nodes[j], q_new)
                if c < best_cost and self._collision_free_segment(nodes[j], q_new):
                    best_cost = c
                    best_parent = j

            nodes.append(q_new)
            parents.append(best_parent)
            costs.append(best_cost)

            # rewire neighbors through q_new where beneficial
            new_idx = len(nodes) - 1
            for j in neighs:
                if j == best_parent:
                    continue
                c_new = costs[new_idx] + self._dist(nodes[new_idx], nodes[j])
                if c_new + 1e-9 < costs[j] and self._collision_free_segment(nodes[new_idx], nodes[j]):
                    parents[j] = new_idx
                    costs[j] = c_new

            if self._dist(q_new, self.goal) <= self.goal_thresh:
                goal_index = new_idx
                logging.info(f"RRT* reached goal vicinity in {it+1} iterations with {len(nodes)} nodes")
                break

        if goal_index is None:
            # choose node nearest to goal
            goal_index = self._nearest(nodes, self.goal)
            logging.warning("RRT*: goal not reached within iteration budget; returning best-effort path")

        return self._reconstruct(nodes, parents, goal_index)

    # ---------- helpers ----------
    def _dist(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _nearest(self, pts: List[Tuple[float, float]], q: Tuple[float, float]) -> int:
        bx = -1
        bd = 1e18
        for i, p in enumerate(pts):
            d = (p[0] - q[0]) * (p[0] - q[0]) + (p[1] - q[1]) * (p[1] - q[1])
            if d < bd:
                bd = d
                bx = i
        return bx

    def _neighbors_within(self, pts: List[Tuple[float, float]], q: Tuple[float, float], radius: float) -> List[int]:
        r2 = radius * radius
        out = []
        for i, p in enumerate(pts):
            d2 = (p[0] - q[0]) * (p[0] - q[0]) + (p[1] - q[1]) * (p[1] - q[1])
            if d2 <= r2:
                out.append(i)
        return out

    def _reconstruct(self, nodes: List[Tuple[float, float]], parents: List[int], leaf_index: int) -> List[Tuple[float, float]]:
        path = []
        i = leaf_index
        while i >= 0:
            path.append(nodes[i])
            i = parents[i]
        path.reverse()
        # densify for follower
        out = [path[0]]
        max_step = max(0.1, self.grid.cell_size * 0.5)
        for i in range(1, len(path)):
            a = out[-1]
            b = path[i]
            seg_len = self._dist(a, b)
            steps = int(max(1, math.floor(seg_len / max_step)))
            for s in range(1, steps + 1):
                t = min(1.0, s * max_step / max(1e-6, seg_len))
                out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
        return out
