#!/usr/bin/env python3
"""Rolling 2D occupancy grid built from live obstacle points, with an A* planner.

NeuPAN is a local planner: it needs a *reference path* and it tracks it while
avoiding the obstacles it sees.  A straight reference path from the robot to the goal
is fine in open space, but it can push the robot into a wall it cannot see through.

This module keeps a small map around the robot, built from the (ground-referenced)
lidar points that the bridge already computes.  The map is used to plan the reference
path with A* - on goal requests and whenever the robot gets stuck - so the demo also
works when a wall sits on the direct line.  Unknown space is assumed free: the
reference path is a hint, NeuPAN remains responsible for safety.

No ROS dependency, so it can be unit tested standalone:

    python3 local_grid.py
"""

import heapq
import math

import numpy as np

try:
    from scipy import ndimage
except ImportError:  # pragma: no cover - scipy is used for the clearance map only
    ndimage = None


class LocalGrid:
    """Rolling occupancy grid in the map frame.

    Args:
        resolution: cell size in metres.
        size: (width, height) of the grid in metres.
        center: (x, y) world position of the grid centre at construction time.
        decay: seconds after which an observation is forgotten (moving obstacles).
        min_height / max_height: limits of the height band treated as obstacles;
            height is measured from the floor (``z = 0``).
    """

    def __init__(self, resolution=0.1, size=(40.0, 40.0), center=(0.0, 0.0),
                 decay=6.0, min_height=0.06, max_height=1.4):
        self.resolution = float(resolution)
        self.width = int(round(size[0] / self.resolution))
        self.height = int(round(size[1] / self.resolution))
        self.origin = np.array([center[0] - size[0] / 2.0,
                                center[1] - size[1] / 2.0], dtype=float)
        self.decay = float(decay)
        self.min_height = float(min_height)
        self.max_height = float(max_height)

        self.last_seen = np.full(self.height * self.width, -1e9, dtype=np.float32)
        self._stamp = 0.0
        self._clearance_cache = None
        self._clearance_stamp = None

    # ------------------------------------------------------------------ #
    # Geometry helpers
    # ------------------------------------------------------------------ #
    def world_to_grid(self, xy):
        gx = int(math.floor((xy[0] - self.origin[0]) / self.resolution))
        gy = int(math.floor((xy[1] - self.origin[1]) / self.resolution))
        return gx, gy

    def grid_to_world(self, gx, gy):
        return (self.origin[0] + (gx + 0.5) * self.resolution,
                self.origin[1] + (gy + 0.5) * self.resolution)

    def in_bounds(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    # ------------------------------------------------------------------ #
    # Observation update
    # ------------------------------------------------------------------ #
    def update(self, points, stamp):
        """Insert ground-referenced obstacle points ``(3, N)`` observed at ``stamp``."""
        self._stamp = float(stamp)
        if points is None or points.shape[1] == 0:
            return
        z = points[2, :]
        keep = (z >= self.min_height) & (z <= self.max_height)
        if not np.any(keep):
            return
        gx = np.floor((points[0, keep] - self.origin[0]) / self.resolution).astype(np.int64)
        gy = np.floor((points[1, keep] - self.origin[1]) / self.resolution).astype(np.int64)
        valid = (gx >= 0) & (gx < self.width) & (gy >= 0) & (gy < self.height)
        if not np.any(valid):
            return
        idx = gy[valid] * self.width + gx[valid]
        np.maximum.at(self.last_seen, idx, np.float32(self._stamp))

    # ------------------------------------------------------------------ #
    # Occupancy / clearance
    # ------------------------------------------------------------------ #
    def occupied(self, stamp=None):
        """Boolean occupancy grid (rows = gy, cols = gx) for recent observations."""
        now = self._stamp if stamp is None else float(stamp)
        return (now - self.last_seen) < self.decay

    def inflated_occupancy(self, inflation_radius, stamp=None):
        """Occupancy dilated by ``inflation_radius`` metres (robot radius + margin)."""
        occ = self.occupied(stamp).reshape(self.height, self.width)
        if ndimage is None or inflation_radius <= 0:
            return occ
        cells = max(1.0, inflation_radius / self.resolution)
        dist = ndimage.distance_transform_edt(~occ)
        return occ | (dist < cells)

    # ------------------------------------------------------------------ #
    # A*
    # ------------------------------------------------------------------ #
    def _nearest_free(self, gx, gy, grid, max_radius=40):
        if self.in_bounds(gx, gy) and not grid[gy, gx]:
            return gx, gy
        for radius in range(1, max_radius):
            best = None
            for dx in range(-radius, radius + 1):
                for dy in (-radius, radius):
                    for nx, ny in ((gx + dx, gy + dy), (gx + dy, gy + dx)):
                        if self.in_bounds(nx, ny) and not grid[ny, nx]:
                            d = dx * dx + dy * dy
                            if best is None or d < best[0]:
                                best = (d, nx, ny)
            if best is not None:
                return best[1], best[2]
        return None

    def plan(self, start_xy, goal_xy, inflation_radius=0.4, max_expansions=120000,
             free_start=True):
        """A* between two world positions.

        The start cell is allowed to be inside the inflated band (the robot is already
        there); only the real occupancy blocks it, otherwise hugging a wall would snap
        the search start far away and produce a nonsense path.

        Returns a list of ``[x, y, yaw]`` waypoints (start -> goal) or ``None``.
        """
        grid = self.inflated_occupancy(inflation_radius)
        gx0, gy0 = self.world_to_grid(start_xy)
        if (free_start and self.in_bounds(gx0, gy0)
                and not self.occupied().reshape(self.height, self.width)[gy0, gx0]):
            grid[gy0, gx0] = False
        start = self._nearest_free(gx0, gy0, grid=grid)
        goal = self._nearest_free(*self.world_to_grid(goal_xy), grid=grid)
        if start is None or goal is None:
            return None
        if start == goal:
            return [[start_xy[0], start_xy[1], 0.0]]

        def h(cell):
            return math.hypot(cell[0] - goal[0], cell[1] - goal[1])

        open_heap = [(h(start), 0.0, start)]
        came_from = {}
        g_score = {start: 0.0}
        closed = set()
        expansions = 0
        while open_heap:
            _, g, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            closed.add(current)
            expansions += 1
            if expansions > max_expansions:
                return None
            if current == goal:
                cells = [current]
                while current in came_from:
                    current = came_from[current]
                    cells.append(current)
                cells.reverse()
                return self._to_waypoints(cells)
            cx, cy = current
            for dx, dy, cost in _NEIGHBOURS:
                nx, ny = cx + dx, cy + dy
                if not self.in_bounds(nx, ny) or grid[ny, nx]:
                    continue
                tentative = g + cost
                if tentative < g_score.get((nx, ny), float('inf')):
                    g_score[(nx, ny)] = tentative
                    came_from[(nx, ny)] = current
                    heapq.heappush(open_heap, (tentative + h((nx, ny)), tentative, (nx, ny)))
        return None

    def _to_waypoints(self, cells, step=4):
        """Convert grid cells to [x, y, yaw] waypoints, keeping about ``step`` cells."""
        kept = cells[::step]
        if kept[-1] != cells[-1]:
            kept.append(cells[-1])
        pts = [self.grid_to_world(gx, gy) for gx, gy in kept]
        out = []
        for i, (x, y) in enumerate(pts):
            if i < len(pts) - 1:
                nx, ny = pts[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = out[-1][2] if out else 0.0
            out.append([x, y, yaw])
        return out

    def clearance(self):
        """Distance (m) from every cell to the nearest observed obstacle cell."""
        if ndimage is None:
            return np.full((self.height, self.width), np.inf, dtype=np.float32)
        occ = self.occupied().reshape(self.height, self.width)
        return (ndimage.distance_transform_edt(~occ) * self.resolution).astype(np.float32)

    def is_free(self, xy, inflation_radius=0.4):
        grid = self.inflated_occupancy(inflation_radius)
        gx, gy = self.world_to_grid(xy)
        return self.in_bounds(gx, gy) and not grid[gy, gx]


_NEIGHBOURS = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
]


if __name__ == '__main__':
    # self test: a wall between start and goal must be avoided
    grid = LocalGrid(resolution=0.1, size=(20.0, 20.0), center=(0.0, 0.0), decay=1000.0)
    xs = np.arange(-0.1, 0.1, 0.02)
    ys = np.arange(-4.0, 4.0, 0.02)
    points = np.vstack([np.tile(xs, len(ys)), np.repeat(ys, len(xs)),
                        np.full(len(xs) * len(ys), 0.4)])
    grid.update(points, stamp=1.0)
    path = grid.plan((-5.0, 0.0), (5.0, 0.0), inflation_radius=0.3)
    assert path is not None, 'no path around the wall'
    length = sum(math.dist(path[i][:2], path[i + 1][:2]) for i in range(len(path) - 1))
    print(f'path around the wall: {len(path)} waypoints, {length:.2f} m '
          f'(straight line would be 10.00 m)')
    assert grid.is_free((5.0, 0.0)) and not grid.is_free((0.0, 0.0))
    assert length > 10.5, 'the path should detour around the wall'
    print('LocalGrid self-test passed')
