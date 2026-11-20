#!/usr/bin/env python3
"""
global_planner.py -- lightweight A* global path planner on a static occupancy map.

Reads a map_server-format ``.pgm/.yaml`` pair, inflates obstacles by the robot
footprint, and runs 8-neighbour A* to produce an obstacle-free reference path.
The result is a list of world-frame waypoints ``[x, y, yaw]`` that NeuPAN consumes
via ``neupan.update_initial_path_from_waypoints``.

This module has no ROS dependency, so it can be unit-tested standalone and reused
directly from ``neupan_ros2_bridge``.

Map conventions (map_server):
  - ``.yaml`` holds ``image`` (pgm path), ``resolution``, ``origin`` [x, y, yaw],
    ``negate``, ``occupied_thresh``, ``free_thresh``.
  - Occupancy is derived from the PGM pixel value. With ``negate: 0`` (the default),
    a black pixel (0) is occupied and a white pixel (254) is free.
  - ``world = origin + (grid + 0.5) * resolution``.
"""

import heapq
import os
from math import sqrt, atan2

import numpy as np

try:
    from PIL import Image
except ImportError:  # pragma: no cover - PIL is a soft dependency for this module only
    Image = None

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

try:
    from scipy import ndimage
except ImportError:  # pragma: no cover
    ndimage = None


class GlobalPlanner:
    """Static-map A* planner producing world-frame reference waypoints."""

    def __init__(self, map_yaml_path, inflation_radius=0.4, downsample_interval=0.2):
        """
        Args:
            map_yaml_path: str, absolute path to the map_server ``.yaml`` file.
            inflation_radius: float, metres to inflate obstacles by (robot radius + margin).
            downsample_interval: float, minimum spacing (m) between output waypoints.
        """
        if Image is None or yaml is None or ndimage is None:
            raise ImportError(
                "global_planner requires PIL, PyYAML and scipy; install them before use."
            )

        self.map_yaml_path = map_yaml_path
        self.inflation_radius = inflation_radius
        self.downsample_interval = downsample_interval

        self._load_map(map_yaml_path)
        self.obstacle = self._build_occupancy()
        self.inflated = self._inflate(self.obstacle)

    # ------------------------------------------------------------------ #
    # Map loading
    # ------------------------------------------------------------------ #
    def _load_map(self, map_yaml_path):
        with open(map_yaml_path, "r") as f:
            cfg = yaml.safe_load(f)

        self.resolution = float(cfg["resolution"])
        self.origin = np.array(cfg["origin"][:2], dtype=float)
        self.negate = int(cfg.get("negate", 0))
        self.occupied_thresh = float(cfg.get("occupied_thresh", 0.65))
        self.free_thresh = float(cfg.get("free_thresh", 0.196))

        image_rel = cfg["image"]
        if not os.path.isabs(image_rel):
            image_rel = os.path.join(os.path.dirname(map_yaml_path), image_rel)
        self.map_image_path = image_rel

        img = np.array(Image.open(image_rel)).astype(np.uint8)
        self.height, self.width = img.shape
        self.pixel = img

    def _build_occupancy(self):
        """Return a boolean grid: True = occupied (untraversable)."""
        pixel = self.pixel.astype(np.float64) / 255.0
        if self.negate:
            occ = pixel
        else:
            occ = 1.0 - pixel
        return occ >= self.occupied_thresh

    def _inflate(self, obstacle):
        """Inflate occupied cells by ``inflation_radius`` using the Euclidean distance transform."""
        inflate_cells = self.inflation_radius / self.resolution
        # distance from every cell to the nearest obstacle
        dist = ndimage.distance_transform_edt(~obstacle)
        return obstacle | (dist < inflate_cells)

    # ------------------------------------------------------------------ #
    # Coordinate helpers
    # ------------------------------------------------------------------ #
    def world_to_grid(self, xy):
        """world (x, y) -> grid (gx, gy); gy increases with world y."""
        x, y = xy
        gx = int((x - self.origin[0]) / self.resolution)
        gy = int((y - self.origin[1]) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        """grid (gx, gy) -> world (x, y) at cell centre."""
        x = self.origin[0] + (gx + 0.5) * self.resolution
        y = self.origin[1] + (gy + 0.5) * self.resolution
        return x, y

    def _in_bounds(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    def _is_obstacle(self, gx, gy, grid):
        if not self._in_bounds(gx, gy):
            return True  # out of map = untraversable
        # image row 0 is the top (largest world y)
        return bool(grid[self.height - 1 - gy, gx])

    def nearest_free(self, gx, gy, grid=None):
        """Snap an occupied/out-of-bounds cell to the nearest free cell (expanding ring)."""
        grid = self.inflated if grid is None else grid
        if self._in_bounds(gx, gy) and not self._is_obstacle(gx, gy, grid):
            return gx, gy
        for radius in range(1, max(self.width, self.height)):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    nx, ny = gx + dx, gy + dy
                    if self._in_bounds(nx, ny) and not self._is_obstacle(nx, ny, grid):
                        return nx, ny
        return None

    # ------------------------------------------------------------------ #
    # A* search
    # ------------------------------------------------------------------ #
    def _astar(self, start, goal, grid):
        """8-neighbour A*. Returns a list of (gx, gy) from start to goal, or None."""
        if start == goal:
            return [start]

        def h(a, b):
            return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

        open_heap = [(h(start, goal), 0.0, start)]
        came_from = {}
        g_score = {start: 0.0}
        closed = set()

        while open_heap:
            _, g, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            closed.add(current)

            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]

            cx, cy = current
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = cx + dx, cy + dy
                    if self._is_obstacle(nx, ny, grid):
                        continue
                    step = sqrt(dx * dx + dy * dy)
                    tentative = g + step
                    if tentative < g_score.get((nx, ny), float("inf")):
                        g_score[(nx, ny)] = tentative
                        came_from[(nx, ny)] = current
                        heapq.heappush(open_heap, (tentative + h((nx, ny), goal), tentative, (nx, ny)))

        return None

    def _downsample(self, path, interval):
        """Keep points spaced >= interval (m) along the path."""
        if len(path) <= 2:
            return path
        kept = [path[0]]
        last = np.array(path[0], dtype=float)
        for p in path[1:-1]:
            pt = np.array(p, dtype=float)
            if np.linalg.norm(pt - last) >= interval:
                kept.append(p)
                last = pt
        kept.append(path[-1])
        return kept

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #
    def plan(self, start_xy, goal_xy):
        """
        Compute an obstacle-free reference path between two world positions.

        Args:
            start_xy: (x, y) world start.
            goal_xy: (x, y) world goal.

        Returns:
            list of [x, y, yaw] world-frame waypoints, or None if no path exists.
            yaw is the direction toward the next waypoint.
        """
        start = self.nearest_free(*self.world_to_grid(start_xy))
        goal = self.nearest_free(*self.world_to_grid(goal_xy))
        if start is None or goal is None:
            return None

        grid_path = self._astar(start, goal, self.inflated)
        if grid_path is None:
            return None

        grid_path = self._downsample(grid_path, self.downsample_interval / self.resolution)

        waypoints = []
        for gx, gy in grid_path:
            waypoints.append(self.grid_to_world(gx, gy))

        # assign yaw as the direction toward the next waypoint
        result = []
        for i in range(len(waypoints)):
            x, y = waypoints[i]
            if i < len(waypoints) - 1:
                nx, ny = waypoints[i + 1]
                yaw = atan2(ny - y, nx - x)
            else:
                yaw = result[-1][2]
            result.append([x, y, yaw])
        return result

    def is_free(self, xy):
        """True if the world position lies on an inflated-free cell."""
        gx, gy = self.world_to_grid(xy)
        return self._in_bounds(gx, gy) and not self._is_obstacle(gx, gy, self.inflated)


if __name__ == "__main__":
    # quick smoke test: plan across the rmul_2025 simulation map
    import sys

    default_map = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "..", "..", "..",
        "src", "fzsd2026_sentry_nav", "pb2025_nav_bringup",
        "map", "simulation", "rmul_2025.yaml",
    )
    map_path = sys.argv[1] if len(sys.argv) > 1 else os.path.normpath(default_map)

    gp = GlobalPlanner(map_path)
    # red spawn -> a point across the field
    wp = gp.plan((-5.0, 3.0), (4.8, -3.5))
    print(f"plan -> {len(wp)} waypoints")
    for p in wp[:5]:
        print(f"  {p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}")
