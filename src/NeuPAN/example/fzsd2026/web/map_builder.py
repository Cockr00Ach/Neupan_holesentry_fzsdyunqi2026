#!/usr/bin/env python3
"""SLAM-lite: accumulate the live obstacle cloud into a 2D occupancy grid.

The NeuPAN bridge publishes the filtered, ground-referenced obstacle points in the map
frame (``/neupan_obstacles``); together with the odometry pose that is enough to build a
map of the arena while the robot drives:

    * every point casts a free ray from the current sensor position ("free space carving")
    * the end point of the ray is marked occupied
    * updates are accumulated in log-odds and clamped, so noise averages out

Outputs

    /neupan_map           nav_msgs/OccupancyGrid   live map (also usable by RViz/nav2)
    /neupan_save_map      std_msgs/String          save request: payload = file prefix
    /neupan_reset_map     std_msgs/Empty           clear the map

    docs/…/map/<prefix>.pgm/.yaml                  map_server format, loadable by
                                                   global_planner.py (use_global_planner)

    python3 web/map_builder.py                       # together with run_sim.sh
    ros2 topic pub --once /neupan_save_map std_msgs/msg/String "{data: 'map/rmul_2025_slam'}"
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from PIL import Image
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty, String

L_OCC = 0.85          # log-odds increment for a hit
L_FREE = -0.40        # log-odds increment for a seen-through cell
L_MIN, L_MAX = -4.0, 4.0
OCC_THRESHOLD = 0.6   # log-odds above this = occupied
FREE_THRESHOLD = -0.6


def decode_xyz(msg: PointCloud2):
    n = int(msg.width) * int(msg.height)
    if n <= 0 or msg.point_step <= 0 or not msg.data:
        return None
    fields = {f.name: f for f in msg.fields}
    if not {'x', 'y', 'z'} <= fields.keys():
        return None
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if raw.size < n * msg.point_step:
        n = raw.size // msg.point_step
        if n <= 0:
            return None
        raw = raw[:n * msg.point_step]
    rows = raw.reshape(n, msg.point_step)
    xyz = np.empty((n, 3), dtype=np.float32)
    for i, name in enumerate(('x', 'y', 'z')):
        off = fields[name].offset
        xyz[:, i] = np.frombuffer(rows[:, off:off + 4].tobytes(), dtype=np.float32)
    finite = np.isfinite(xyz).all(axis=1)
    return xyz[finite]


class GridMapper:
    """Log-odds occupancy grid with free-space carving."""

    def __init__(self, resolution=0.05, size=(24.0, 24.0), center=(0.0, -1.0)):
        self.resolution = float(resolution)
        self.width = int(round(size[0] / self.resolution))
        self.height = int(round(size[1] / self.resolution))
        self.origin = np.array([center[0] - size[0] / 2.0,
                                center[1] - size[1] / 2.0], dtype=float)
        self.logodds = np.zeros((self.height, self.width), dtype=np.float32)
        self.updates = 0

    # geometry ---------------------------------------------------------- #
    def world_to_cell(self, xy):
        gx = np.floor((xy[..., 0] - self.origin[0]) / self.resolution).astype(np.int64)
        gy = np.floor((xy[..., 1] - self.origin[1]) / self.resolution).astype(np.int64)
        return gx, gy

    def cell_to_world(self, gx, gy):
        return (self.origin[0] + (gx + 0.5) * self.resolution,
                self.origin[1] + (gy + 0.5) * self.resolution)

    # update ------------------------------------------------------------ #
    def update(self, points, origin_xy, samples=60, min_range=0.25):
        if points is None or len(points) == 0:
            return
        pts = np.asarray(points, dtype=np.float64)[:, :2]
        delta = pts - np.asarray(origin_xy, dtype=np.float64)
        dist = np.hypot(delta[:, 0], delta[:, 1])
        keep = dist > min_range
        if not np.any(keep):
            return
        delta, dist = delta[keep], dist[keep]

        t = np.linspace(0.03, 0.97, samples)[None, :]
        along = dist[:, None] * t                        # (N, S) metres along the ray
        xs = origin_xy[0] + delta[:, 0:1] * t
        ys = origin_xy[1] + delta[:, 1:2] * t
        gx = np.floor((xs - self.origin[0]) / self.resolution).astype(np.int64)
        gy = np.floor((ys - self.origin[1]) / self.resolution).astype(np.int64)
        valid = ((along > min_range) & (gx >= 0) & (gx < self.width)
                 & (gy >= 0) & (gy < self.height))
        np.add.at(self.logodds, (gy[valid], gx[valid]), L_FREE)

        # end points = occupied
        ex, ey = self.world_to_cell(pts)
        ok = (ex >= 0) & (ex < self.width) & (ey >= 0) & (ey < self.height)
        np.add.at(self.logodds, (ey[ok], ex[ok]), L_OCC)
        np.clip(self.logodds, L_MIN, L_MAX, out=self.logodds)
        self.updates += 1

    # queries ----------------------------------------------------------- #
    def occupied(self):
        return self.logodds > OCC_THRESHOLD

    def known(self):
        return np.abs(self.logodds) > 0.15

    def to_nav_msgs(self, frame_id='map', stamp=None):
        msg = OccupancyGrid()
        msg.header.frame_id = frame_id
        if stamp is not None:
            msg.header.stamp = stamp
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = float(self.origin[0])
        msg.info.origin.position.y = float(self.origin[1])
        msg.info.origin.orientation.w = 1.0
        grid = np.full((self.height, self.width), -1, dtype=np.int8)
        grid[self.known()] = 0
        grid[self.occupied()] = 100
        # OccupancyGrid rows: y increases upwards, same as our array
        msg.data = grid.reshape(-1).tolist()
        return msg

    def save(self, prefix):
        """Write a map_server compatible .pgm/.yaml pair."""
        occ = self.occupied()
        known = self.known()
        img = np.full((self.height, self.width), 205, dtype=np.uint8)
        img[known] = 254
        img[occ] = 0
        img = img[::-1, :]                     # image row 0 = max y
        directory = os.path.dirname(os.path.abspath(prefix))
        os.makedirs(directory, exist_ok=True)
        Image.fromarray(img, mode='L').save(prefix + '.pgm')
        with open(prefix + '.yaml', 'w') as f:
            f.write(f'image: {os.path.basename(prefix)}.pgm\n')
            f.write('mode: trinary\n')
            f.write(f'resolution: {self.resolution}\n')
            f.write(f'origin: [{self.origin[0]:.3f}, {self.origin[1]:.3f}, 0.0]\n')
            f.write('negate: 0\n')
            f.write('occupied_thresh: 0.65\n')
            f.write('free_thresh: 0.25\n')
        return prefix + '.pgm', prefix + '.yaml'


class MapBuilderNode(Node):
    def __init__(self, resolution=0.05, size=(24.0, 24.0), center=(0.0, -1.0),
                 autosave=''):
        super().__init__('neupan_map_builder')
        self.mapper = GridMapper(resolution, size, center)
        self.autosave = autosave
        self.pose = None
        self.last_cloud = 0.0
        self.cloud_period = 0.15    # use at most ~7 clouds per second
        self.map_pub = self.create_publisher(OccupancyGrid, '/neupan_map', 2)
        self.create_subscription(PointCloud2, '/neupan_obstacles', self.on_cloud, 10)
        self.create_subscription(Odometry, '/red_standard_robot1/chassis_odometry_gt',
                                 self.on_odom, 10)
        self.create_subscription(String, '/neupan_save_map', self.on_save, 10)
        self.create_subscription(Empty, '/neupan_reset_map', self.on_reset, 10)
        self.create_timer(1.5, self.publish_map)
        self.get_logger().info(
            f'map builder ready: {self.mapper.width}x{self.mapper.height} cells @ '
            f'{resolution} m, origin {np.round(self.mapper.origin, 2)}')

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    def on_cloud(self, msg):
        if self.pose is None:
            return
        now = time.time()
        if now - self.last_cloud < self.cloud_period:
            return
        self.last_cloud = now
        pts = decode_xyz(msg)
        if pts is None or len(pts) == 0:
            return
        self.mapper.update(pts, self.pose)

    def publish_map(self):
        msg = self.mapper.to_nav_msgs(stamp=self.get_clock().now().to_msg())
        self.map_pub.publish(msg)

    def on_save(self, msg):
        prefix = msg.data.strip() or 'map/neupan_slam'
        try:
            pgm, yml = self.mapper.save(prefix)
            self.get_logger().info(f'saved map: {pgm} / {yml} '
                                   f'({int(self.mapper.occupied().sum())} occupied cells)')
        except OSError as exc:
            self.get_logger().warn(f'cannot save the map: {exc}')

    def on_reset(self, _msg):
        self.mapper = GridMapper(self.mapper.resolution,
                                 (self.mapper.width * self.mapper.resolution,
                                  self.mapper.height * self.mapper.resolution),
                                 (self.mapper.origin[0] + self.mapper.width * self.mapper.resolution / 2,
                                  self.mapper.origin[1] + self.mapper.height * self.mapper.resolution / 2))
        self.get_logger().info('map cleared')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--size', type=float, nargs=2, default=[24.0, 24.0])
    parser.add_argument('--center', type=float, nargs=2, default=[0.0, -1.0])
    parser.add_argument('--autosave', default='',
                        help='write the map here when the node is stopped (Ctrl-C)')
    args = parser.parse_args()

    rclpy.init()
    node = MapBuilderNode(args.resolution, tuple(args.size), tuple(args.center),
                          args.autosave)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if args.autosave:
            try:
                pgm, yml = node.mapper.save(args.autosave)
                print(f'[map] saved on exit: {pgm} / {yml}')
            except OSError as exc:
                print(f'[map] cannot save: {exc}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
