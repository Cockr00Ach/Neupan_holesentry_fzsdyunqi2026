#!/usr/bin/env python3
"""Build a map_server (.pgm/.yaml) occupancy map of the running Gazebo world.

The maps shipped in ``pb2025_nav_bringup/map`` come from the real robot and do not
share the Gazebo world frame, so an A* on them plans through walls.  This tool
scans the *actual* simulation instead: it teleports the robot to a few poses and
accumulates the filtered obstacle cloud that the NeuPAN bridge publishes on
``/neupan_obstacles`` (map frame, ground referenced), then writes a
map_server-compatible map.

    # with the simulator and the bridge running (./run_sim.sh --wait)
    python3 build_map_from_lidar.py --output map/rmul_2025_sim

    # then use it:
    ./run_sim.sh --wait
    ros2 ... -p use_global_planner:=true -p global_map:=<abs path to the .yaml>

The scan poses default to five well spread points of the rmul_2025 field; pass
``--poses "x,y x,y ..."`` to scan another arena.
"""
import argparse
import math
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
from PIL import Image
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

DEFAULT_POSES = [(-4.0, 2.0), (4.0, 2.0), (-4.0, -6.0), (4.0, -6.0), (5.0, -2.0)]


class MapBuilder(Node):
    def __init__(self, robot):
        super().__init__('neupan_map_builder')
        self.cloud = None
        self.points = []
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(PointCloud2, '/neupan_obstacles', self.cb, qos)
        self.robot = robot

    def cb(self, msg):
        raw = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if isinstance(raw, np.ndarray) and raw.dtype.names:
            pts = np.stack([raw['x'], raw['y'], raw['z']], axis=1)
        else:
            pts = np.asarray(raw, dtype=np.float64).reshape(-1, 3)
        pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.size:
            self.points.append(pts)

    def teleport(self, x, y, yaw=0.0, z=0.28, timeout=6.0):
        req = (f'name: "{self.robot}", position: {{x: {x}, y: {y}, z: {z}}}, '
               f'orientation: {{z: {math.sin(yaw / 2.0)}, w: {math.cos(yaw / 2.0)}}}')
        try:
            subprocess.run(
                ['ign', 'service', '-s', '/world/default/set_pose',
                 '--reqtype', 'ignition.msgs.Pose',
                 '--reptype', 'ignition.msgs.Boolean',
                 f'--timeout', str(int(timeout * 1000)), '--req', req],
                check=True, capture_output=True, timeout=timeout + 4)
        except (subprocess.SubprocessError, FileNotFoundError) as exc:
            print(f'[map] teleport failed: {exc}')
            return False
        return True

    def collect(self, settle=1.6):
        """Collect clouds published after the last call (the bridge runs at 50 Hz)."""
        self.points = []
        t0 = time.time()
        while time.time() - t0 < settle:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not self.points:
            return np.zeros((0, 3))
        return np.vstack(self.points)


def write_map(occ, origin, resolution, prefix):
    height, width = occ.shape
    img = np.where(occ[::-1, :], 0, 254).astype(np.uint8)  # row 0 = max y
    os.makedirs(os.path.dirname(os.path.abspath(prefix)), exist_ok=True)
    Image.fromarray(img, mode='L').save(prefix + '.pgm')
    with open(prefix + '.yaml', 'w') as f:
        f.write(f'image: {os.path.basename(prefix)}.pgm\n')
        f.write('mode: trinary\n')
        f.write(f'resolution: {resolution}\n')
        f.write(f'origin: [{origin[0]:.4f}, {origin[1]:.4f}, 0.0]\n')
        f.write('negate: 0\n')
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.25\n')
    print(f'[map] wrote {prefix}.pgm ({width}x{height} cells) and {prefix}.yaml')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--robot', default='red_standard_robot1')
    parser.add_argument('--output', default=None,
                        help='output prefix (default <this dir>/map/<robot world>)')
    parser.add_argument('--poses', default=None,
                        help='scan poses as "x,y x,y ..." (default: rmul_2025 field)')
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--size', type=float, nargs=2, default=[24.0, 24.0])
    parser.add_argument('--center', type=float, nargs=2, default=[0.0, -1.5])
    parser.add_argument('--settle', type=float, default=1.6)
    parser.add_argument('--dilate', type=float, default=0.15,
                        help='grow the scanned points by this radius [m] so that the '
                             'dotted wall returns become continuous lines')
    args = parser.parse_args()

    if args.poses:
        poses = [tuple(float(v) for v in p.split(',')) for p in args.poses.split()]
    else:
        poses = DEFAULT_POSES
    output = args.output or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         'map', 'rmul_2025_sim')

    rclpy.init()
    node = MapBuilder(args.robot)
    width = int(round(args.size[0] / args.resolution))
    height = int(round(args.size[1] / args.resolution))
    origin = np.array([args.center[0] - args.size[0] / 2.0,
                       args.center[1] - args.size[1] / 2.0])
    occ = np.zeros((height, width), dtype=bool)

    try:
        for i, (x, y) in enumerate(poses):
            print(f'[map] pose {i + 1}/{len(poses)}: ({x:.1f}, {y:.1f})')
            if not node.teleport(x, y):
                continue
            pts = node.collect(args.settle)
            if pts.shape[0] == 0:
                print('[map] warning: no obstacle points at this pose '
                      '(is the bridge running?)')
                continue
            gx = np.floor((pts[:, 0] - origin[0]) / args.resolution).astype(np.int64)
            gy = np.floor((pts[:, 1] - origin[1]) / args.resolution).astype(np.int64)
            valid = (gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)
            occ[gy[valid], gx[valid]] = True
            print(f'[map] {int(valid.sum())} points accumulated')
    finally:
        if args.dilate > 0:
            try:
                from scipy import ndimage
                cells = args.dilate / args.resolution
                yy, xx = np.mgrid[-int(cells) - 1:int(cells) + 2,
                                  -int(cells) - 1:int(cells) + 2]
                disk = (xx * xx + yy * yy) <= cells * cells
                occ = ndimage.binary_dilation(occ, structure=disk)
                print(f'[map] dilated by {args.dilate:.2f} m')
            except ImportError:
                print('[map] scipy not available, skipping dilation')
        write_map(occ, origin, args.resolution, output)
        node.destroy_node()
        rclpy.shutdown()

    print(f'[map] occupied cells: {int(occ.sum())}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
