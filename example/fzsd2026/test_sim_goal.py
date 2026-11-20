#!/usr/bin/env python3
"""Drive the simulated sentry to a goal through the NeuPAN bridge and verify the result.

The test publishes a ``/goal_pose`` (the same topic RViz' "2D Goal Pose" tool uses),
records the chassis odometry and the commanded velocity, and reports whether the robot
reached the goal and stopped inside the tolerance.

Example::

    ros2 launch ...                       # bring up the simulator
    python3 example/fzsd2026/neupan_ros2_bridge.py --ros-args \
        --params-file example/fzsd2026/bridge_params.yaml &
    python3 example/fzsd2026/test_sim_goal.py --goal 6 -6 0 --timeout 90
"""
import argparse
import csv
import math
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.z * q.z + q.y * q.y))


class GoalTest(Node):
    def __init__(self, goal):
        super().__init__('neupan_goal_test')
        self.goal = goal
        self.pose = None
        self.speed = 0.0
        self.trace = []
        self.start_time = None
        self._last = None
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_subscription(Odometry, '/red_standard_robot1/chassis_odometry_gt',
                                 self.odom_cb, 10)
        self.timer = self.create_timer(10.0, self.publish_goal)
        self.goal_count = 0

    def publish_goal(self):
        if self.pose is None or self.goal_count >= 1:
            return
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.goal[0]
        msg.pose.position.y = self.goal[1]
        msg.pose.orientation.z = math.sin(self.goal[2] / 2.0)
        msg.pose.orientation.w = math.cos(self.goal[2] / 2.0)
        self.pub.publish(msg)
        self.goal_count += 1
        self.start_time = time.time()
        print(f'[test] goal published: ({self.goal[0]:.2f}, {self.goal[1]:.2f}, {self.goal[2]:.2f})')

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        now = time.time()
        prev = self._last
        if prev is not None:
            dt = max(now - prev[3], 1e-3)
            self.speed = math.hypot(p.x - prev[0], p.y - prev[1]) / dt
        self._last = (p.x, p.y, yaw, now)
        self.pose = (p.x, p.y, yaw)
        if self.start_time is not None:
            self.trace.append((time.time() - self.start_time, p.x, p.y, yaw, self.speed))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--goal', nargs=3, type=float, default=[5.0, 0.0, 0.0],
                        metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--tolerance', type=float, default=0.35)
    parser.add_argument('--csv', default='/tmp/neupan_goal_trace.csv')
    args = parser.parse_args()

    rclpy.init()
    node = GoalTest(args.goal)
    print('[test] waiting for odometry...')
    while node.pose is None:
        rclpy.spin_once(node, timeout_sec=0.2)

    start = node.pose
    print(f'[test] start pose ({start[0]:.2f}, {start[1]:.2f}, {start[2]:.2f})')

    deadline = time.time() + args.timeout
    reached_at = None
    min_speed_after = None
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.goal_count and node.pose is not None:
            dist = math.hypot(node.pose[0] - args.goal[0], node.pose[1] - args.goal[1])
            if dist < args.tolerance and node.speed < 0.05:
                reached_at = time.time()
                break

    if reached_at is None:
        print('[test] FAILED: goal not reached before the timeout')
        print(f'[test] final pose {tuple(round(v, 3) for v in node.pose)}')
        ok = False
    else:
        print(f'[test] reached the goal in {reached_at - node.start_time:.2f} s '
              f'at ({node.pose[0]:.2f}, {node.pose[1]:.2f}), speed {node.speed:.3f} m/s')
        # the chassis controller keeps the last command, so make sure it really stops
        t0 = time.time()
        while time.time() - t0 < 3.0:
            rclpy.spin_once(node, timeout_sec=0.05)
        min_speed_after = node.speed
        print(f'[test] speed 3 s after arrival: {min_speed_after:.3f} m/s')
        ok = min_speed_after < 0.05

    if node.trace:
        with open(args.csv, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t', 'x', 'y', 'yaw', 'speed'])
            writer.writerows(node.trace)
        print(f'[test] trace written to {args.csv} ({len(node.trace)} samples)')

    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
