#!/usr/bin/env python3
"""Dynamic obstacle test in the Gazebo arena.

The red sentry is sent to a goal across the field while the *blue* sentry is driven
straight into its path with plain velocity commands.  The bridge must steer the red
robot around the moving obstacle and still reach the goal.

It also verifies that the two robots never touch (their boxes are 0.6 x 0.5 m, so a
centre distance below ~0.55 m means contact).

    python3 example/fzsd2026/test_sim_dynamic.py            # after run_sim.sh
"""
import argparse
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.z * q.z + q.y * q.y))


class DynamicTest(Node):
    def __init__(self, goal, blocker_goal, blocker_speed=1.2):
        super().__init__('neupan_dynamic_test')
        self.goal = goal
        self.blocker_goal = blocker_goal
        self.red = None
        self.blue = None
        self.min_separation = float('inf')
        self.blocker_speed = blocker_speed
        self.start_time = None
        self.goal_sent = False

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.blue_pub = self.create_publisher(Twist, '/blue_standard_robot1/cmd_vel', 10)
        self.create_subscription(Odometry, '/red_standard_robot1/chassis_odometry_gt',
                                 self.red_cb, 10)
        self.create_subscription(Odometry, '/blue_standard_robot1/chassis_odometry_gt',
                                 self.blue_cb, 10)

    def red_cb(self, msg):
        p = msg.pose.pose.position
        self.red = (p.x, p.y, quat_to_yaw(msg.pose.pose.orientation))
        self._update_separation()

    def blue_cb(self, msg):
        p = msg.pose.pose.position
        self.blue = (p.x, p.y, quat_to_yaw(msg.pose.pose.orientation))
        self._update_separation()

    def _update_separation(self):
        if self.red is None or self.blue is None:
            return
        d = math.hypot(self.red[0] - self.blue[0], self.red[1] - self.blue[1])
        self.min_separation = min(self.min_separation, d)

    def send_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.goal[0]
        msg.pose.position.y = self.goal[1]
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self.goal_sent = True
        self.start_time = time.time()
        print(f'[test] red goal ({self.goal[0]:.2f}, {self.goal[1]:.2f})')

    def place_blocker(self, x, y):
        subprocess.run(['ign', 'service', '-s', '/world/default/set_pose',
                        '--reqtype', 'ignition.msgs.Pose',
                        '--reptype', 'ignition.msgs.Boolean',
                        '--timeout', '3000', '--req',
                        f'name: "blue_standard_robot1", position: {{x: {x}, y: {y}, z: 0.28}}, '
                        'orientation: {w: 1.0}'], capture_output=True, timeout=20)

    def hold_blocker(self):
        """Position-hold the parked robot (the sim has no brake: it would drift)."""
        if self.blue is None:
            return
        dx = self.blocker_goal[0] - self.blue[0]
        dy = self.blocker_goal[1] - self.blue[1]
        dist = math.hypot(dx, dy)
        cmd = Twist()
        if dist > 0.05:
            gain = min(0.8, 0.9 * dist)
            yaw = self.blue[2]
            c, s = math.cos(-yaw), math.sin(-yaw)
            vx, vy = gain * dx / dist, gain * dy / dist
            cmd.linear.x = c * vx - s * vy
            cmd.linear.y = s * vx + c * vy
        self.blue_pub.publish(cmd)

    def drive_blocker(self):
        if self.blue is None:
            return
        dx = self.blocker_goal[0] - self.blue[0]
        dy = self.blocker_goal[1] - self.blue[1]
        dist = math.hypot(dx, dy)
        cmd = Twist()
        if dist > 0.25:
            # the chassis controller expects the velocity in the chassis frame
            yaw = self.blue[2] if self.blue is not None else 0.0
            c, s = math.cos(-yaw), math.sin(-yaw)
            vx, vy = self.blocker_speed * dx / dist, self.blocker_speed * dy / dist
            cmd.linear.x = c * vx - s * vy
            cmd.linear.y = s * vx + c * vy
        self.blue_pub.publish(cmd)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--goal', nargs=2, type=float, default=[5.0, -1.0])
    parser.add_argument('--blocker-goal', nargs=2, type=float, default=[0.0, 1.5])
    parser.add_argument('--timeout', type=float, default=90.0)
    parser.add_argument('--tolerance', type=float, default=0.4)
    args = parser.parse_args()

    rclpy.init()
    node = DynamicTest(args.goal, args.blocker_goal)
    while node.red is None or node.blue is None:
        rclpy.spin_once(node, timeout_sec=0.2)
    print(f'[test] red start {tuple(round(v, 2) for v in node.red)}, '
          f'blue start {tuple(round(v, 2) for v in node.blue)}')

    # phase 1: park the blue robot in the red robot's way (teleport: deterministic)
    print('[test] placing the blue robot in the red robot\'s path...')
    node.place_blocker(args.blocker_goal[0], args.blocker_goal[1])
    time.sleep(1.0)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    print(f'[test] blue parked at {tuple(round(v, 2) for v in node.blue)}, '
          f'red at {tuple(round(v, 2) for v in node.red)}')

    # phase 2: send the red robot to its goal, across the parked obstacle
    time.sleep(0.5)
    node.send_goal()
    deadline = time.time() + args.timeout
    reached = False
    while time.time() < deadline:
        node.hold_blocker()
        rclpy.spin_once(node, timeout_sec=0.02)
        if node.red is None:
            continue
        dist = math.hypot(node.red[0] - args.goal[0], node.red[1] - args.goal[1])
        if dist < args.tolerance:
            reached = True
            break

    elapsed = time.time() - node.start_time
    sep = node.min_separation
    print(f'[test] arrived={reached} in {elapsed:.1f} s, '
          f'final red {tuple(round(v, 2) for v in node.red)}, '
          f'blue {tuple(round(v, 2) for v in node.blue)}')
    print(f'[test] minimum centre distance red/blue: {sep:.2f} m')
    ok = reached and sep > 0.55
    print(f'[test] {"PASS" if ok else "FAIL"}: '
          f'{"reached the goal" if reached else "did not reach the goal"}, '
          f'{"no contact" if sep > 0.55 else "robots touched"}')

    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
