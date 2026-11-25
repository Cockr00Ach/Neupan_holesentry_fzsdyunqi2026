#!/usr/bin/env python3
"""Reproduce/regress the "twitch in place" behaviour of the NeuPAN bridge.

Teleports the robot to --start, sends --goal over /goal_pose and records
/neupan_status (20 Hz) into a CSV, then reports:

    * whether the goal was reached (and how long it took)
    * the travelled distance vs. the straight-line distance
    * the number of command direction reversals (oscillation count)
    * the worst "twitch window": 1 s windows with little displacement but a lot of
      motion, i.e. the robot shaking in place
    * the minimum obstacle clearance seen

Exit code 0 = reached the goal without twitching.

    python3 test_sim_twitch.py --start 3.66 -2.02 --goal -3.64 2.02
"""
import argparse
import csv
import math
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


class TwitchTest(Node):
    def __init__(self, start, goal, robot='red_standard_robot1'):
        super().__init__('neupan_twitch_test')
        self.robot = robot
        self.start = start
        self.goal = goal
        self.rows = []
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_subscription(String, '/neupan_status', self.on_status, 10)
        self.t0 = None

    def teleport(self, x, y):
        req = (f'name: "{self.robot}", position: {{x: {x}, y: {y}, z: 0.28}}, '
               f'orientation: {{w: 1.0}}')
        subprocess.run(['ign', 'service', '-s', '/world/default/set_pose',
                        '--reqtype', 'ign.msgs.Pose' if False else 'ignition.msgs.Pose',
                        '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000',
                        '--req', req], capture_output=True, timeout=20)

    def on_status(self, msg):
        try:
            import json
            st = json.loads(msg.data)
        except ValueError:
            return
        if self.t0 is None or st.get('pose') is None:
            return
        self.rows.append({
            't': time.time() - self.t0,
            'x': st['pose'][0], 'y': st['pose'][1], 'yaw': st['pose'][2],
            'vx': st['cmd'][0], 'vy': st['cmd'][1],
            'min_dist': st.get('min_dist', 0.0),
            'mode': st.get('mode', ''),
        })

    def send_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(self.goal[0])
        msg.pose.position.y = float(self.goal[1])
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)
        self.t0 = time.time()


def analyse(rows, start, goal, tolerance):
    if not rows:
        return {'arrived': False, 'reason': 'no status frames'}
    t = np.array([r['t'] for r in rows])
    xy = np.array([[r['x'], r['y']] for r in rows])
    v = np.array([[r['vx'], r['vy']] for r in rows])
    md = np.array([r['min_dist'] for r in rows])

    step = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    travelled = float(step.sum())
    straight = float(np.linalg.norm(np.array(goal) - np.array(start)))

    # oscillation: a command reversal is a sign flip of the displacement direction
    reversals = 0
    speed = np.linalg.norm(v, axis=1)
    dirs = np.zeros_like(v)
    mask = speed > 0.15
    dirs[mask] = v[mask] / speed[mask][:, None]
    for i in range(1, len(dirs)):
        if mask[i] and mask[i - 1] and float(dirs[i] @ dirs[i - 1]) < -0.3:
            reversals += 1

    # twitch windows: 1 s window with < 0.12 m displacement but > 0.3 m path length
    twitch = []
    j = 0
    for i in range(len(t)):
        while t[i] - t[j] > 1.0:
            j += 1
        if i - j < 5:
            continue
        win_step = step[j:i]
        if win_step.size == 0:
            continue
        disp = float(np.linalg.norm(xy[i] - xy[j]))
        path = float(win_step.sum())
        if disp < 0.12 and path > 0.3:
            twitch.append((t[i], disp, path))

    d_goal = np.linalg.norm(xy - np.array(goal), axis=1)
    arrived_idx = np.where(d_goal < tolerance)[0]
    arrived = bool(arrived_idx.size)
    return {
        'arrived': arrived,
        'time_to_goal': float(t[arrived_idx[0]]) if arrived else None,
        'final': xy[-1].tolist(),
        'final_distance': float(d_goal[-1]),
        'travelled': travelled,
        'straight': straight,
        'detour_ratio': travelled / max(straight, 0.1),
        'reversals': reversals,
        'twitch_windows': len(twitch),
        'worst_twitch': max(twitch, key=lambda w: w[2]) if twitch else None,
        'min_clearance': float(md.min()),
        'duration': float(t[-1]),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--start', nargs=2, type=float, default=[3.66, -2.02])
    parser.add_argument('--goal', nargs=2, type=float, default=[-3.64, 2.02])
    parser.add_argument('--timeout', type=float, default=90.0)
    parser.add_argument('--tolerance', type=float, default=0.4)
    parser.add_argument('--csv', default='/tmp/neupan_twitch.csv')
    args = parser.parse_args()

    rclpy.init()
    node = TwitchTest(tuple(args.start), tuple(args.goal))
    # wait for a status frame before teleporting
    t_end = time.time() + 10
    while time.time() < t_end and not node.rows:
        rclpy.spin_once(node, timeout_sec=0.2)
    print(f'[test] teleport to {tuple(args.start)}, goal {tuple(args.goal)}')
    node.teleport(*args.start)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    node.send_goal()

    deadline = time.time() + args.timeout
    last_progress = None
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if not node.rows:
            continue
        last = node.rows[-1]
        d = math.hypot(last['x'] - args.goal[0], last['y'] - args.goal[1])
        if d < args.tolerance:
            break

    result = analyse(node.rows, args.start, args.goal, args.tolerance)
    if node.rows:
        with open(args.csv, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(node.rows[0].keys()))
            writer.writeheader()
            writer.writerows(node.rows)
    print(f'[test] arrived={result["arrived"]} '
          f'time={result.get("time_to_goal")} final={[round(v, 2) for v in result.get("final", [])]} '
          f'dist={result.get("final_distance", 0):.2f}')
    print(f'[test] travelled={result.get("travelled", 0):.2f} m '
          f'(straight {result.get("straight", 0):.2f} m, '
          f'ratio {result.get("detour_ratio", 0):.2f})')
    print(f'[test] reversals={result.get("reversals")} '
          f'twitch_windows={result.get("twitch_windows")} '
          f'worst={result.get("worst_twitch")}')
    print(f'[test] min_clearance={result.get("min_clearance", 0):.3f} m '
          f'duration={result.get("duration", 0):.1f} s -> {args.csv}')
    ok = result['arrived'] and result.get('twitch_windows', 0) <= 1
    print('[test] ' + ('PASS' if ok else 'FAIL'))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
