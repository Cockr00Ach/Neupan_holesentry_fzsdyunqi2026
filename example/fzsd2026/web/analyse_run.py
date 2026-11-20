#!/usr/bin/env python3
"""Analyse a recorded run: trajectory quality for the NeuPAN bridge.

Reads a rosbag (or the status CSV) and prints
    * arrival / time to goal
    * travelled vs straight-line distance
    * command reversals and "twitch" windows (little movement, lots of motion)
    * minimum obstacle clearance
    * a coarse ASCII trajectory next to the obstacle map

    python3 analyse_run.py --bag /tmp/run_baseline --goal -3.64 2.02
"""
import argparse
import json
import math
import sys

import numpy as np


def read_status_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic != '/neupan_status':
            continue
        msg = deserialize_message(data, String)
        try:
            st = json.loads(msg.data)
        except ValueError:
            continue
        out.append(st)
    return out


def analyse(states, start, goal, tolerance=0.4):
    rows = [s for s in states if s.get('pose')]
    if not rows:
        return None
    t = np.array([i * 0.05 for i in range(len(rows))])  # nominal 20 Hz
    xy = np.array([[s['pose'][0], s['pose'][1]] for s in rows])
    v = np.array([[s['cmd'][0], s['cmd'][1]] for s in rows])
    md = np.array([s.get('min_dist', 0.0) for s in rows])
    step = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    travelled = float(step.sum())
    straight = float(np.linalg.norm(np.array(goal) - np.array(start)))

    speed = np.linalg.norm(v, axis=1)
    mask = speed > 0.2
    dirs = np.zeros_like(v)
    dirs[mask] = v[mask] / speed[mask][:, None]
    reversals = 0
    for i in range(1, len(dirs)):
        if mask[i] and mask[i - 1] and float(dirs[i] @ dirs[i - 1]) < -0.3:
            reversals += 1

    twitch = 0
    j = 0
    n = int(round(1.0 / 0.05))
    for i in range(len(xy)):
        j = max(0, i - n)
        if i - j < n // 2:
            continue
        disp = float(np.linalg.norm(xy[i] - xy[j]))
        path = float(step[j:i].sum()) if step[j:i].size else 0.0
        if disp < 0.12 and path > 0.3:
            twitch += 1

    d_goal = np.linalg.norm(xy - np.array(goal), axis=1)
    hit = np.where(d_goal < tolerance)[0]
    return {
        'arrived': bool(hit.size),
        'time': float(t[hit[0]]) if hit.size else None,
        'final': xy[-1],
        'travelled': travelled,
        'straight': straight,
        'ratio': travelled / max(straight, 0.1),
        'reversals': reversals,
        'twitch_windows': twitch,
        'min_clearance': float(md.min()),
        'duration': float(t[-1]),
        'xy': xy,
    }


def ascii_trajectory(xy, cloud, goal, start, res=0.25, lim=7.0):
    xs = np.arange(-lim, lim, res)
    ys = np.arange(lim, -lim - 1e-9, -res)
    grid = np.zeros((len(ys), len(xs)), dtype=bool)
    if cloud is not None and len(cloud):
        xi = ((cloud[:, 0] + lim) / res).astype(int)
        yi = ((lim - cloud[:, 1]) / res).astype(int)
        ok = (xi >= 0) & (xi < len(xs)) & (yi >= 0) & (yi < len(ys))
        grid[yi[ok], xi[ok]] = True
    marks = {}
    for k, (x, y) in enumerate(xy):
        xi = int((x + lim) / res)
        yi = int((lim - y) / res)
        if 0 <= xi < len(xs) and 0 <= yi < len(ys):
            marks[(yi, xi)] = '.' if (yi, xi) in marks else 'o'
    for name, (x, y) in (('S', start), ('G', goal)):
        xi = int((x + lim) / res)
        yi = int((lim - y) / res)
        if 0 <= xi < len(xs) and 0 <= yi < len(ys):
            marks[(yi, xi)] = name
    lines = []
    for j, y in enumerate(ys):
        lines.append(f'{y:5.1f} ' + ''.join(marks.get((j, i), '#' if grid[j, i] else '.')
                                            for i in range(len(xs))))
    lines.append('      ' + ''.join(str(int(abs(x)) % 10) for x in xs))
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True)
    parser.add_argument('--goal', nargs=2, type=float, default=[-3.64, 2.02])
    parser.add_argument('--start', nargs=2, type=float, default=None)
    parser.add_argument('--ascii', action='store_true')
    args = parser.parse_args()

    states = read_status_bag(args.bag)
    if not states:
        print('no /neupan_status in the bag')
        return 1
    rows = [s for s in states if s.get('pose')]
    start = args.start or rows[0]['pose'][:2]
    res = analyse(states, start, args.goal)
    if res is None:
        print('no usable frames')
        return 1
    print(f'arrived={res["arrived"]} time={res["time"]} '
          f'final={np.round(res["final"], 2).tolist()}')
    print(f'travelled={res["travelled"]:.2f} m straight={res["straight"]:.2f} m '
          f'ratio={res["ratio"]:.2f}')
    print(f'reversals={res["reversals"]} twitch_windows={res["twitch_windows"]} '
          f'min_clearance={res["min_clearance"]:.3f} m duration={res["duration"]:.1f} s')
    if args.ascii:
        print(ascii_trajectory(res['xy'], None, args.goal, start))
    return 0


if __name__ == '__main__':
    sys.exit(main())
