#!/usr/bin/env python3
"""Spawn a wide static wall in Gazebo (for obstacle-avoidance experiments).

    python3 web/spawn_wall.py --x 0 --y 1.5 --yaw 1.57 --length 6 --boxes 8
    python3 web/spawn_wall.py --clear            # remove the spawned wall again

The wall is a row of static boxes; `--yaw` is the direction of the wall in radians
(1.57 = along the y axis).  Models are named `neupan_wall_<i>` so they can be deleted.
"""
import argparse
import os
import subprocess
import sys

BOX_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.3f}</pose>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
        <material><ambient>0.45 0.45 0.5 1</ambient><diffuse>0.55 0.55 0.6 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""


def spawn_box(name, x, y, z, yaw, size):
    path = f'/tmp/{name}.sdf'
    with open(path, 'w') as f:
        f.write(BOX_TEMPLATE.format(name=name, x=x, y=y, z=z, yaw=yaw,
                                    sx=size[0], sy=size[1], sz=size[2]))
    cmd = ['ros2', 'run', 'ros_gz_sim', 'create', '-file', path, '-name', name,
           '-x', str(x), '-y', str(y), '-z', str(z), '-Y', str(yaw)]
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    out = (res.stdout or '') + (res.stderr or '')
    return 'OK creation of entity' in out or res.returncode == 0


def remove_models(pattern='neupan_wall'):
    try:
        listing = subprocess.run(['ign', 'model', '--list'], capture_output=True,
                                 text=True, timeout=20).stdout
    except subprocess.SubprocessError:
        return 0
    removed = 0
    for line in listing.splitlines():
        name = line.strip().lstrip('- ').strip()
        if name.startswith(pattern):
            subprocess.run(['ign', 'service', '-s', '/world/default/remove',
                            '--reqtype', 'ignition.msgs.Entity',
                            '--reptype', 'ignition.msgs.Boolean',
                            '--timeout', '2000',
                            '--req', f'name: "{name}", type: 2'],
                           capture_output=True, timeout=20)
            removed += 1
    return removed


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=1.5)
    parser.add_argument('--yaw', type=float, default=1.5708,
                        help='wall direction [rad] (1.5708 = along +y)')
    parser.add_argument('--length', type=float, default=6.0, help='total wall length [m]')
    parser.add_argument('--boxes', type=int, default=8)
    parser.add_argument('--thickness', type=float, default=0.4)
    parser.add_argument('--height', type=float, default=0.8)
    parser.add_argument('--clear', action='store_true', help='remove the walls and exit')
    args = parser.parse_args()

    if args.clear:
        print(f'removed {remove_models()} wall models')
        return 0

    remove_models()
    seg = args.length / args.boxes
    import math
    dx, dy = math.cos(args.yaw), math.sin(args.yaw)
    ok = 0
    for i in range(args.boxes):
        offset = -args.length / 2 + seg * (i + 0.5)
        x = args.x + dx * offset
        y = args.y + dy * offset
        # boxes are spaced along the wall direction; each box is seg long, thick across
        size = (seg * 0.98, args.thickness, args.height)
        if spawn_box(f'neupan_wall_{i}', x, y, args.height / 2.0, args.yaw, size):
            ok += 1
    print(f'spawned {ok}/{args.boxes} wall boxes at ({args.x}, {args.y}) '
          f'yaw {args.yaw:.2f}, length {args.length} m')
    return 0 if ok == args.boxes else 1


if __name__ == '__main__':
    sys.exit(main())
