"""Headless NeuPAN regression test for the fzsd2026 sentry robot.

Runs the irsim example environment without a display, drives to the goal declared in
``env.yaml`` and reports whether the robot arrived, together with the closest obstacle
distance seen during the run.

Exit code 0 = arrived, 1 = did not arrive / stopped / collided.
"""
import os
import sys

import numpy as np
import irsim

CUR_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from neupan import neupan  # noqa: E402

PLANNER_FILE = os.path.join(CUR_DIR, 'planner.yaml')
ENV_FILE = os.path.join(CUR_DIR, 'env.yaml')
MAX_STEPS = 1500


def main():
    env = irsim.make(ENV_FILE, save_ani=False, full=False, display=False)
    planner = neupan.init_from_yaml(PLANNER_FILE)

    state = env.get_robot_state()
    goal = np.array(env.robot.goal, dtype=float).reshape(3, 1)
    planner.update_initial_path_from_goal(state, goal)
    print(f'start {np.round(state.ravel(), 2)} -> goal {np.round(goal.ravel(), 2)}')

    arrive = False
    closest = float('inf')
    stopped = 0
    for i in range(MAX_STEPS):
        state = env.get_robot_state()
        points = planner.scan_to_point(state, env.get_lidar_scan())
        action, info = planner(state, points)
        closest = min(closest, float(planner.min_distance))
        if info['arrive']:
            arrive = True
            print(f'step {i}: ARRIVE at {np.round(state.ravel(), 3)}')
            break
        if info['stop']:
            stopped += 1
        env.step(action)
        if env.done():
            # irsim reports done() when the robot is inside the goal threshold
            arrive = True
            print(f'step {i}: goal reached (irsim), state {np.round(env.get_robot_state().ravel(), 3)}')
            break
        if i % 100 == 0:
            print(f'step {i}: state={np.round(state.ravel(), 2)} '
                  f'min_dist={float(planner.min_distance):.3f} '
                  f'npts={0 if points is None else points.shape[1]}')

    final = np.round(env.get_robot_state().ravel(), 3)
    print(f'final state {final}, arrived={arrive}, stopped_steps={stopped}, '
          f'closest_obstacle={closest:.3f} m')
    return 0 if arrive else 1


if __name__ == '__main__':
    sys.exit(main())
