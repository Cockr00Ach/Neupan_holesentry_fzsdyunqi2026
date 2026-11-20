"""Run the NeuPAN fzsd2026 demo in irsim.

The initial path is generated from the robot's current state to the goal declared in
``env.yaml``, then NeuPAN tracks it while avoiding the obstacles.
"""
import os
import sys

import numpy as np
import irsim

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from neupan import neupan  # noqa: E402

CUR_DIR = os.path.dirname(os.path.abspath(__file__))
PLANNER_FILE = os.path.join(CUR_DIR, 'planner.yaml')
ENV_FILE = os.path.join(CUR_DIR, 'env.yaml')


def main():
    env = irsim.make(ENV_FILE, save_ani=False, full=False, display=True)
    neupan_planner = neupan.init_from_yaml(PLANNER_FILE)

    robot_state = env.get_robot_state()
    goal = np.array(env.robot.goal, dtype=float).reshape(3, 1)
    neupan_planner.update_initial_path_from_goal(robot_state, goal)

    for i in range(2000):
        robot_state = env.get_robot_state()
        lidar_scan = env.get_lidar_scan()
        points = neupan_planner.scan_to_point(robot_state, lidar_scan)

        action, info = neupan_planner(robot_state, points)

        if info['stop']:
            print('NeuPAN stops because of minimum distance')

        if info['arrive']:
            print('NeuPAN arrives at the target!')
            break

        # Visualize
        env.draw_points(neupan_planner.dune_points, s=25, c='g', refresh=True)
        env.draw_points(neupan_planner.nrmp_points, s=13, c='r', refresh=True)
        env.draw_trajectory(neupan_planner.opt_trajectory, 'r', refresh=True)
        env.draw_trajectory(neupan_planner.ref_trajectory, 'b', refresh=True)

        env.step(action)
        env.render()

        if i == 0:
            env.draw_trajectory(neupan_planner.initial_path, traj_type='-k',
                                show_direction=False)
            env.render()

        if env.done():
            break

    env.end(3)


if __name__ == '__main__':
    main()
