"""Run dyna_obs omni demo with moving obstacles."""
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import irsim
from neupan import neupan

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

CUR_DIR = os.path.dirname(os.path.abspath(__file__))
PLANNER_FILE = os.path.join(CUR_DIR, 'planner.yaml')
ENV_FILE = os.path.join(CUR_DIR, 'env.yaml')

# Right-click to set new goal
clicked_goal = None

def on_click(event):
    global clicked_goal
    if event.button == 3 and event.inaxes is not None:
        clicked_goal = (event.xdata, event.ydata)


def main():
    global clicked_goal

    env = irsim.make(ENV_FILE, save_ani=False, full=False, display=True)
    neupan_planner = neupan.init_from_yaml(PLANNER_FILE)

    plt.connect('button_press_event', on_click)

    print("=" * 50)
    print("Dynamic Obstacle Demo - 20 moving obstacles (RVO)")
    print("Right click on map -> SET NEW GOAL")
    print("=" * 50)

    last_goal = None

    for i in range(5000):
        robot_state = env.get_robot_state()

        if clicked_goal is not None and clicked_goal != last_goal:
            last_goal = clicked_goal
            goal = np.array([[clicked_goal[0]], [clicked_goal[1]], [0.0]])
            neupan_planner.update_initial_path_from_goal(robot_state, goal)
            neupan_planner.reset()
            print(f">>> New goal: ({clicked_goal[0]:.2f}, {clicked_goal[1]:.2f})")

        lidar_scan = env.get_lidar_scan()
        points = neupan_planner.scan_to_point(robot_state, lidar_scan)

        action, info = neupan_planner(robot_state, points)

        if info['stop']:
            print('NeuPAN stops: min distance', flush=True)

        if info['arrive']:
            print('Arrived! Right-click to set new goal.', flush=True)

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

    print("Simulation complete. Close the window to exit.")
    env.end(0)  # 0 = keep open until manually closed


if __name__ == '__main__':
    main()
