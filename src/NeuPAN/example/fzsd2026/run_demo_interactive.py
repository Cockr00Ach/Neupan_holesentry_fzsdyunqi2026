"""Interactive NeuPAN demo — right-click on map to set goal."""
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import irsim
from neupan import neupan

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

CUR_DIR = os.path.dirname(os.path.abspath(__file__))
PLANNER_FILE = os.path.join(CUR_DIR, 'planner.yaml')
ENV_FILE = os.path.join(CUR_DIR, 'env.yaml')

# Store clicked goal persistently (survives mouse release)
clicked_goal = None


def on_click(event):
    global clicked_goal
    if event.button == 3 and event.inaxes is not None:  # Right click
        clicked_goal = (event.xdata, event.ydata)


def main():
    global clicked_goal

    env = irsim.make(ENV_FILE, save_ani=False, full=False, display=True)
    neupan_planner = neupan.init_from_yaml(PLANNER_FILE)

    # Hook into matplotlib for persistent click detection
    plt.connect('button_press_event', on_click)

    print("=" * 50)
    print("Interactive NeuPAN Demo")
    print("Right click on map → SET NEW GOAL")
    print("=" * 50)

    last_goal = None

    for i in range(5000):
        robot_state = env.get_robot_state()

        # Check for new right-click goal
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
