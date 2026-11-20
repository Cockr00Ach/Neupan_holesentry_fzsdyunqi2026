#!/usr/bin/env python3
"""ROS 2 bridge between the NeuPAN 3D planner and the fzsd2026 sentry robot.

The sentry is modelled as a 3D box (see ``planner.yaml``: ``length/width/height/z_robot``),
so the planner expects obstacle points in a *ground-referenced* frame:

    z = 0      -> the floor the robot drives on
    z = z_robot -> centre of the robot box

This node

* reads the robot pose from the chassis odometry,
* converts the 3D lidar points into that ground-referenced map frame,
* runs NeuPAN at a fixed rate and forwards the velocity command to the chassis,
* broadcasts ``map -> odom`` and ``odom -> base_footprint`` so RViz can show the scene,
* (optionally) builds the reference path with the A* global planner on a static map,
* publishes the planner internals (paths, DUNE/NRMP points, filtered cloud, goal).

Subscribed
    ~/odom_topic          nav_msgs/Odometry        robot pose (default <ns>/chassis_odometry_gt)
    ~/points_topic        sensor_msgs/PointCloud2 3D lidar (default <ns>/livox/lidar)
    ~/goal_topic          geometry_msgs/PoseStamped RViz "2D Goal Pose" (default /goal_pose)
    ~/initial_path_topic  nav_msgs/Path            external reference path (default /initial_path)

Published
    <ns>/robot_base/chassis_cmd   rmoss_interfaces/ChassisCmd  chassis velocity command
    /neupan_plan                  nav_msgs/Path   optimised trajectory (receding horizon)
    /neupan_ref_state             nav_msgs/Path   reference states
    /neupan_initial_path          nav_msgs/Path   initial/reference global path
    /neupan_global_path           nav_msgs/Path   A* path (only when the global planner is on)
    /neupan_obstacles             sensor_msgs/PointCloud2  filtered obstacle points (map frame)
    /neupan_goal                  geometry_msgs/PoseStamped current goal
    /dune_point_markers           visualization_msgs/MarkerArray
    /nrmp_point_markers           visualization_msgs/MarkerArray
    /robot_marker                 visualization_msgs/Marker  robot box
    /tf, /tf_static               TF tree map -> odom -> base_footprint

Run it with::

    ros2 run  # (standalone script)
    python3 example/fzsd2026/neupan_ros2_bridge.py --ros-args \
        --params-file example/fzsd2026/bridge_params.yaml
"""

import math
import os
import sys
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

try:  # Humble and newer expose point_cloud2 through sensor_msgs_py
    from sensor_msgs_py import point_cloud2 as pc2
except ImportError:  # pragma: no cover - ROS 1 style layout / older builds
    from sensor_msgs import point_cloud2 as pc2

from rmoss_interfaces.msg import ChassisCmd

CUR_DIR = os.path.dirname(os.path.abspath(__file__))
NEUPAN_DIR = os.path.normpath(os.path.join(CUR_DIR, '..', '..'))
sys.path.insert(0, NEUPAN_DIR)
sys.path.insert(0, CUR_DIR)

from neupan import neupan  # noqa: E402

try:
    from global_planner import GlobalPlanner
except ImportError:  # pragma: no cover - PIL/scipy missing
    GlobalPlanner = None

try:
    from local_grid import LocalGrid
except ImportError:  # pragma: no cover
    LocalGrid = None


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def quat_to_yaw(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.z * q.z + q.y * q.y))


def resolve_path(path: str, base_dir: str) -> str:
    """Resolve ``path`` against the cwd, then against ``base_dir``."""
    if os.path.isabs(path) or not path:
        return path
    if os.path.exists(path):
        return os.path.abspath(path)
    candidate = os.path.join(base_dir, path)
    if os.path.exists(candidate):
        return candidate
    return os.path.abspath(path)


class NeuPANBridge(Node):
    def __init__(self):
        super().__init__('neupan_bridge')

        # ----------------------------- parameters ----------------------------- #
        self.declare_parameter('robot_ns', 'red_standard_robot1')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('planner_config', '')
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('publish_visualization', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('relay_robot_tf', True)

        # sensor / point-cloud preprocessing
        self.declare_parameter('odom_topic', '')
        self.declare_parameter('points_topic', '')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('initial_path_topic', '/initial_path')
        self.declare_parameter('lidar_height', 0.0)      # 0 -> auto-detect from the cloud
        self.declare_parameter('ground_clearance', 0.06)  # drop points lower than this
        self.declare_parameter('max_points', 600)
        self.declare_parameter('self_filter_margin', 0.06)
        self.declare_parameter('max_range', 12.0)

        # local occupancy grid used to plan the reference path (A* on live lidar)
        self.declare_parameter('use_local_grid', True)   # rolling grid (clearance/detours)
        self.declare_parameter('use_local_planner', False)  # A* reference path on that grid
        self.declare_parameter('local_grid_size', [40.0, 40.0])
        self.declare_parameter('local_grid_resolution', 0.1)
        self.declare_parameter('local_grid_decay', 6.0)
        self.declare_parameter('local_inflation', 0.5)
        self.declare_parameter('local_lookahead', 4.0)
        self.declare_parameter('replan_interval', 1.0)

        # stuck detection / recovery
        self.declare_parameter('use_startup_waypoints', True)
        self.declare_parameter('goal_tolerance', 0.35)
        self.declare_parameter('max_detours', 4)
        self.declare_parameter('detour_step', 0.9)
        self.declare_parameter('stuck_time', 1.0)
        self.declare_parameter('stuck_distance', 0.05)

        # global planner (A*) on a static map
        self.declare_parameter('use_global_planner', False)
        self.declare_parameter('global_map', '')
        self.declare_parameter('inflation_radius', 0.45)
        self.declare_parameter('map_waypoint_interval', 0.35)

        # start-up behaviour
        self.declare_parameter('startup_goal', [-99.0, -99.0, 0.0])  # sentinel -> use yaml waypoints
        self.declare_parameter('max_linear_speed', 0.0)   # 0 -> planner value
        self.declare_parameter('max_angular_speed', 0.0)

        self.robot_ns = str(self.get_parameter('robot_ns').value).strip('/')
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.vis = bool(self.get_parameter('publish_visualization').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        planner_config = str(self.get_parameter('planner_config').value) or os.path.join(
            CUR_DIR, 'planner.yaml')
        planner_config = resolve_path(planner_config, CUR_DIR)

        odom_topic = str(self.get_parameter('odom_topic').value) or \
            f'/{self.robot_ns}/chassis_odometry_gt'
        points_topic = str(self.get_parameter('points_topic').value) or \
            f'/{self.robot_ns}/livox/lidar'
        goal_topic = str(self.get_parameter('goal_topic').value)
        initial_path_topic = str(self.get_parameter('initial_path_topic').value)

        self.lidar_height = float(self.get_parameter('lidar_height').value)
        self.ground_clearance = float(self.get_parameter('ground_clearance').value)
        self.max_points = int(self.get_parameter('max_points').value)
        self.self_margin = float(self.get_parameter('self_filter_margin').value)
        self.max_range = float(self.get_parameter('max_range').value)

        # ----------------------------- planner -------------------------------- #
        self.get_logger().info(f'loading NeuPAN config: {planner_config}')
        self.planner = neupan.init_from_yaml(planner_config)
        self.omni = self.planner.robot.kinematics == 'omni'
        self.box_length = float(getattr(self.planner.robot, 'length', 0.6) or 0.6)
        self.box_width = float(getattr(self.planner.robot, 'width', 0.5) or 0.5)
        self.box_height = float(getattr(self.planner.robot, 'height', 0.5) or 0.5)
        self.z_robot = float(getattr(self.planner.robot, 'z_robot', self.box_height / 2.0))

        max_linear = float(self.get_parameter('max_linear_speed').value)
        max_angular = float(self.get_parameter('max_angular_speed').value)
        speed_bound = np.asarray(self.planner.robot.max_speed).reshape(-1)
        self.max_linear = max_linear if max_linear > 0 else float(speed_bound[0])
        self.max_angular = max_angular if max_angular > 0 else float(speed_bound[-1])

        self.global_planner = None
        if bool(self.get_parameter('use_global_planner').value):
            map_path = resolve_path(str(self.get_parameter('global_map').value), CUR_DIR)
            if GlobalPlanner is None:
                self.get_logger().warn('use_global_planner=true but PIL/scipy are missing; '
                                       'falling back to straight reference paths')
            elif not map_path or not os.path.exists(map_path):
                self.get_logger().warn(f'global map not found: {map_path!r}; '
                                       'falling back to straight reference paths')
            else:
                self.global_planner = GlobalPlanner(
                    map_path,
                    inflation_radius=float(self.get_parameter('inflation_radius').value),
                    downsample_interval=float(
                        self.get_parameter('map_waypoint_interval').value))
                self.get_logger().info(
                    f'global planner ready: {map_path} '
                    f'({self.global_planner.width}x{self.global_planner.height} cells, '
                    f'{self.global_planner.resolution} m)')

        self.local_grid = None
        self.use_local_planner = bool(self.get_parameter('use_local_planner').value)
        self.use_local_grid = bool(self.get_parameter('use_local_grid').value) or \
            self.use_local_planner
        self.local_inflation = float(self.get_parameter('local_inflation').value)
        self.local_lookahead = float(self.get_parameter('local_lookahead').value)
        self.replan_interval = float(self.get_parameter('replan_interval').value)
        self.use_startup_waypoints = bool(self.get_parameter('use_startup_waypoints').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.stuck_time = float(self.get_parameter('stuck_time').value)
        self.stuck_distance = float(self.get_parameter('stuck_distance').value)
        if self.use_local_grid and LocalGrid is None:
            self.get_logger().warn('local occupancy grid unavailable (scipy missing?); '
                                   'reference paths will be straight lines')
            self.use_local_planner = False
            self.use_local_grid = False

        # ------------------------------ state --------------------------------- #
        self.robot_state = None      # np (3, 1): x, y, yaw in the map frame
        self.robot_odom = None       # np (3, 1): x, y, yaw in the odom frame
        self.obstacle_points = None  # np (3, N): ground-referenced map frame
        self.goal = None             # np (3, 1) in the map frame
        self.arrived = False
        self._last_action = (0.0, 0.0)
        self._lidar_height_est = None
        self._pose_history = deque()
        self._last_replan = 0.0
        self._last_stuck_check = 0.0
        self._stuck_since = None
        self._got_goal = False
        self._detour_attempts = 0
        self._stop_since = None
        self.failed = False
        self._clearance_cache = None
        self.max_detours = int(self.get_parameter('max_detours').value)
        self.detour_step = float(self.get_parameter('detour_step').value)

        self._last_log = self.get_clock().now()

        # --------------------------- interfaces ------------------------------- #
        cloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        self.create_subscription(PointCloud2, points_topic, self.pointcloud_callback, cloud_qos)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.create_subscription(PoseStamped, goal_topic, self.goal_callback, 10)
        self.create_subscription(Path, initial_path_topic, self.initial_path_callback, 10)

        self.cmd_pub = self.create_publisher(
            ChassisCmd, f'/{self.robot_ns}/robot_base/chassis_cmd', 10)

        if self.vis:
            self.plan_pub = self.create_publisher(Path, '/neupan_plan', 10)
            self.ref_state_pub = self.create_publisher(Path, '/neupan_ref_state', 10)
            self.initial_path_pub = self.create_publisher(Path, '/neupan_initial_path', 10)
            self.global_path_pub = self.create_publisher(Path, '/neupan_global_path', 10)
            self.goal_pub = self.create_publisher(PoseStamped, '/neupan_goal', 10)
            self.cloud_pub = self.create_publisher(PointCloud2, '/neupan_obstacles', 10)
            self.dune_marker_pub = self.create_publisher(MarkerArray, '/dune_point_markers', 10)
            self.nrmp_marker_pub = self.create_publisher(MarkerArray, '/nrmp_point_markers', 10)
            self.robot_marker_pub = self.create_publisher(Marker, '/robot_marker', 10)

        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
            self.tf_static_broadcaster = StaticTransformBroadcaster(self)
            self._broadcast_static_tf()
            # The Gazebo stack publishes the robot TF on the namespaced topics
            # (/<ns>/tf, /<ns>/tf_static); RViz listens on the global ones, so relay
            # them to complete the tree map -> odom -> base_footprint -> links.
            if bool(self.get_parameter('relay_robot_tf').value):
                self._tf_relay_pub = self.create_publisher(TFMessage, '/tf', 100)
                # /tf_static is latched: tf2 listeners subscribe with TRANSIENT_LOCAL
                static_qos = QoSProfile(
                    depth=100,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=ReliabilityPolicy.RELIABLE,
                )
                self._tf_static_relay_pub = self.create_publisher(
                    TFMessage, '/tf_static', static_qos)
                self.create_subscription(TFMessage, f'/{self.robot_ns}/tf',
                                         self._relay_tf, 100)
                # the namespaced tf_static topic is latched: match its durability or the
                # already published transforms are never delivered
                self.create_subscription(TFMessage, f'/{self.robot_ns}/tf_static',
                                         self._relay_static_tf, static_qos)

        period = 1.0 / max(1.0, float(self.get_parameter('control_rate').value))
        self.timer = self.create_timer(period, self.control_loop)

        # ---------------------------- start-up -------------------------------- #
        if len(self.planner.waypoints) > 0:
            self.get_logger().info(
                f'planner has {len(self.planner.waypoints)} initial waypoints; '
                'the reference path will start at the robot pose')
        else:
            self.get_logger().info('no initial waypoints; waiting for a goal')

        startup_goal = list(self.get_parameter('startup_goal').value)
        self.startup_goal = None
        if len(startup_goal) == 3 and startup_goal[0] > -90.0:
            self.startup_goal = np.array(startup_goal, dtype=float).reshape(3, 1)

        self.get_logger().info(
            f'NeuPAN bridge ready: omni={self.omni} box=({self.box_length:.2f}x'
            f'{self.box_width:.2f}x{self.box_height:.2f}) z_robot={self.z_robot:.2f} '
            f'lidar_height={"auto" if self.lidar_height <= 0 else self.lidar_height}')

    # ------------------------------------------------------------------ #
    # TF
    # ------------------------------------------------------------------ #
    def _broadcast_static_tf(self):
        """map -> odom: identity, i.e. the odometry already lives in the map frame."""
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.odom_frame
        tf.transform.rotation.w = 1.0
        self.tf_static_broadcaster.sendTransform(tf)

    def _relay_tf(self, msg: TFMessage):
        self._tf_relay_pub.publish(msg)

    def _relay_static_tf(self, msg: TFMessage):
        self._tf_static_relay_pub.publish(msg)

    def _broadcast_robot_tf(self):
        if self.robot_odom is None or not self.publish_tf:
            return
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = float(self.robot_odom[0, 0])
        tf.transform.translation.y = float(self.robot_odom[1, 0])
        tf.transform.translation.z = 0.0
        tf.transform.rotation = yaw_to_quat(float(self.robot_odom[2, 0]))
        self.tf_broadcaster.sendTransform(tf)

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #
    def odom_callback(self, msg: Odometry):
        pose = msg.pose.pose
        yaw = quat_to_yaw(pose.orientation)
        # map -> odom is identity for now (the Gazebo odometry is already world based);
        # a real localization stack would publish map -> odom itself.
        self.robot_odom = np.array([pose.position.x, pose.position.y, yaw]).reshape(3, 1)
        self.robot_state = self.robot_odom.copy()
        self._pose_history.append((time.time(), pose.position.x, pose.position.y))

    def goal_callback(self, msg: PoseStamped):
        if self.robot_state is None:
            self.get_logger().warn('no robot state yet, ignoring goal')
            return
        goal = np.array([[msg.pose.position.x],
                         [msg.pose.position.y],
                         [quat_to_yaw(msg.pose.orientation)]]).reshape(3, 1)
        self.set_goal(goal)

    def initial_path_callback(self, msg: Path):
        if len(msg.poses) < 2:
            return
        waypoints = [np.array([[p.pose.position.x],
                               [p.pose.position.y],
                               [quat_to_yaw(p.pose.orientation)]]).reshape(3, 1)
                     for p in msg.poses]
        self.planner.update_initial_path_from_waypoints(waypoints)
        self.planner.reset()
        self.arrived = False
        self.get_logger().info(f'reference path replaced with {len(waypoints)} waypoints')
        if len(waypoints) > 0:
            self.goal = waypoints[-1]

    def set_goal(self, goal: np.ndarray):
        """Set a new goal and build the reference path towards it."""
        self.goal = goal
        self.arrived = False
        self._got_goal = True
        self.failed = False
        self._detour_attempts = 0
        self._reference_goal = goal.copy()
        self.get_logger().info(
            f'new goal ({goal[0, 0]:.2f}, {goal[1, 0]:.2f}, {goal[2, 0]:.2f})')
        self._update_reference_path(reason='new goal', force=True)
        self.planner.reset()

    def _local_plan(self, start_xy, goal_xy):
        """Plan a short feasible reference segment towards ``goal_xy``.

        A full A* towards a far goal can pick arbitrarily large detours through
        unobserved space, so the local planner only looks ``local_lookahead`` metres
        ahead: it puts a carrot on the goal bearing and searches around it (like a
        gap search) for the closest direction that is actually free.  The result is a
        short path that NeuPAN can track; it is refreshed while the robot drives.
        """
        if self.local_grid is None:
            return None
        dx, dy = goal_xy[0] - start_xy[0], goal_xy[1] - start_xy[1]
        distance = math.hypot(dx, dy)
        if distance < 1e-3:
            return [list(start_xy) + [0.0]]
        bearing = math.atan2(dy, dx)
        lookahead = min(self.local_lookahead, distance)
        final_segment = distance <= self.local_lookahead

        for offset in (0.0, 0.35, -0.35, 0.7, -0.7, 1.05, -1.05, 1.4, -1.4,
                       1.75, -1.75, 2.1, -2.1, 2.45, -2.45, 2.8, -2.8):
            if final_segment and offset != 0.0:
                continue
            target = (start_xy[0] + lookahead * math.cos(bearing + offset),
                      start_xy[1] + lookahead * math.sin(bearing + offset)) \
                if not final_segment else goal_xy
            path = self.local_grid.plan(start_xy, target, self.local_inflation)
            if path:
                return path
        return None

    def _build_waypoints(self, goal: np.ndarray):
        """Return (waypoints, source) for the reference path towards ``goal``."""
        start = self.robot_state
        start_xy = (float(start[0, 0]), float(start[1, 0]))
        goal_xy = (float(goal[0, 0]), float(goal[1, 0]))

        path = None
        source = 'straight line'
        if self.use_local_planner and self.local_grid is not None:
            path = self._local_plan(start_xy, goal_xy)
            source = 'local A*'
        if path is None and self.global_planner is not None:
            path = self.global_planner.plan(start_xy, goal_xy)
            source = 'global A*'
            if path:
                path = list(path) + [list(goal_xy) + [0.0]]

        if path:
            waypoints = [np.array([[x], [y], [yaw]]).reshape(3, 1) for x, y, yaw in path]
            # the last waypoint of a local plan is a carrot, not the goal
            if source == 'local A*' and math.hypot(
                    waypoints[-1][0, 0] - goal_xy[0],
                    waypoints[-1][1, 0] - goal_xy[1]) > 0.05:
                pass
            return waypoints, source
        return None, source

    def _update_reference_path(self, reason='', force=False):
        """(Re)build the reference path, keeping the previous one when it still works."""
        if self.goal is None or self.robot_state is None:
            return False
        now = time.time()
        if not force and (now - self._last_replan) < self.replan_interval:
            return False
        self._last_replan = now

        goal = self.goal
        waypoints, source = self._build_waypoints(goal)
        if waypoints is None:
            if force:
                self.planner.update_initial_path_from_goal(self.robot_state, goal)
                self.get_logger().info(
                    f'no A* path found ({reason}); using a straight reference path')
            else:
                self.get_logger().warn(
                    f'no A* path found ({reason}); keeping the current reference path',
                    throttle_duration_sec=2.0)
            return False

        self.planner.update_initial_path_from_waypoints(waypoints)
        self.planner.reset()
        self.arrived = False
        if self.vis:
            self._publish_path(self.global_path_pub, waypoints)
        self.get_logger().info(
            f'reference path rebuilt ({reason}, {source}): {len(waypoints)} waypoints')
        return True

    def _reference_blocked(self):
        """True if an obstacle is now observed on the reference path ahead."""
        if self.local_grid is None or self.planner.initial_path is None:
            return False
        path = self.planner.initial_path
        start_idx = getattr(self.planner.ipath, 'point_index', 0)
        # only look a few metres ahead: near points are handled by NeuPAN itself
        for pt in path[start_idx:start_idx + 80]:
            if math.hypot(pt[0, 0] - self.robot_state[0, 0],
                          pt[1, 0] - self.robot_state[1, 0]) > 6.0:
                break
            if not self.local_grid.is_free((float(pt[0, 0]), float(pt[1, 0])),
                                           self.local_inflation):
                return True
        return False

    def _check_stuck(self, now):
        """Detect "commanded to move but not moving" and replan around the blockage."""
        if len(self._pose_history) < 2:
            return
        while self._pose_history and (now - self._pose_history[0][0]) > self.stuck_time:
            self._pose_history.popleft()
        if not self._pose_history or (now - self._last_stuck_check) < 0.2:
            return
        self._last_stuck_check = now
        commanded = math.hypot(*self._last_action)
        oldest = self._pose_history[0]
        moved = math.hypot(self.robot_state[0, 0] - oldest[1],
                           self.robot_state[1, 0] - oldest[2])
        if commanded > 0.25 and moved < self.stuck_distance:
            if self._stuck_since is None:
                self._stuck_since = now
            elif (now - self._stuck_since) > self.stuck_time:
                self.get_logger().warn(
                    f'robot is blocked (commanded {commanded:.2f} m/s, moved {moved:.3f} m '
                    f'in {self.stuck_time:.1f} s)')
                self._stuck_since = None
                if not self._try_detour():
                    self.failed = True
                    self.get_logger().warn(
                        'no way around the blockage left; stopping (send a new goal to '
                        'retry)', throttle_duration_sec=2.0)
                    self._publish_stop()
        else:
            self._stuck_since = None

    def _side_score(self, start_xy, target_xy):
        """Mean obstacle clearance along a segment (1.0 when there is no local map)."""
        if self.local_grid is None:
            return 1.0
        clearance = self._clearance()
        dx, dy = target_xy[0] - start_xy[0], target_xy[1] - start_xy[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            return 1.0
        steps = max(4, int(dist / self.local_grid.resolution))
        total = 0.0
        for i in range(steps + 1):
            t = i / steps
            gx, gy = self.local_grid.world_to_grid(
                (start_xy[0] + t * dx, start_xy[1] + t * dy))
            if not self.local_grid.in_bounds(gx, gy):
                total += 0.0
                continue
            total += float(clearance[gy, gx])
        return total / (steps + 1)

    def _clearance(self):
        if getattr(self, '_clearance_cache', None) is None or \
                (time.time() - self._clearance_cache[1]) > 0.5:
            self._clearance_cache = (self.local_grid.clearance(), time.time())
        return self._clearance_cache[0]

    def _try_detour(self):
        """Escape a blockage by inserting a lateral intermediate waypoint.

        NeuPAN only tracks the reference path, so when the direct path is physically
        blocked the cheapest robust fix is to bend the reference sideways.  The side
        is chosen with the rolling occupancy grid (more free cells wins) and the
        lateral offset grows with every attempt.  Gives up after ``max_detours``.
        """
        if self.goal is None:
            return False
        if self._detour_attempts >= self.max_detours:
            return False
        start = (float(self.robot_state[0, 0]), float(self.robot_state[1, 0]))
        goal = (float(self.goal[0, 0]), float(self.goal[1, 0]))
        dx, dy = goal[0] - start[0], goal[1] - start[1]
        distance = math.hypot(dx, dy)
        if distance < 0.6:
            return False
        ux, uy = dx / distance, dy / distance
        reach = min(3.0, 0.6 * distance)
        mid = (start[0] + ux * reach, start[1] + uy * reach)
        self._detour_attempts += 1
        lateral = self.detour_step * self._detour_attempts

        best = None
        for side in (1.0, -1.0):
            target = (mid[0] - uy * lateral * side, mid[1] + ux * lateral * side)
            score = self._side_score(start, target)
            if best is None or score > best[0]:
                best = (score, target, side)
        score, target, side = best

        waypoints = [
            np.array([[start[0]], [start[1]], [self.robot_state[2, 0]]]).reshape(3, 1),
            np.array([[target[0]], [target[1]], [0.0]]).reshape(3, 1),
            self.goal.copy(),
        ]
        self.planner.update_initial_path_from_waypoints(waypoints)
        self.planner.reset()
        self.arrived = False
        if self.vis:
            self._publish_path(self.global_path_pub, waypoints)
        self.get_logger().info(
            f'detour #{self._detour_attempts}: lateral {lateral:.1f} m to the '
            f'{"left" if side > 0 else "right"} (clearance score {score:.2f})')
        return True

    # ------------------------------------------------------------------ #
    # Sensor processing
    # ------------------------------------------------------------------ #
    def _read_points(self, msg: PointCloud2):
        """Return the xyz points of a PointCloud2 as a (3, N) float array."""
        try:
            raw = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            if isinstance(raw, np.ndarray) and raw.dtype.names:
                pts = np.stack([raw['x'], raw['y'], raw['z']], axis=1)
            else:
                pts = np.asarray(raw, dtype=np.float64).reshape(-1, 3)
        except Exception as exc:  # pragma: no cover - malformed cloud
            self.get_logger().warn(f'failed to read point cloud: {exc}',
                                   throttle_duration_sec=5.0)
            return None
        if pts.size == 0:
            return None
        pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.size == 0:
            return None
        return pts.T.astype(np.float64)

    def pointcloud_callback(self, msg: PointCloud2):
        if self.robot_state is None:
            return
        points = self._read_points(msg)
        if points is None:
            self.obstacle_points = None
            return

        # 1. sensor height above the floor (auto-calibrated once, then smoothed)
        if self.lidar_height <= 0.0:
            floor_local = float(np.percentile(points[2, :], 0.5))
            est = -floor_local
            if 0.05 < est < 2.0:
                if self._lidar_height_est is None:
                    self._lidar_height_est = est
                else:
                    self._lidar_height_est = 0.95 * self._lidar_height_est + 0.05 * est
                self.lidar_height = self._lidar_height_est
            else:
                self.get_logger().warn(
                    f'cannot auto-detect the lidar height (candidate {est:.2f} m); '
                    'set the lidar_height parameter', throttle_duration_sec=5.0)
                return

        # 2. ground-referenced sensor frame -> ground-referenced map frame
        ranges = np.linalg.norm(points[0:2, :], axis=0)
        height = self.lidar_height + points[2, :]

        x, y, yaw = (float(self.robot_state[0, 0]), float(self.robot_state[1, 0]),
                     float(self.robot_state[2, 0]))
        c, s = math.cos(yaw), math.sin(yaw)
        px = c * points[0, :] - s * points[1, :] + x
        py = s * points[0, :] + c * points[1, :] + y

        keep = (height >= self.ground_clearance) & (ranges <= self.max_range)

        # 3. drop the robot's own body: anything inside the footprint slightly inflated
        #    and below the box top can only be a self-hit
        half_l = self.box_length / 2.0 + self.self_margin
        half_w = self.box_width / 2.0 + self.self_margin
        in_box = ((np.abs(px - x) <= half_l) & (np.abs(py - y) <= half_w)
                  & (height <= self.box_height + 0.15))
        keep &= ~in_box

        if keep.sum() < 5:
            self.obstacle_points = None
            return

        cloud = np.vstack([px[keep], py[keep], height[keep]])

        # 4. decimate with a fixed stride so the DUNE input stays bounded
        if cloud.shape[1] > self.max_points:
            stride = int(math.ceil(cloud.shape[1] / self.max_points))
            cloud = cloud[:, ::stride]

        self.obstacle_points = cloud
        self._update_local_grid(cloud)

    def _update_local_grid(self, cloud):
        """Feed the rolling occupancy grid with the latest (ground-referenced) points."""
        if not self.use_local_grid or cloud is None:
            return
        center = (float(self.robot_state[0, 0]), float(self.robot_state[1, 0]))
        if self.local_grid is None:
            size = list(self.get_parameter('local_grid_size').value) or [40.0, 40.0]
            self.local_grid = LocalGrid(
                resolution=float(self.get_parameter('local_grid_resolution').value),
                size=tuple(size),
                center=center,
                decay=float(self.get_parameter('local_grid_decay').value),
            )
            self.get_logger().info(
                f'local occupancy grid: {self.local_grid.width}x{self.local_grid.height} '
                f'cells at {self.local_grid.resolution} m around {center}')
        else:
            # re-center the rolling window when the robot approaches its border
            half_w = self.local_grid.width * self.local_grid.resolution / 2.0
            half_h = self.local_grid.height * self.local_grid.resolution / 2.0
            cx = self.local_grid.origin[0] + half_w
            cy = self.local_grid.origin[1] + half_h
            if (abs(center[0] - cx) > half_w * 0.6
                    or abs(center[1] - cy) > half_h * 0.6):
                self.local_grid = LocalGrid(
                    resolution=self.local_grid.resolution,
                    size=(2 * half_w, 2 * half_h),
                    center=center,
                    decay=self.local_grid.decay,
                    min_height=self.local_grid.min_height,
                    max_height=self.local_grid.max_height,
                )
        self.local_grid.update(cloud, stamp=time.time())

    # ------------------------------------------------------------------ #
    # Control loop
    # ------------------------------------------------------------------ #
    def control_loop(self):
        if self.robot_state is None:
            self.get_logger().info('waiting for odometry...', throttle_duration_sec=2.0)
            return

        if (self.planner.initial_path is None and self.use_startup_waypoints
                and len(self.planner.waypoints) > 0):
            # generate the reference path from the current pose through the yaml waypoints
            self.planner.set_initial_path_from_state(self.robot_state)
            self.arrived = False

        if self.planner.initial_path is None:
            self.get_logger().info('waiting for a reference path (click 2D Goal Pose)...',
                                   throttle_duration_sec=5.0)
            self._broadcast_robot_tf()
            self._publish_obstacles()
            self._publish_visualization(None)
            self._publish_goal()
            return

        if self.startup_goal is not None and self.goal is None:
            self.set_goal(self.startup_goal)

        self._broadcast_robot_tf()
        self._publish_goal()

        if self.arrived or self.failed:
            self._publish_stop()
            self._publish_visualization(self.planner.info)
            return

        action, info = self.planner(self.robot_state, self.obstacle_points)
        self._publish_obstacles()

        now_ts = time.time()
        self._check_stuck(now_ts)
        if (self.use_local_planner and self._stuck_since is None
                and self._reference_blocked()):
            self._update_reference_path(reason='path blocked')

        if info['arrive']:
            distance_to_goal = float('inf')
            if self.goal is not None:
                distance_to_goal = math.hypot(self.robot_state[0, 0] - self.goal[0, 0],
                                              self.robot_state[1, 0] - self.goal[1, 0])
            if self.goal is None or distance_to_goal <= self.goal_tolerance:
                self.arrived = True
                where = ('end of the reference path' if self.goal is None
                         else f'distance {distance_to_goal:.2f} m')
                self.get_logger().info(f'>>> arrived ({where})', throttle_duration_sec=1.0)
                self._publish_stop()
            else:
                # end of the current reference segment (local carrot): keep going
                self.get_logger().info(
                    f'reference segment finished, {distance_to_goal:.2f} m to the goal',
                    throttle_duration_sec=1.0)
                self._update_reference_path(reason='segment finished', force=True)
                self._publish_action(action)
        elif info['stop']:
            self.get_logger().warn(
                f'stop: min_dist={float(self.planner.min_distance):.3f} < '
                f'threshold={self.planner.collision_threshold:.3f}',
                throttle_duration_sec=1.0)
            self._publish_stop()
            # a safety stop next to an obstacle is not a dead end: bend the reference
            # path away and let NeuPAN back out of the corner
            if self._stop_since is None:
                self._stop_since = time.time()
            elif (time.time() - self._stop_since) > 0.5:
                self._stop_since = None
                if self.goal is not None:
                    if not self._try_detour():
                        self.failed = True
                        self.get_logger().warn(
                            'still in collision after the detours; stopping (send a new '
                            'goal to retry)', throttle_duration_sec=2.0)
        else:
            self._stop_since = None
            self._publish_action(action)

        self._publish_visualization(info)

        now = self.get_clock().now()
        if (now - self._last_log).nanoseconds > 2e9:
            self._last_log = now
            self.get_logger().info(
                f'pose=({self.robot_state[0, 0]:.2f}, {self.robot_state[1, 0]:.2f}, '
                f'{self.robot_state[2, 0]:.2f}) '
                f'min_dist={float(self.planner.min_distance):.3f} '
                f'points={0 if self.obstacle_points is None else self.obstacle_points.shape[1]} '
                f'lidar_h={self.lidar_height:.2f} '
                f'v=({self._last_action[0]:.2f}, {self._last_action[1]:.2f})')

    def _publish_action(self, action: np.ndarray):
        cmd = ChassisCmd()
        cmd.type = ChassisCmd.VELOCITY
        if self.omni:
            vx = float(np.clip(action[0, 0], -self.max_linear, self.max_linear))
            vy = float(np.clip(action[1, 0], -self.max_linear, self.max_linear))
            cmd.twist.linear.x = vx
            cmd.twist.linear.y = vy
            self._last_action = (vx, vy)
        else:
            vx = float(np.clip(action[0, 0], -self.max_linear, self.max_linear))
            wz = float(np.clip(action[1, 0], -self.max_angular, self.max_angular))
            cmd.twist.linear.x = vx
            cmd.twist.angular.z = wz
            self._last_action = (vx, wz)
        self.cmd_pub.publish(cmd)

    def _publish_stop(self):
        cmd = ChassisCmd()
        cmd.type = ChassisCmd.VELOCITY
        self._last_action = (0.0, 0.0)
        self.cmd_pub.publish(cmd)

    # ------------------------------------------------------------------ #
    # Visualization helpers
    # ------------------------------------------------------------------ #
    def _publish_visualization(self, info):
        """Publish everything RViz needs; also runs while parked (arrived/failed)."""
        if not self.vis:
            return
        if info:
            self._publish_path(self.plan_pub, info.get('opt_state_list'))
            self._publish_path(self.ref_state_pub, info.get('ref_state_list'))
        self._publish_initial_path()
        self._publish_markers(self.dune_marker_pub, self.planner.dune_points,
                              (0.63, 0.13, 0.94), 0.05)
        self._publish_markers(self.nrmp_marker_pub, self.planner.nrmp_points,
                              (1.0, 0.5, 0.0), 0.03)
        self._publish_robot_marker()

    def _publish_path(self, pub, state_list):
        if not state_list:
            return
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for state in state_list:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(state[0, 0])
            ps.pose.position.y = float(state[1, 0])
            ps.pose.orientation = yaw_to_quat(float(state[2, 0]) if state.shape[0] > 2 else 0.0)
            path.poses.append(ps)
        pub.publish(path)

    def _publish_initial_path(self):
        if self.planner.initial_path is None:
            return
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for pt in self.planner.initial_path:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(pt[0, 0])
            ps.pose.position.y = float(pt[1, 0])
            ps.pose.orientation = yaw_to_quat(float(pt[2, 0]) if len(pt) > 2 else 0.0)
            path.poses.append(ps)
        self.initial_path_pub.publish(path)

    def _publish_goal(self):
        if not self.vis or self.goal is None:
            return
        ps = PoseStamped()
        ps.header.frame_id = self.map_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(self.goal[0, 0])
        ps.pose.position.y = float(self.goal[1, 0])
        ps.pose.orientation = yaw_to_quat(float(self.goal[2, 0]))
        self.goal_pub.publish(ps)

    def _publish_obstacles(self):
        if not self.vis or self.obstacle_points is None:
            return
        pts = self.obstacle_points.T
        msg = PointCloud2()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = pts.shape[0]
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * pts.shape[0]
        msg.is_dense = True
        msg.data = np.ascontiguousarray(pts.astype(np.float32)).tobytes()
        self.cloud_pub.publish(msg)

    def _publish_markers(self, pub, points, rgb, size):
        if points is None:
            return
        ma = MarkerArray()
        for idx, pt in enumerate(points.T):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = idx
            m.type = Marker.SPHERE
            m.scale.x = m.scale.y = m.scale.z = size
            m.color.r, m.color.g, m.color.b = rgb
            m.color.a = 1.0
            m.pose.position.x = float(pt[0])
            m.pose.position.y = float(pt[1])
            m.pose.position.z = float(pt[2]) if len(pt) > 2 else 0.3
            m.pose.orientation.w = 1.0
            ma.markers.append(m)
        pub.publish(ma)

    def _publish_robot_marker(self):
        if self.robot_state is None:
            return
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.id = 0
        m.type = Marker.CUBE
        m.scale.x = self.box_length
        m.scale.y = self.box_width
        m.scale.z = self.box_height
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.4, 0.45
        m.pose.position.x = float(self.robot_state[0, 0])
        m.pose.position.y = float(self.robot_state[1, 0])
        m.pose.position.z = self.z_robot
        m.pose.orientation = yaw_to_quat(float(self.robot_state[2, 0]))
        self.robot_marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = NeuPANBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
