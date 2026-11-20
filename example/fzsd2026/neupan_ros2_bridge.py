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
    /neupan_status                std_msgs/String  JSON summary for the web UI (5 Hz)
    /tf, /tf_static               TF tree map -> odom -> base_footprint

Subscribed (control)
    /neupan_cancel                std_msgs/Empty   stop and clear the goal (web "stop")
    /neupan_set_params            std_msgs/String  live-tune q_s/p_u/eta/d_max/d_min/ref_speed
                                                   (JSON, {"reset": true} restores the yaml)

Run it with::

    ros2 run  # (standalone script)
    python3 example/fzsd2026/neupan_ros2_bridge.py --ros-args \
        --params-file example/fzsd2026/bridge_params.yaml
"""

import json
import math
import os
import sys
import time

import yaml
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
from std_msgs.msg import Empty, Header, String
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


def decode_xyz(msg: PointCloud2):
    """Fast PointCloud2 -> (N, 3) float32 array of the x/y/z fields (or None)."""
    n = int(msg.width) * int(msg.height)
    if n <= 0 or msg.point_step <= 0 or not msg.data:
        return None
    fields = {f.name: f for f in msg.fields}
    if not {'x', 'y', 'z'} <= fields.keys():
        return None
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if raw.size < n * msg.point_step:
        n = raw.size // msg.point_step
        if n <= 0:
            return None
        raw = raw[:n * msg.point_step]
    rows = raw.reshape(n, msg.point_step)
    xyz = np.empty((n, 3), dtype=np.float32)
    for i, name in enumerate(('x', 'y', 'z')):
        off = fields[name].offset
        xyz[:, i] = np.frombuffer(rows[:, off:off + 4].tobytes(), dtype=np.float32)
    finite = np.isfinite(xyz).all(axis=1)
    return xyz[finite]


def _path_length(path) -> float:
    """Planar length of a list of [x, y, ...] waypoints."""
    total = 0.0
    for a, b in zip(path[:-1], path[1:]):
        total += math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
    return total


def scalar(value) -> float:
    """float() for tensors (without the requires_grad warning) and plain numbers."""
    if hasattr(value, 'item'):
        try:
            return float(value.item())
        except (ValueError, RuntimeError):
            pass
    return float(value)


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
        self.declare_parameter('viz_rate', 15.0)      # plan/ref/markers publish rate [Hz]
        self.declare_parameter('cloud_rate', 12.0)    # filtered cloud publish rate [Hz]
        self.declare_parameter('status_rate', 20.0)   # /neupan_status rate [Hz]
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
        self.declare_parameter('self_filter_margin', 0.10)
        self.declare_parameter('max_range', 12.0)

        # local occupancy grid used to plan the reference path (A* on live lidar)
        self.declare_parameter('use_local_grid', True)   # rolling grid (clearance/detours)
        self.declare_parameter('use_local_planner', True)  # A* reference path on that grid
        self.declare_parameter('local_grid_size', [40.0, 40.0])
        self.declare_parameter('local_grid_resolution', 0.1)
        self.declare_parameter('local_grid_decay', 6.0)
        self.declare_parameter('local_inflation', 0.45)
        self.declare_parameter('local_lookahead', 4.0)
        self.declare_parameter('replan_interval', 0.8)
        self.declare_parameter('max_path_deviation', 2.0)   # re-anchor further off the path
        self.declare_parameter('max_recoveries', 8)
        self.declare_parameter('reverse_speed', 0.40)        # back-out speed [m/s]
        self.declare_parameter('speed_adapt', True)          # slow down near obstacles
        self.declare_parameter("min_ref_speed", 0.6)
        self.declare_parameter("slowdown_distance", 0.8)

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
        self.adjust_names = ('q_s', 'p_u', 'eta', 'd_max', 'd_min')
        self.adjust_defaults = {}
        self.adjust_current = {}
        self.ref_speed_default = float(self.planner.ref_speed)
        try:
            with open(planner_config, 'r') as f:
                raw_cfg = yaml.safe_load(f) or {}
            adjust_cfg = raw_cfg.get('adjust') or {}
            for name in self.adjust_names:
                if name in adjust_cfg:
                    self.adjust_defaults[name] = float(adjust_cfg[name])
                    self.adjust_current[name] = float(adjust_cfg[name])
        except (OSError, ValueError, TypeError) as exc:  # pragma: no cover
            self.get_logger().warn(f'cannot read the adjust defaults: {exc}')

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
        self.max_path_deviation = float(self.get_parameter('max_path_deviation').value)
        self.max_recoveries = int(self.get_parameter('max_recoveries').value)
        self.reverse_speed = float(self.get_parameter('reverse_speed').value)
        self.speed_adapt = bool(self.get_parameter('speed_adapt').value)
        self.min_ref_speed = float(self.get_parameter('min_ref_speed').value)
        self.slowdown_distance = float(self.get_parameter('slowdown_distance').value)
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
        self._cmd_history = deque()
        self._recoveries = 0
        self._escape = None
        self.base_ref_speed = float(getattr(self.planner, 'ref_speed', 1.5))
        self._current_ref_speed = self.base_ref_speed
        self._last_replan = 0.0
        self._last_stuck_check = 0.0
        self._stuck_since = None
        self._got_goal = False
        self._last_event = ''
        self._detour_attempts = 0
        self._stop_since = None
        self.failed = False
        self._clearance_cache = None
        self.max_detours = int(self.get_parameter('max_detours').value)
        self.detour_step = float(self.get_parameter('detour_step').value)

        self._last_log = self.get_clock().now()
        self._viz_at = 0.0
        self._cloud_at = 0.0
        self.viz_rate = max(1.0, float(self.get_parameter('viz_rate').value))
        self.cloud_rate = max(1.0, float(self.get_parameter('cloud_rate').value))
        self.status_rate = max(1.0, float(self.get_parameter('status_rate').value))

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

        self.status_pub = self.create_publisher(String, '/neupan_status', 10)
        self.create_subscription(Empty, '/neupan_cancel', self.cancel_callback, 10)
        self.create_subscription(String, '/neupan_set_params', self.set_params_callback, 10)

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
        self._status_timer = self.create_timer(1.0 / self.status_rate, self.publish_status)

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

    def cancel_callback(self, _msg: Empty):
        """Stop planning and drop the current goal (web UI "stop" button)."""
        self.goal = None
        self.arrived = True          # the control loop publishes zero velocity
        self.failed = False
        self._detour_attempts = 0
        self._stuck_since = None
        self._stop_since = None
        self._last_event = 'cancelled by user'
        self.planner.reset()
        self._publish_stop()
        self.get_logger().info('cancelled: goal cleared, robot stopped')

    def set_params_callback(self, msg: String):
        """Live tuning from the web UI: {"q_s": 4, ...} or {"reset": true}."""
        try:
            payload = json.loads(msg.data)
        except ValueError:
            self.get_logger().warn('invalid /neupan_set_params payload (expected JSON)')
            return
        if not isinstance(payload, dict):
            return
        if payload.get('reset'):
            self._apply_params(dict(self.adjust_defaults), ref_speed=self.ref_speed_default)
            return
        values = {}
        for name in self.adjust_names:
            if name in payload:
                try:
                    values[name] = float(payload[name])
                except (TypeError, ValueError):
                    continue
        ref_speed = payload.get('ref_speed')
        self._apply_params(values, ref_speed=float(ref_speed) if ref_speed is not None else None)

    def _apply_params(self, values, ref_speed=None):
        if values:
            try:
                self.planner.update_adjust_parameters(**values)
                self.adjust_current.update(values)
            except Exception as exc:  # pragma: no cover - cvxpy/torch guard
                self.get_logger().warn(f'cannot update {list(values)}: {exc}')
        if ref_speed is not None:
            self.base_ref_speed = max(0.1, float(ref_speed))
            self.planner.set_reference_speed(self.base_ref_speed)
            self._current_ref_speed = self.base_ref_speed
        shown = ', '.join(f'{k}={v:g}' for k, v in sorted(self.adjust_current.items()))
        speed = '' if ref_speed is None else f', ref_speed={self.base_ref_speed:g}'
        self._last_event = f'params updated: {shown}{speed}'
        self.get_logger().info(f'planner parameters: {shown}{speed}')

    def initial_path_callback(self, msg: Path):
        if len(msg.poses) < 2:
            return
        waypoints = [np.array([[p.pose.position.x],
                               [p.pose.position.y],
                               [quat_to_yaw(p.pose.orientation)]]).reshape(3, 1)
                     for p in msg.poses]
        if not force and self._same_direction(waypoints):
            # the replanned path leaves the same way: keep the current one to avoid the
            # reference churn that made the robot weave
            self.get_logger().info(
                f'replan ignored ({reason}): {source} path is equivalent',
                throttle_duration_sec=2.0)
            return False

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
        self._recoveries = 0
        self._escape = None
        self._reference_goal = goal.copy()
        self._last_event = f'new goal ({goal[0, 0]:.2f}, {goal[1, 0]:.2f})'
        self.get_logger().info(
            f'new goal ({goal[0, 0]:.2f}, {goal[1, 0]:.2f}, {goal[2, 0]:.2f})')
        self._update_reference_path(reason='new goal', force=True)
        self.planner.reset()

    # ------------------------------------------------------------------ #
    # Reference path: the "global" constraint, planned on the rolling grid
    # ------------------------------------------------------------------ #
    def _carrot_candidates(self, start_xy, goal_xy):
        """Targets the local A* may aim at: the goal itself when close, else carrots."""
        dx = goal_xy[0] - start_xy[0]
        dy = goal_xy[1] - start_xy[1]
        distance = math.hypot(dx, dy)
        if distance < 1e-3:
            return []
        bearing = math.atan2(dy, dx)
        if distance <= self.local_lookahead:
            return [(goal_xy, 0.0)]
        reach = self.local_lookahead
        out = []
        for offset in (0.0, 0.26, -0.26, 0.52, -0.52, 0.79, -0.79, 1.05, -1.05,
                       1.31, -1.31, 1.57, -1.57, 1.83, -1.83, 2.09, -2.09, 2.62, -2.62):
            out.append(((start_xy[0] + reach * math.cos(bearing + offset),
                         start_xy[1] + reach * math.sin(bearing + offset)), offset))
        return out

    def _local_plan(self, start_xy, goal_xy):
        """A* reference towards the goal on the rolling occupancy grid.

        The grid is inflated by ``local_inflation`` (robot radius + margin), so gaps
        narrower than the robot are *not* traversable for the planner: that is the
        global constraint that stops the robot from trying to squeeze between two
        boards.  Only ``local_lookahead`` metres are planned ahead - long plans through
        unobserved space can pick absurd detours - and the carrot is rotated when the
        direct bearing is blocked, which yields a feasible corridor around wide
        obstacles instead of a path through them.
        """
        if self.local_grid is None:
            return None
        dx = goal_xy[0] - start_xy[0]
        dy = goal_xy[1] - start_xy[1]
        distance = math.hypot(dx, dy)

        # Prefer one stable plan to the goal: replanning short carrots every second
        # makes the reference jitter and the robot zig-zags.  A plan is accepted when
        # it is not an absurd detour through unobserved space.
        path = self.local_grid.plan(start_xy, goal_xy, self.local_inflation)
        if path:
            length = _path_length(path)
            if length <= max(2.0 * distance + 2.0, distance + 4.0):
                return self._smooth_path(path)

        # Fallback: a short carrot on (or around) the direct bearing
        for target, _offset in self._carrot_candidates(start_xy, goal_xy):
            path = self.local_grid.plan(start_xy, target, self.local_inflation)
            if path:
                return self._smooth_path(path)
        return None

    def _smooth_path(self, path, iterations=2, spacing=0.35):
        """Chaikin-smooth and decimate an A* polyline so NeuPAN tracks a calm reference."""
        pts = np.array([[float(p[0]), float(p[1])] for p in path], dtype=float)
        if len(pts) < 3:
            return path
        for _ in range(iterations):
            new_pts = [pts[0]]
            for a, b in zip(pts[:-1], pts[1:]):
                new_pts.append(0.75 * a + 0.25 * b)
                new_pts.append(0.25 * a + 0.75 * b)
            new_pts.append(pts[-1])
            pts = np.array(new_pts)
        kept = [pts[0]]
        for p in pts[1:-1]:
            if np.linalg.norm(p - kept[-1]) >= spacing:
                kept.append(p)
        if np.linalg.norm(pts[-1] - kept[-1]) > 1e-3:
            kept.append(pts[-1])
        out = []
        for i, p in enumerate(kept):
            if i < len(kept) - 1:
                nxt = kept[i + 1]
                yaw = math.atan2(float(nxt[1] - p[1]), float(nxt[0] - p[0]))
            elif out:
                yaw = out[-1][2]
            else:
                yaw = 0.0
            out.append([float(p[0]), float(p[1]), yaw])
        return out

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
            # a local plan ends on a carrot: do NOT jump straight to the goal from
            # there, the segment to the goal is replanned when the carrot is reached
            return waypoints, source
        return None, source

    def _direction_at(self, points, distance=2.0):
        """Bearing from the robot to the first path point at least ``distance`` away."""
        rx, ry = float(self.robot_state[0, 0]), float(self.robot_state[1, 0])
        for p in points:
            dx, dy = float(p[0]) - rx, float(p[1]) - ry
            if math.hypot(dx, dy) >= distance:
                return math.atan2(dy, dx)
        return None

    def _same_direction(self, waypoints, tolerance_deg=20.0):
        """True when the new plan leaves in (almost) the same direction as the old one."""
        cur = self.planner.initial_path
        if cur is None or len(waypoints) < 2:
            return False
        new_bearing = self._direction_at([(w[0, 0], w[1, 0]) for w in waypoints])
        cur_bearing = self._direction_at([(p[0, 0], p[1, 0]) for p in cur])
        if new_bearing is None or cur_bearing is None:
            return False
        diff = abs(math.atan2(math.sin(new_bearing - cur_bearing),
                              math.cos(new_bearing - cur_bearing)))
        return diff < math.radians(tolerance_deg)

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
            if force and self.global_planner is None:
                self.planner.update_initial_path_from_goal(self.robot_state, goal)
                self.get_logger().info(
                    f'no A* path ({reason}); using a straight reference path')
                return True
            self.get_logger().warn(
                f'no A* path found ({reason}); keeping the current reference path',
                throttle_duration_sec=2.0)
            return False

        if not force and self._same_direction(waypoints):
            # the replanned path leaves the same way: keep the current one to avoid the
            # reference churn that made the robot weave
            self.get_logger().info(
                f'replan ignored ({reason}): {source} path is equivalent',
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
        # a slightly smaller inflation here: the path may legitimately hug an obstacle
        inflation = max(0.2, self.local_inflation * 0.75)
        for pt in path[start_idx:start_idx + 80]:
            distance = math.hypot(pt[0, 0] - self.robot_state[0, 0],
                                  pt[1, 0] - self.robot_state[1, 0])
            if distance > 5.0:
                break
            # the robot's own neighbourhood is NeuPAN's business; checking it here made
            # any path that starts next to a board look "blocked" and caused churn
            if distance < 0.8:
                continue
            if not self.local_grid.is_free((float(pt[0, 0]), float(pt[1, 0])), inflation):
                return True
        return False

    def _path_deviation(self):
        """Distance from the robot to the current reference path [m]."""
        path = self.planner.initial_path
        if path is None or self.robot_state is None:
            return 0.0
        x, y = float(self.robot_state[0, 0]), float(self.robot_state[1, 0])
        best = float('inf')
        for pt in path[::2]:
            d = math.hypot(float(pt[0, 0]) - x, float(pt[1, 0]) - y)
            if d < best:
                best = d
        return best

    # ------------------------------------------------------------------ #
    # Progress monitoring and recovery
    # ------------------------------------------------------------------ #
    def _is_oscillating(self, now):
        """True when the command keeps flipping direction without moving."""
        while self._cmd_history and (now - self._cmd_history[0][0]) > 1.5:
            self._cmd_history.popleft()
        hist = list(self._cmd_history)
        if len(hist) < 6:
            return False
        reversals = 0
        prev = None
        for _t, vx, vy in hist:
            norm = math.hypot(vx, vy)
            if norm < 0.2:
                prev = None
                continue
            cur = (vx / norm, vy / norm)
            if prev is not None and (cur[0] * prev[0] + cur[1] * prev[1]) < -0.3:
                reversals += 1
            prev = cur
        return reversals >= 4

    def _check_progress(self, now):
        """Detect a wedged/oscillating robot and trigger a recovery."""
        if len(self._pose_history) < 2:
            return
        while self._pose_history and (now - self._pose_history[0][0]) > self.stuck_time:
            self._pose_history.popleft()
        if len(self._pose_history) < 2 or (now - self._last_stuck_check) < 0.2:
            return
        self._last_stuck_check = now
        commanded = math.hypot(*self._last_action)
        oldest = self._pose_history[0]
        moved = math.hypot(self.robot_state[0, 0] - oldest[1],
                           self.robot_state[1, 0] - oldest[2])
        oscillating = self._is_oscillating(now)
        if commanded > 0.2 and (moved < self.stuck_distance or oscillating):
            if self._stuck_since is None:
                self._stuck_since = now
            elif (now - self._stuck_since) > self.stuck_time:
                self._stuck_since = None
                kind = 'wedged' if moved < 0.02 else ('oscillating' if oscillating else 'blocked')
                self.get_logger().warn(
                    f'no progress ({kind}): commanded {commanded:.2f} m/s, moved '
                    f'{moved:.3f} m, deviation {self._path_deviation():.2f} m')
                self._recover(kind)
        else:
            self._stuck_since = None

    def _recover(self, kind):
        """Replan with clearance; back out of the slot when the robot is pinned."""
        self._recoveries += 1
        if self._recoveries > self.max_recoveries:
            self.failed = True
            self._last_event = f'stuck ({kind}): giving up after {self.max_recoveries} tries'
            self.get_logger().warn(
                'cannot get around the obstacle; stopping (send a new goal to retry)',
                throttle_duration_sec=2.0)
            self._publish_stop()
            return

        deviation = self._path_deviation()
        try_reverse = kind == 'wedged' or (self._recoveries % 2 == 0)
        if try_reverse and self._start_reverse_escape():
            self._last_event = f'reversing out ({kind}, try {self._recoveries})'
            return
        if self._update_reference_path(reason=f'recover/{kind}', force=True):
            self._last_event = f'replanned around the obstacle ({kind})'
        elif self._try_detour():
            self._last_event = f'lateral detour ({kind})'
        elif deviation > self.max_path_deviation:
            self.planner.update_initial_path_from_goal(self.robot_state, self.goal)
            self.planner.reset()
            self._last_event = f'reference re-anchored ({kind})'
        else:
            self._last_event = f'no path around the obstacle ({kind})'

    def _start_reverse_escape(self):
        """Back out of a narrow slot for a moment (NeuPAN cannot un-wedge a robot)."""
        if self.goal is None or self.robot_state is None:
            return False
        yaw = float(self.robot_state[2, 0])
        x, y = float(self.robot_state[0, 0]), float(self.robot_state[1, 0])
        distance = min(1.2, 0.55 + 0.2 * self._recoveries)
        if self.local_grid is not None:
            behind = (x - 0.4 * math.cos(yaw), y - 0.4 * math.sin(yaw))
            if not self.local_grid.is_free(behind, max(0.2, self.local_inflation * 0.5)):
                return False       # nothing to back into
        duration = max(0.45, distance / max(self.reverse_speed, 0.1))
        self._escape = {'until': time.time() + duration, 'vx': -self.reverse_speed, 'vy': 0.0}
        self.planner.set_reference_speed(self.min_ref_speed)
        self._current_ref_speed = self.min_ref_speed
        self.get_logger().info(
            f'reverse escape: backing out {distance:.2f} m at {self.reverse_speed:.2f} m/s')
        return True

    def _adapt_speed(self):
        """Slow the reference down when the robot gets close to an obstacle."""
        if not self.speed_adapt or self.base_ref_speed <= 0:
            return
        md = scalar(self.planner.min_distance)
        if not math.isfinite(md):
            target = self.base_ref_speed
        elif md >= self.slowdown_distance:
            target = self.base_ref_speed
        else:
            target = max(self.min_ref_speed,
                         self.base_ref_speed * max(0.0, md) / self.slowdown_distance)
        if abs(target - self._current_ref_speed) > 0.05:
            self.planner.set_reference_speed(target)
            self._current_ref_speed = target

    # ------------------------------------------------------------------ #
    # Sensor processing
    # ------------------------------------------------------------------ #
    def _read_points(self, msg: PointCloud2):
        """Return the xyz points of a PointCloud2 as a (3, N) float array.

        ``sensor_msgs_py.point_cloud2.read_points`` builds one Python object per point
        (12k points at 10 Hz is ~50% of a core), so the cloud is decoded straight from
        the raw buffer with numpy instead.
        """
        pts = decode_xyz(msg)
        if pts is None or pts.shape[0] == 0:
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
    # Status (web UI / debugging)
    # ------------------------------------------------------------------ #
    def _mode(self):
        if self.robot_state is None:
            return 'no odometry'
        if self.failed:
            return 'failed'
        if self.arrived:
            return 'idle' if self.goal is None else 'arrived'
        if self.planner.initial_path is None:
            return 'waiting for goal'
        if self._escape:
            return 'reversing'
        return 'driving'

    def _status_dict(self):
        state = self.robot_state
        goal = self.goal
        dune = self.planner.dune_points
        nrmp = self.planner.nrmp_points
        return {
            'mode': self._mode(),
            'event': self._last_event,
            'pose': None if state is None else [round(float(state[0, 0]), 3),
                                                round(float(state[1, 0]), 3),
                                                round(float(state[2, 0]), 3)],
            'cmd': [round(float(self._last_action[0]), 3),
                    round(float(self._last_action[1]), 3)],
            'goal': None if goal is None else [round(float(goal[0, 0]), 2),
                                               round(float(goal[1, 0]), 2),
                                               round(float(goal[2, 0]), 2)],
            'min_dist': round(scalar(self.planner.min_distance), 3),
            'points': 0 if self.obstacle_points is None else int(self.obstacle_points.shape[1]),
            'dune_points': 0 if dune is None else int(np.asarray(dune).shape[1]),
            'nrmp_points': 0 if nrmp is None else int(np.asarray(nrmp).shape[1]),
            'lidar_height': round(self.lidar_height, 3),
            'box': [self.box_length, self.box_width, self.box_height],
            'z_robot': self.z_robot,
            'max_linear': self.max_linear,
            'kinematics': 'omni' if self.omni else 'diff',
            'adjust': {k: round(float(v), 3) for k, v in self.adjust_current.items()},
            'adjust_defaults': {k: round(float(v), 3)
                                for k, v in self.adjust_defaults.items()},
            'ref_speed': round(float(self.base_ref_speed), 2),
            'ref_speed_default': round(float(self.ref_speed_default), 2),
        }

    def publish_status(self):
        msg = String()
        msg.data = json.dumps(self._status_dict(), ensure_ascii=False)
        self.status_pub.publish(msg)

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
            if self._due('_cloud_at', self.cloud_rate):
                self._publish_obstacles()
            if self._due('_viz_at', self.viz_rate):
                self._publish_visualization(None)
            self._publish_goal()
            return

        if self.startup_goal is not None and self.goal is None:
            self.set_goal(self.startup_goal)

        self._broadcast_robot_tf()
        self._publish_goal()

        if self._escape:
            if time.time() < self._escape['until']:
                # open-loop back-out: NeuPAN cannot help while the robot is wedged
                self._publish_action(np.array([[self._escape['vx']], [self._escape['vy']]]))
                self._broadcast_robot_tf()
                return
            self._escape = None
            self._update_reference_path(reason='after reverse', force=True)
            self._publish_stop()
            return

        if self.arrived or self.failed:
            self._publish_stop()
            if self._due('_viz_at', self.viz_rate):
                self._publish_visualization(self.planner.info)
            return

        action, info = self.planner(self.robot_state, self.obstacle_points)
        if self._due('_cloud_at', self.cloud_rate):
            self._publish_obstacles()

        now_ts = time.time()
        self._adapt_speed()
        self._check_progress(now_ts)
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
                self._last_event = f'arrived ({where})'
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
                f'stop: min_dist={scalar(self.planner.min_distance):.3f} < '
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
                f'min_dist={scalar(self.planner.min_distance):.3f} '
                f'points={0 if self.obstacle_points is None else self.obstacle_points.shape[1]} '
                f'lidar_h={self.lidar_height:.2f} '
                f'v=({self._last_action[0]:.2f}, {self._last_action[1]:.2f})')

    def _publish_action(self, action: np.ndarray):
        self._cmd_history.append((time.time(), float(action[0, 0]),
                                  float(action[1, 0]) if action.shape[0] > 1 else 0.0))
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
    def _due(self, attr, rate):
        """True (and update the stamp) when ``rate`` Hz throttle allows a publish."""
        now = time.time()
        if now - getattr(self, attr) >= 1.0 / rate:
            setattr(self, attr, now)
            return True
        return False

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
