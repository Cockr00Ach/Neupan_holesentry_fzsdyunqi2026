#!/usr/bin/env python3
"""Web bridge: stream the NeuPAN/RViz view to a browser and accept goals from it.

This is the "RViz in the browser" part of the fzsd2026 NeuPAN demo.  It runs a small
ROS 2 node that aggregates the planner topics into one JSON frame and serves it over
HTTP (Server-Sent Events) together with a single-page canvas UI:

    http://127.0.0.1:8088/

Topics it reads (published by ``neupan_ros2_bridge.py``)
    /neupan_status            std_msgs/String      JSON: mode, pose, goal, min_dist, ...
    /neupan_obstacles         sensor_msgs/PointCloud2   filtered obstacle points (map)
    /neupan_plan              nav_msgs/Path        optimised trajectory
    /neupan_ref_state         nav_msgs/Path        reference states
    /neupan_initial_path      nav_msgs/Path        initial/reference path
    /neupan_global_path       nav_msgs/Path        A* path (when enabled)
    /dune_point_markers       visualization_msgs/MarkerArray
    /nrmp_point_markers       visualization_msgs/MarkerArray

Topics it writes
    /goal_pose                geometry_msgs/PoseStamped   (click on the map)
    /neupan_cancel            std_msgs/Empty              ("stop" button)

Run it standalone::

    python3 web/web_server.py --port 8088
    # or together with everything else: ./run_sim.sh --web
"""

import argparse
import json
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from flask import Flask, Response, jsonify, request, send_from_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty, String
from visualization_msgs.msg import MarkerArray

HERE = os.path.dirname(os.path.abspath(__file__))
STATIC = os.path.join(HERE, 'static')


# --------------------------------------------------------------------------- #
# Shared state
# --------------------------------------------------------------------------- #
class Store:
    """Latest planner snapshot, shared between the ROS node and the HTTP handlers."""

    def __init__(self):
        self.lock = threading.Lock()
        self.version = 0
        self.data = {
            'seq': 0,
            'frame': 'map',
            'status': {'mode': 'starting'},
            'cloud': [],
            'dune': [],
            'nrmp': [],
            'plan': [],
            'ref': [],
            'initial': [],
            'astar': [],
        }

    def update(self, **kwargs):
        with self.lock:
            self.data.update(kwargs)
            self.version += 1
            self.data['seq'] = self.version
            self.data['stamp'] = time.time()

    def snapshot(self):
        with self.lock:
            return json.loads(json.dumps(self.data))


STORE = Store()


# --------------------------------------------------------------------------- #
# ROS node
# --------------------------------------------------------------------------- #
class WebBridge(Node):
    def __init__(self, cloud_max=300, point_rate=12.0):
        super().__init__('neupan_web')
        self.cloud_max = cloud_max
        self._last_cloud_time = 0.0
        self._min_cloud_period = 1.0 / max(1.0, point_rate)

        cloud_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                               history=HistoryPolicy.KEEP_LAST, depth=2)
        self.create_subscription(String, '/neupan_status', self.on_status, 10)
        self.create_subscription(PointCloud2, '/neupan_obstacles', self.on_cloud, cloud_qos)
        for topic, key in (('/neupan_plan', 'plan'),
                           ('/neupan_ref_state', 'ref'),
                           ('/neupan_initial_path', 'initial'),
                           ('/neupan_global_path', 'astar')):
            self.create_subscription(Path, topic,
                                     lambda msg, k=key: self.on_path(k, msg), 10)
        for topic, key in (('/dune_point_markers', 'dune'),
                           ('/nrmp_point_markers', 'nrmp')):
            self.create_subscription(MarkerArray, topic,
                                     lambda msg, k=key: self.on_markers(k, msg), 10)

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cancel_pub = self.create_publisher(Empty, '/neupan_cancel', 10)
        self.get_logger().info('web bridge ready')

    # ---- callbacks -------------------------------------------------------- #
    def on_status(self, msg: String):
        try:
            status = json.loads(msg.data)
        except ValueError:
            return
        STORE.update(status=status)

    def on_cloud(self, msg: PointCloud2):
        now = time.time()
        if now - self._last_cloud_time < self._min_cloud_period:
            return
        self._last_cloud_time = now
        arr = decode_xyz(msg)
        if arr is None:
            return
        pts = arr.tolist()
        if not pts:
            return
        if len(pts) > self.cloud_max:
            step = int(math.ceil(len(pts) / self.cloud_max))
            pts = pts[::step]
        STORE.update(cloud=[[round(p[0], 2), round(p[1], 2), round(p[2], 2)] for p in pts])

    def on_path(self, key, msg: Path):
        pts = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in msg.poses]
        STORE.update(**{key: pts})

    def on_markers(self, key, msg: MarkerArray):
        pts = [[round(m.pose.position.x, 3), round(m.pose.position.y, 3),
                round(m.pose.position.z, 3)] for m in msg.markers]
        STORE.update(**{key: pts})

    # ---- commands --------------------------------------------------------- #
    def send_goal(self, x, y, yaw=0.0):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(float(yaw) / 2.0)
        msg.pose.orientation.w = math.cos(float(yaw) / 2.0)
        self.goal_pub.publish(msg)
        self.get_logger().info(f'web goal: ({x:.2f}, {y:.2f})')

    def cancel(self):
        self.cancel_pub.publish(Empty())
        self.get_logger().info('web cancel')


NODE = None


def ros_thread(args):
    global NODE
    rclpy.init()
    NODE = WebBridge(cloud_max=args.cloud_max, point_rate=args.point_rate)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(NODE)
    try:
        executor.spin()
    except (KeyboardInterrupt, Exception):  # pragma: no cover
        pass
    finally:
        executor.shutdown()
        NODE.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# --------------------------------------------------------------------------- #
# HTTP app
# --------------------------------------------------------------------------- #
app = Flask(__name__, static_folder=None)


@app.route('/')
def index():
    return send_from_directory(STATIC, 'index.html')


@app.route('/<path:name>')
def static_files(name):
    return send_from_directory(STATIC, name)


@app.route('/api/state')
def api_state():
    return jsonify(STORE.snapshot())


@app.route('/api/stream')
def api_stream():
    def generate():
        last = -1
        while True:
            data = STORE.snapshot()
            if data['seq'] != last:
                last = data['seq']
                yield 'data: ' + json.dumps(data, separators=(',', ':')) + '\n\n'
            time.sleep(0.04)

    return Response(generate(), mimetype='text/event-stream',
                    headers={'Cache-Control': 'no-cache',
                             'X-Accel-Buffering': 'no',
                             'Connection': 'keep-alive'})


@app.route('/api/goal', methods=['POST'])
def api_goal():
    payload = request.get_json(silent=True) or {}
    try:
        x = float(payload['x'])
        y = float(payload['y'])
    except (KeyError, TypeError, ValueError):
        return jsonify({'ok': False, 'error': 'x and y are required'}), 400
    yaw = float(payload.get('yaw', 0.0))
    if NODE is None:
        return jsonify({'ok': False, 'error': 'ROS node not ready'}), 503
    NODE.send_goal(x, y, yaw)
    return jsonify({'ok': True, 'goal': [x, y, yaw]})


@app.route('/api/stop', methods=['POST'])
def api_stop():
    if NODE is None:
        return jsonify({'ok': False, 'error': 'ROS node not ready'}), 503
    NODE.cancel()
    return jsonify({'ok': True})


@app.route('/api/health')
def api_health():
    data = STORE.snapshot()
    return jsonify({'ok': True, 'mode': data['status'].get('mode'),
                    'seq': data['seq'], 'ros': NODE is not None})


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


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8088)
    parser.add_argument('--cloud-max', type=int, default=300,
                        help='max obstacle points sent to the browser')
    parser.add_argument('--point-rate', type=float, default=10.0,
                        help='max obstacle cloud update rate [Hz]')
    args = parser.parse_args()

    thread = threading.Thread(target=ros_thread, args=(args,), daemon=True)
    thread.start()
    time.sleep(1.0)

    print(f'[web] NeuPAN web view on http://{args.host}:{args.port}/')
    app.run(host=args.host, port=args.port, threaded=True, debug=False, use_reloader=False)


if __name__ == '__main__':
    main()
