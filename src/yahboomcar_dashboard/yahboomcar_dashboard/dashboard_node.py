#!/usr/bin/env python3
# encoding: utf-8
"""
Yahboomcar X3 Web Dashboard.

Subscribes to chassis / IMU / odom / lidar / safety topics, tracks each
device's last-seen timestamp to infer connection status, and serves a
single-page web dashboard over an embedded HTTP server (Python stdlib,
no Flask required).

URL:  http://<robot-ip>:8088/
API:
  GET  /api/state    -> JSON snapshot (polled by the frontend at ~10 Hz)
  POST /api/cmd_vel  -> JSON {vx, vy, wz} -> published once on /cmd_vel
"""

import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool, Float32, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry

from ament_index_python.packages import get_package_share_directory


# -------- helpers ----------------------------------------------------------

def quat_to_rpy(x, y, z, w):
    # roll (X), pitch (Y), yaw (Z)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


# -------- shared state -----------------------------------------------------

class SharedState:
    """Thread-safe snapshot consumed by the HTTP handler."""

    def __init__(self):
        self._lock = threading.Lock()
        self._data = {
            # chassis / battery
            'voltage': None,
            'edition': None,
            'safety_ok': None,
            # commanded velocity
            'cmd_vel': {'vx': 0.0, 'vy': 0.0, 'vz': 0.0,
                        'wx': 0.0, 'wy': 0.0, 'wz': 0.0},
            # actual velocity from vel_raw
            'vel_raw': {'vx': 0.0, 'vy': 0.0, 'vz': 0.0,
                        'wx': 0.0, 'wy': 0.0, 'wz': 0.0},
            # odometry (pose + velocity in odom frame)
            'odom': {
                'x': 0.0, 'y': 0.0, 'z': 0.0,
                'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                'vx': 0.0, 'vy': 0.0, 'vz': 0.0,
                'wx': 0.0, 'wy': 0.0, 'wz': 0.0,
            },
            # IMU
            'imu': {
                'ax': 0.0, 'ay': 0.0, 'az': 0.0,
                'wx': 0.0, 'wy': 0.0, 'wz': 0.0,
                'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            },
            # lidar quick stats
            'scan': {'count': 0, 'min': 0.0, 'max': 0.0},
            # lidar points transformed into the odom frame (subsampled)
            # list of [x, y] floats; cleared each scan
            'scan_xy': [],
            # collision / joy flags
            'collision': None,
            'joy_state': None,
            # device last-seen timestamps (wall clock, seconds)
            'last_seen': {},
        }

    def set(self, key, value):
        with self._lock:
            self._data[key] = value

    def touch(self, device, now):
        with self._lock:
            self._data['last_seen'][device] = now

    def snapshot(self, now):
        with self._lock:
            data = json.loads(json.dumps(self._data))  # deep copy
        # derive connection status from staleness
        timeouts = {
            'chassis': 2.0,    # voltage @ 10 Hz
            'imu': 1.0,        # imu/data_raw @ ~50 Hz
            'odom': 2.0,
            'lidar': 2.0,
            'cmd_vel': 2.0,
            'safety': 5.0,
            'joy': 5.0,
        }
        devices = {}
        for dev, timeout in timeouts.items():
            ts = data['last_seen'].get(dev)
            if ts is None:
                devices[dev] = {'connected': False, 'age': None}
            else:
                age = now - ts
                devices[dev] = {'connected': age < timeout, 'age': round(age, 2)}
        data['devices'] = devices
        data['server_time'] = now
        return data


# -------- ROS node ---------------------------------------------------------

class DashboardNode(Node):
    def __init__(self, state: SharedState):
        super().__init__('yahboomcar_dashboard')
        self.state = state

        # sensor data can be lossy; use BEST_EFFORT for IMU/scan
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(Float32, 'voltage', self._on_voltage, 10)
        self.create_subscription(Float32, 'edition', self._on_edition, 10)
        self.create_subscription(Bool, 'safety_status', self._on_safety, 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(Twist, 'vel_raw', self._on_vel_raw, 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(Imu, 'imu/data_raw', self._on_imu, sensor_qos)
        self.create_subscription(LaserScan, 'scan', self._on_scan, sensor_qos)
        self.create_subscription(Bool, 'JoyState', self._on_joy, 10)

        # The collision detector publishes on the private topic ~/collision
        # which resolves to /<node>/collision (default: /collision_detector
        # /collision). Make this configurable so other setups can rewire it
        # without an external remap.
        self.declare_parameter(
            'collision_topic', '/collision_detector/collision')
        collision_topic = self.get_parameter(
            'collision_topic').get_parameter_value().string_value
        self.create_subscription(
            Bool, collision_topic, self._on_collision, 10)
        self.get_logger().info(
            f'Listening for collision flag on: {collision_topic}')

        # publisher for web joystick -> /cmd_vel
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._cmd_lock = threading.Lock()
        self._last_web_cmd_ts = 0.0
        self._web_cmd_active = False
        # watchdog: if a web command was the last to publish, stop the robot
        # when the browser stops sending (network drop / tab closed).
        self.create_timer(0.1, self._cmd_watchdog)

        # Cached pose for the scan callback so it does not need to call
        # SharedState.snapshot() (which deep-copies the whole dict) on every
        # high-rate /scan message. Updated in _on_odom.
        self._pose_xy_yaw = (0.0, 0.0, 0.0)

        self.get_logger().info('yahboomcar_dashboard node started.')

    # --- web -> /cmd_vel --------------------------------------------------

    def publish_web_cmd(self, vx: float, vy: float, wz: float):
        """Called from the HTTP thread when the browser POSTs a command."""
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(wz)
        with self._cmd_lock:
            self._cmd_pub.publish(msg)
            self._last_web_cmd_ts = time.time()
            self._web_cmd_active = (vx != 0.0 or vy != 0.0 or wz != 0.0)

    def _cmd_watchdog(self):
        # If we were actively driving from the web but stopped hearing
        # heartbeats for >0.4 s, publish a single zero Twist as a safety.
        with self._cmd_lock:
            if not self._web_cmd_active:
                return
            if time.time() - self._last_web_cmd_ts > 0.4:
                stop = Twist()
                self._cmd_pub.publish(stop)
                self._web_cmd_active = False
                self.get_logger().warn('Web cmd_vel watchdog: stopped.')

    @staticmethod
    def _now():
        import time
        return time.time()

    # --- callbacks --------------------------------------------------------

    def _on_voltage(self, msg: Float32):
        self.state.set('voltage', round(float(msg.data), 2))
        self.state.touch('chassis', self._now())

    def _on_edition(self, msg: Float32):
        self.state.set('edition', round(float(msg.data), 2))
        self.state.touch('chassis', self._now())

    def _on_safety(self, msg: Bool):
        self.state.set('safety_ok', bool(msg.data))
        self.state.touch('safety', self._now())

    def _on_cmd_vel(self, msg: Twist):
        self.state.set('cmd_vel', {
            'vx': msg.linear.x, 'vy': msg.linear.y, 'vz': msg.linear.z,
            'wx': msg.angular.x, 'wy': msg.angular.y, 'wz': msg.angular.z,
        })
        self.state.touch('cmd_vel', self._now())

    def _on_vel_raw(self, msg: Twist):
        self.state.set('vel_raw', {
            'vx': msg.linear.x, 'vy': msg.linear.y, 'vz': msg.linear.z,
            'wx': msg.angular.x, 'wy': msg.angular.y, 'wz': msg.angular.z,
        })

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.twist.twist
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        self.state.set('odom', {
            'x': p.x, 'y': p.y, 'z': p.z,
            'roll': roll, 'pitch': pitch, 'yaw': yaw,
            'vx': t.linear.x, 'vy': t.linear.y, 'vz': t.linear.z,
            'wx': t.angular.x, 'wy': t.angular.y, 'wz': t.angular.z,
        })
        # cache for _on_scan; tuple rebind is atomic under the GIL so no
        # lock is needed even with a multi-threaded executor
        self._pose_xy_yaw = (p.x, p.y, yaw)
        self.state.touch('odom', self._now())

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        self.state.set('imu', {
            'ax': msg.linear_acceleration.x,
            'ay': msg.linear_acceleration.y,
            'az': msg.linear_acceleration.z,
            'wx': msg.angular_velocity.x,
            'wy': msg.angular_velocity.y,
            'wz': msg.angular_velocity.z,
            'roll': roll, 'pitch': pitch, 'yaw': yaw,
        })
        self.state.touch('imu', self._now())

    def _on_scan(self, msg: LaserScan):
        # use cached pose (updated in _on_odom) instead of taking a full
        # SharedState.snapshot() — the latter does a JSON deep-copy of all
        # state, which is wasteful on every high-rate /scan callback
        rx, ry, ryaw = self._pose_xy_yaw
        cos_y = math.cos(ryaw)
        sin_y = math.sin(ryaw)

        # subsample so the JSON stays small (~180 points)
        n = len(msg.ranges)
        step = max(1, n // 180)

        ranges_clean = []
        xy = []
        for i in range(0, n, step):
            r = msg.ranges[i]
            if math.isnan(r) or math.isinf(r) or r <= 0.0:
                continue
            if msg.range_min and r < msg.range_min:
                continue
            if msg.range_max and r > msg.range_max:
                continue
            ranges_clean.append(r)
            ang = msg.angle_min + i * msg.angle_increment
            # point in robot frame
            px = r * math.cos(ang)
            py = r * math.sin(ang)
            # transform into odom frame
            wx = rx + cos_y * px - sin_y * py
            wy = ry + sin_y * px + cos_y * py
            xy.append([round(wx, 3), round(wy, 3)])

        if ranges_clean:
            self.state.set('scan', {
                'count': len(ranges_clean),
                'min': round(min(ranges_clean), 3),
                'max': round(max(ranges_clean), 3),
            })
        self.state.set('scan_xy', xy)
        self.state.touch('lidar', self._now())

    def _on_collision(self, msg: Bool):
        self.state.set('collision', bool(msg.data))

    def _on_joy(self, msg: Bool):
        self.state.set('joy_state', bool(msg.data))
        self.state.touch('joy', self._now())


# -------- HTTP server ------------------------------------------------------

def make_handler(state: SharedState, web_dir: str, node: 'DashboardNode'):
    # Resolve once so we can enforce containment on every request.
    web_root = os.path.realpath(web_dir)

    def safe_join(rel: str):
        """Resolve rel under web_root, or return None if it escapes."""
        # reject absolute paths and any path with traversal components up front
        if not rel or rel.startswith('/') or rel.startswith('\\'):
            return None
        # split on both separators; reject empty / dot / dotdot segments
        for part in rel.replace('\\', '/').split('/'):
            if part in ('', '.', '..'):
                return None
        candidate = os.path.realpath(os.path.join(web_root, rel))
        # must be strictly inside web_root (use sep to avoid prefix collisions)
        if candidate != web_root and not candidate.startswith(
                web_root + os.sep):
            return None
        return candidate

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):  # silence default access log
            pass

        def _send(self, code, body, content_type):
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path in ('/', '/index.html'):
                self._serve_file(os.path.join(web_dir, 'index.html'),
                                 'text/html; charset=utf-8')
                return
            if path == '/api/state':
                payload = json.dumps(state.snapshot(time.time())).encode('utf-8')
                self._send(200, payload, 'application/json')
                return
            # static assets under /web/
            if path.startswith('/web/'):
                rel = path[len('/web/'):]
                resolved = safe_join(rel)
                if resolved is None:
                    self._send(403, b'forbidden', 'text/plain')
                    return
                self._serve_file(resolved, self._guess_type(rel))
                return
            self._send(404, b'not found', 'text/plain')

        def do_POST(self):
            path = self.path.split('?', 1)[0]
            if path != '/api/cmd_vel':
                self._send(404, b'not found', 'text/plain')
                return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                body = self.rfile.read(length) if length > 0 else b'{}'
                data = json.loads(body.decode('utf-8'))
                vx = float(data.get('vx', 0.0))
                vy = float(data.get('vy', 0.0))
                wz = float(data.get('wz', 0.0))
                # hard server-side clamps as a final safety net
                vx = max(-1.0, min(1.0, vx))
                vy = max(-1.0, min(1.0, vy))
                wz = max(-3.0, min(3.0, wz))
                node.publish_web_cmd(vx, vy, wz)
                self._send(200, b'{"ok":true}', 'application/json')
            except Exception as e:
                self._send(400, json.dumps({'ok': False, 'error': str(e)})
                           .encode('utf-8'), 'application/json')

        def _serve_file(self, fpath, ctype):
            try:
                if not os.path.isfile(fpath):
                    self._send(404, b'not found', 'text/plain')
                    return
                with open(fpath, 'rb') as f:
                    body = f.read()
                self._send(200, body, ctype)
            except (FileNotFoundError, IsADirectoryError, PermissionError):
                self._send(404, b'not found', 'text/plain')

        @staticmethod
        def _guess_type(name):
            if name.endswith('.html'):
                return 'text/html; charset=utf-8'
            if name.endswith('.js'):
                return 'application/javascript'
            if name.endswith('.css'):
                return 'text/css'
            if name.endswith('.svg'):
                return 'image/svg+xml'
            return 'application/octet-stream'

    return Handler


def find_web_dir():
    # 1) installed share dir
    try:
        share = get_package_share_directory('yahboomcar_dashboard')
        cand = os.path.join(share, 'web')
        if os.path.isdir(cand):
            return cand
    except Exception:
        pass
    # 2) sibling of this source file (works with colcon --symlink-install)
    here = os.path.dirname(os.path.abspath(__file__))
    cand = os.path.join(here, 'web')
    if os.path.isdir(cand):
        return cand
    raise RuntimeError('Could not locate dashboard web/ assets.')


# -------- main -------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    state = SharedState()
    node = DashboardNode(state)

    node.declare_parameter('host', '0.0.0.0')
    node.declare_parameter('port', 8088)
    host = node.get_parameter('host').get_parameter_value().string_value
    port = node.get_parameter('port').get_parameter_value().integer_value

    web_dir = find_web_dir()
    handler_cls = make_handler(state, web_dir, node)
    httpd = ThreadingHTTPServer((host, port), handler_cls)

    server_thread = threading.Thread(
        target=httpd.serve_forever, name='dashboard-http', daemon=True)
    server_thread.start()
    node.get_logger().info(f'Dashboard serving on http://{host}:{port}/')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
