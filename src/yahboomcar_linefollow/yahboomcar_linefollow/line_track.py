#!/usr/bin/env python3
# encoding: utf-8
"""Line-follow TRACK mode.

Load an HSV range previously saved by `line_detect`, look at a configurable
horizontal band near the bottom of the frame, find the line centroid, and
drive /cmd_vel with a PID on the lateral error.

Topics:
    publishes  /cmd_vel  (geometry_msgs/Twist)
    subscribes /JoyState (std_msgs/Bool)  -- when True, pause autonomy

Parameters:
    camera_index        int   default 0   (used only when image_topic is empty)
    image_topic         str   ''   subscribe to this sensor_msgs/Image instead
                                    of opening a local camera (e.g. Gazebo's
                                    /camera/image_raw). Empty = use camera_index.
    frame_width         int   640
    frame_height        int   480
    hsv_file            str   ~/.yahboomcar_linefollow_hsv.txt
    roi_top_ratio       float 0.65  (band starts at 65% down the frame)
    roi_bottom_ratio    float 1.00
    linear              float 0.15  m/s forward speed
    angular_max         float 1.2   rad/s clamp
    kp/ki/kd            float PID gains on normalized lateral error
    show_window         bool  True
    switch              bool  True  master enable
"""

import os
import time

import cv2 as cv
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image



from ament_index_python.packages import get_package_share_directory

from .line_common import (
    read_hsv,
    mask_with_hsv,
    largest_contour_centroid,
    count_blobs,
    SimplePID,
)
from .qr_common import QRReader, load_actions, resolve_action

WINDOW = 'line_track'


def _default_hsv_path():
    return os.path.join(
        get_package_share_directory('yahboomcar_linefollow'),
        'params', 'HSV.txt')


def _default_qr_actions_path():
    return os.path.join(
        get_package_share_directory('yahboomcar_linefollow'),
        'params', 'qr_actions.json')


class LineTrack(Node):
    def __init__(self):
        super().__init__('line_track')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('image_topic', '')
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('hsv_file', _default_hsv_path())
        self.declare_parameter('roi_top_ratio', 0.65)
        self.declare_parameter('roi_bottom_ratio', 1.0)
        self.declare_parameter('linear', 0.15)
        self.declare_parameter('angular_max', 1.2)
        self.declare_parameter('kp', 1.6)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.4)
        self.declare_parameter('show_window', True)
        self.declare_parameter('switch', True)
        # Verbose per-frame diagnostics (error/centroid/area/blob count).
        self.declare_parameter('debug', False)
        # Glue with yahboomcar_collision: when a True pulse arrives on
        # `collision_topic`, stop publishing motion for `collision_pause_sec`
        # seconds and reset the PID. Set the topic empty to disable.
        self.declare_parameter('collision_topic',
                               '/collision_detector/collision')
        self.declare_parameter('collision_pause_sec', 2.0)

        # QR priority: scan the same frame for a QR code; when one resolves to
        # a fork-road action, interrupt line following and execute it. The QR
        # maneuver outranks normal line following but yields to the manual
        # joystick override and the collision safety stop.
        self.declare_parameter('enable_qr', True)
        self.declare_parameter('qr_actions_file', _default_qr_actions_path())
        self.declare_parameter('qr_check_every', 3)      # run detect every N ticks
        self.declare_parameter('qr_cooldown_sec', 4.0)   # ignore same payload again

        self.cam_index = self.get_parameter('camera_index').value
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.w = int(self.get_parameter('frame_width').value)
        self.h = int(self.get_parameter('frame_height').value)
        self.hsv_file = self.get_parameter('hsv_file').value
        self.show = bool(self.get_parameter('show_window').value)

        self.hsv_range = read_hsv(self.hsv_file)
        if not self.hsv_range:
            self.get_logger().error(
                f'HSV file not found or empty: {self.hsv_file}. '
                f'Run line_detect first to learn an HSV range.')
            raise RuntimeError('no HSV calibration')

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_joy = self.create_subscription(
            Bool, '/JoyState', self._on_joy, 1)
        self.joy_active = False

        # collision hold-off state
        self._collision_hold_until = self.get_clock().now()
        collision_topic = self.get_parameter('collision_topic').value
        if collision_topic:
            self.sub_collision = self.create_subscription(
                Bool, collision_topic, self._on_collision, 10)
            self.get_logger().info(
                f'collision hold-off listening on "{collision_topic}"')

        # QR priority state — QRReader is created once; enable_qr is read
        # live each tick so `ros2 param set /line_track enable_qr false`
        # takes effect immediately without restarting the node.
        self._qr = QRReader()
        if not self._qr.available():
            self.get_logger().warn(
                'cv2.QRCodeDetector not available in this OpenCV build -- '
                'QR priority disabled regardless of enable_qr parameter.')
        self._qr_table = load_actions(
            self.get_parameter('qr_actions_file').value)
        self._qr_state = None          # active maneuver dict, or None
        self._qr_tick = 0
        self._qr_last_payload = None
        self._qr_cooldown_until = self.get_clock().now()
        self.pub_qr = self.create_publisher(String, '~/qr', 10)
        n = len(self._qr_table.get('actions', {}))
        self.get_logger().info(
            f'QR reader {"ready" if self._qr.available() else "unavailable"}'
            f' -- {n} action(s) from '
            f'{self.get_parameter("qr_actions_file").value} '
            f'(enable_qr={self.get_parameter("enable_qr").value})')

        self.pid = SimplePID(
            kp=float(self.get_parameter('kp').value),
            ki=float(self.get_parameter('ki').value),
            kd=float(self.get_parameter('kd').value),
            i_clamp=1.0,
            out_clamp=float(self.get_parameter('angular_max').value),
        )

        # Image source: a ROS topic (simulation / external driver) or a local
        # camera opened directly with OpenCV (real USB camera).
        self.cap = None
        self._frame = None
        self._bridge = None
        if self.image_topic:
            from cv_bridge import CvBridge
            self._bridge = CvBridge()
            self.sub_image = self.create_subscription(
                Image, self.image_topic, self._on_image, 10)
            self.get_logger().info(
                f'reading frames from image topic "{self.image_topic}"')
        else:
            self.cap = cv.VideoCapture(self.cam_index)
            if not self.cap.isOpened():
                self.get_logger().error(
                    f'cannot open camera index {self.cam_index}')
                raise RuntimeError('camera open failed')
            self.cap.set(cv.CAP_PROP_FRAME_WIDTH, self.w)
            self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.h)

        if self.show:
            cv.namedWindow(WINDOW, cv.WINDOW_AUTOSIZE)

        self.timer = self.create_timer(0.03, self._tick)
        self._t_prev = time.time()

        # diagnostics state
        self._tracking = False          # were we locked onto a line last tick?
        self._no_frame_since = time.time()
        self._frames_seen = 0

    def _on_image(self, msg):
        try:
            self._frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'cv_bridge convert failed: {exc}')

    def _grab(self):
        """Return (ok, frame) from whichever image source is configured."""
        if self.cap is not None:
            return self.cap.read()
        if self._frame is None:
            return False, None
        return True, self._frame

    def _on_joy(self, msg):
        self.joy_active = bool(msg.data)
        if self.joy_active:
            self.pid.reset()
            self.pub_cmd.publish(Twist())

    def _on_collision(self, msg):
        if not msg.data:
            return
        pause = float(self.get_parameter('collision_pause_sec').value)
        dt = rclpy.duration.Duration(
            seconds=int(pause),
            nanoseconds=int((pause % 1) * 1e9))
        self._collision_hold_until = self.get_clock().now() + dt
        self.pid.reset()
        self.pub_cmd.publish(Twist())
        self.get_logger().warn(
            f'collision flag received -- pausing line-follow for {pause:.1f}s')

    def _after(self, seconds):
        dt = rclpy.duration.Duration(
            seconds=int(seconds),
            nanoseconds=int((seconds % 1) * 1e9))
        return self.get_clock().now() + dt

    def _publish_qr(self, text):
        self.pub_qr.publish(String(data=text))

    def _start_qr_maneuver(self, payload, action):
        """Latch a resolved fork-road action as the active maneuver."""
        a_type = action['type']
        state = {'type': a_type, 'payload': payload}

        if a_type in ('left', 'right'):
            state['until'] = self._after(float(action.get('turn_time', 1.2)))
            state['turn_speed'] = float(action.get('turn_speed', 0.6))
            state['cross_speed'] = float(action.get('cross_speed', 0.12))
        elif a_type == 'straight':
            state['until'] = self._after(float(action.get('cross_time', 0.6)))
            state['cross_speed'] = float(action.get('cross_speed', 0.12))
        else:  # station / stop
            hold = float(action.get('hold_time', 0.0))
            # hold_time <= 0 latches the stop until 'switch' is toggled.
            state['until'] = self._after(hold) if hold > 0 else None

        self._qr_state = state
        self.pid.reset()
        self._qr_last_payload = payload
        self._qr_cooldown_until = self._after(
            float(self.get_parameter('qr_cooldown_sec').value))
        self.get_logger().info(f'QR "{payload}" -> {a_type} maneuver')
        self._publish_qr(f'{payload}:{a_type}')

    def _qr_maneuver_twist(self):
        """Return (twist, still_active) for the active QR maneuver.

        still_active is False once the maneuver is done; the caller then
        clears self._qr_state and resumes line following.
        """
        st = self._qr_state
        a_type = st['type']

        if a_type in ('station', 'stop'):
            if st['until'] is None:
                # Latched: stay stopped. Toggling 'switch' off clears the latch.
                if not bool(self.get_parameter('switch').value):
                    return Twist(), False
                return Twist(), True
            if self.get_clock().now() >= st['until']:
                return Twist(), False
            return Twist(), True

        # left / right / straight are timed open-loop moves.
        if self.get_clock().now() >= st['until']:
            return Twist(), False
        twist = Twist()
        twist.linear.x = st['cross_speed']
        if a_type == 'left':
            twist.angular.z = st['turn_speed']
        elif a_type == 'right':
            twist.angular.z = -st['turn_speed']
        return twist, True

    def _maybe_detect_qr(self, frame, now):
        # Skip detection while a timed turn is running so the same code can't
        # retrigger mid-maneuver; a latched stop keeps scanning so a fresh code
        # can redirect / resume the car.
        timed_active = (self._qr_state is not None
                        and self._qr_state.get('until') is not None)
        if timed_active:
            return
        self._qr_tick += 1
        every = max(1, int(self.get_parameter('qr_check_every').value))
        if self._qr_tick % every != 0:
            return
        payload, _pts = self._qr.detect(frame)
        if not payload:
            return
        if payload == self._qr_last_payload and now < self._qr_cooldown_until:
            return
        action = resolve_action(payload, self._qr_table)
        if action is None:
            self.get_logger().warn(f'QR "{payload}" has no matching action')
            self._qr_last_payload = payload
            self._qr_cooldown_until = self._after(
                float(self.get_parameter('qr_cooldown_sec').value))
            return
        self._start_qr_maneuver(payload, action)

    def _refresh_params(self):
        self.pid.kp = float(self.get_parameter('kp').value)
        self.pid.ki = float(self.get_parameter('ki').value)
        self.pid.kd = float(self.get_parameter('kd').value)
        self.pid.out_clamp = float(self.get_parameter('angular_max').value)

    def _tick(self):
        ok, frame = self._grab()
        if not ok or frame is None:
            # No image yet / stale stream -- a frequent "robot does nothing"
            # cause. Warn (throttled) so it is not silent.
            src = self.image_topic or f'camera index {self.cam_index}'
            self.get_logger().warn(
                f'no frame from {src} ({time.time() - self._no_frame_since:.1f}s '
                f'since last); is the camera/topic publishing?',
                throttle_duration_sec=3.0)
            return
        self._no_frame_since = time.time()
        self._frames_seen += 1
        debug = bool(self.get_parameter('debug').value)
        frame = cv.resize(frame, (self.w, self.h))
        self._refresh_params()

        # ROI band near the bottom
        top_r = float(self.get_parameter('roi_top_ratio').value)
        bot_r = float(self.get_parameter('roi_bottom_ratio').value)
        y0 = int(self.h * max(0.0, min(1.0, top_r)))
        y1 = int(self.h * max(0.0, min(1.0, bot_r)))
        if y1 <= y0:
            y1 = self.h
        band = frame[y0:y1]

        if not self.hsv_range:
            self.get_logger().error(
                'no HSV range loaded -- run line_detect or set hsv_file',
                throttle_duration_sec=5.0)
            return

        binary = mask_with_hsv(band, self.hsv_range)
        centroid = largest_contour_centroid(binary)
        blobs = count_blobs(binary)

        # Ambiguity: more than one blob means the HSV matches several lines
        # (e.g. two colours), which makes the centroid jump between them.
        if blobs > 1:
            self.get_logger().warn(
                f'{blobs} blobs in ROI -- HSV may match more than one line/'
                f'colour; tighten hsv_file or narrow the ROI',
                throttle_duration_sec=2.0)

        twist = Twist()
        now = self.get_clock().now()
        in_collision_hold = now < self._collision_hold_until
        switch_on = bool(self.get_parameter('switch').value)
        manual_or_safety = self.joy_active or in_collision_hold
        enabled = switch_on and not manual_or_safety
        linear = float(self.get_parameter('linear').value)
        if in_collision_hold:
            self.pid.reset()

        # Manual override or the safety stop cancel any QR maneuver in flight.
        if manual_or_safety and self._qr_state is not None:
            self._qr_state = None

        # QR priority: scan the frame and (re)arm a fork-road maneuver.
        # enable_qr is re-read every tick so `ros2 param set` takes effect live.
        enable_qr = bool(self.get_parameter('enable_qr').value)
        if enable_qr and self._qr is not None and not manual_or_safety:
            self._maybe_detect_qr(frame, now)

        # Explain (throttled) why we are not driving, so a "stuck" robot is
        # diagnosable from the logs.
        if not enabled:
            if in_collision_hold:
                reason = 'collision hold-off'
            elif self.joy_active:
                reason = 'joystick takeover (/JoyState)'
            else:
                reason = 'switch parameter is false'
            self.get_logger().info(
                f'not driving: {reason}', throttle_duration_sec=3.0)

        qr_label = ''
        if self._qr_state is not None and not manual_or_safety:
            # The QR maneuver owns /cmd_vel; the line PID is bypassed this tick.
            twist, still = self._qr_maneuver_twist()
            qr_label = self._qr_state['type']
            if not still:
                self.get_logger().info(
                    f'QR {qr_label} maneuver done -- resume line follow')
                self._publish_qr('resume')
                self._qr_state = None
                self.pid.reset()
        elif centroid is not None:
            cx, cy, area, cnt = centroid
            # normalized lateral error in [-1, 1]; positive = line is to the right
            err = (cx - self.w / 2.0) / (self.w / 2.0)
            ang = self.pid.step(err)
            if not self._tracking:
                self.get_logger().info('line acquired')
                self._tracking = True
            # near the frame edge the line is about to leave view
            if abs(err) > 0.85:
                self.get_logger().warn(
                    f'line near frame edge (err={err:+.2f}); may be lost soon '
                    f'-- slow down or widen the camera view',
                    throttle_duration_sec=2.0)
            if debug:
                self.get_logger().info(
                    f'err={err:+.2f} cx={cx} area={area:.0f} blobs={blobs} '
                    f'ang={ang:+.2f} enabled={enabled}',
                    throttle_duration_sec=0.5)
            if enabled:
                twist.linear.x = linear
                twist.angular.z = float(-ang)  # turn toward the line
            if self.show:
                cv.drawContours(band, [cnt], -1, (0, 255, 0), 2)
                cv.circle(band, (cx, cy), 6, (0, 0, 255), -1)
                cv.putText(frame, f'err={err:+.2f} ang={ang:+.2f}',
                           (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5,
                           (0, 255, 255), 1)
        else:
            self.pid.reset()
            if self._tracking:
                self.get_logger().warn(
                    'line lost -- no blob >= min_area in ROI; stopping steering',
                    throttle_duration_sec=2.0)
                self._tracking = False
            else:
                self.get_logger().warn(
                    'no line detected -- check HSV / lighting / camera_pitch / '
                    'ROI band', throttle_duration_sec=3.0)
            if self.show:
                cv.putText(frame, 'no line', (10, 20),
                           cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        if self.show and qr_label:
            cv.putText(frame, f'QR: {qr_label}', (10, 80),
                       cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
        if self.show and in_collision_hold:
            cv.putText(frame, 'COLLISION HOLD', (10, 60),
                       cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        self.pub_cmd.publish(twist)

        if self.show:
            cv.rectangle(frame, (0, y0), (self.w - 1, y1 - 1),
                         (255, 255, 0), 1)
            bin_bgr = cv.cvtColor(binary, cv.COLOR_GRAY2BGR)
            bin_full = cv.copyMakeBorder(
                bin_bgr, y0, self.h - y1, 0, 0,
                cv.BORDER_CONSTANT, value=(0, 0, 0))
            view = cv.hconcat([frame, bin_full])
            cv.imshow(WINDOW, view)
            key = cv.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                self._shutdown()

    def _shutdown(self):
        try:
            self.pub_cmd.publish(Twist())
            if self.cap is not None:
                self.cap.release()
            if self.show:
                cv.destroyAllWindows()
        finally:
            rclpy.shutdown()


def main():
    rclpy.init()
    node = LineTrack()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._shutdown()


if __name__ == '__main__':
    main()
