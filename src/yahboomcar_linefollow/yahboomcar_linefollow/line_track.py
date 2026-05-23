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
    camera_index        int   default 0
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
from std_msgs.msg import Bool

from ament_index_python.packages import get_package_share_directory

from .line_common import (
    read_hsv,
    mask_with_hsv,
    largest_contour_centroid,
    SimplePID,
)

WINDOW = 'line_track'


def _default_hsv_path():
    return os.path.join(
        get_package_share_directory('yahboomcar_linefollow'),
        'params', 'HSV.txt')


class LineTrack(Node):
    def __init__(self):
        super().__init__('line_track')

        self.declare_parameter('camera_index', 0)
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
        # Glue with yahboomcar_collision: when a True pulse arrives on
        # `collision_topic`, stop publishing motion for `collision_pause_sec`
        # seconds and reset the PID. Set the topic empty to disable.
        self.declare_parameter('collision_topic',
                               '/collision_detector/collision')
        self.declare_parameter('collision_pause_sec', 2.0)

        self.cam_index = self.get_parameter('camera_index').value
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

        self.pid = SimplePID(
            kp=float(self.get_parameter('kp').value),
            ki=float(self.get_parameter('ki').value),
            kd=float(self.get_parameter('kd').value),
            i_clamp=1.0,
            out_clamp=float(self.get_parameter('angular_max').value),
        )

        self.cap = cv.VideoCapture(self.cam_index)
        if not self.cap.isOpened():
            self.get_logger().error(f'cannot open camera index {self.cam_index}')
            raise RuntimeError('camera open failed')
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, self.w)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.h)

        if self.show:
            cv.namedWindow(WINDOW, cv.WINDOW_AUTOSIZE)

        self.timer = self.create_timer(0.03, self._tick)
        self._t_prev = time.time()

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

    def _refresh_params(self):
        self.pid.kp = float(self.get_parameter('kp').value)
        self.pid.ki = float(self.get_parameter('ki').value)
        self.pid.kd = float(self.get_parameter('kd').value)
        self.pid.out_clamp = float(self.get_parameter('angular_max').value)

    def _tick(self):
        ok, frame = self.cap.read()
        if not ok:
            return
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

        binary = mask_with_hsv(band, self.hsv_range)
        centroid = largest_contour_centroid(binary)

        twist = Twist()
        in_collision_hold = self.get_clock().now() < self._collision_hold_until
        enabled = (bool(self.get_parameter('switch').value)
                   and not self.joy_active
                   and not in_collision_hold)
        linear = float(self.get_parameter('linear').value)
        if in_collision_hold:
            self.pid.reset()

        if centroid is not None:
            cx, cy, _area, cnt = centroid
            # normalized lateral error in [-1, 1]; positive = line is to the right
            err = (cx - self.w / 2.0) / (self.w / 2.0)
            ang = self.pid.step(err)
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
            if self.show:
                cv.putText(frame, 'no line', (10, 20),
                           cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

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
