#!/usr/bin/env python3
# encoding: utf-8
"""QR Check -- standalone diagnostic node.

Open the camera, decode any QR code in view, resolve it against the JSON
action table, and preview both live. Publishes the decoded payload and the
resolved action type on topics so the rest of the system (or an operator)
can see what each code maps to.

This node is the bench-test / calibration counterpart to the QR priority
logic that runs *inside* line_track at runtime (analogous to how line_detect
is the diagnostic counterpart of line_track). Because a single USB camera
cannot be opened by two processes at once, run this node on its own -- not
alongside line_track.

Topics:
    publishes  ~/payload  (std_msgs/String)  decoded QR text ('' when none)
    publishes  ~/action   (std_msgs/String)  resolved action type ('' when none)

Keys (focus the OpenCV window):
    q / ESC : quit
"""

import os

import cv2 as cv
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory

from .qr_common import QRReader, load_actions, resolve_action

WINDOW = 'qr_check'


def _default_actions_path():
    return os.path.join(
        get_package_share_directory('yahboomcar_linefollow'),
        'params', 'qr_actions.json')


class QRCheck(Node):
    def __init__(self):
        super().__init__('qr_check')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('qr_actions_file', _default_actions_path())
        self.declare_parameter('show_window', True)

        self.cam_index = self.get_parameter('camera_index').value
        self.w = int(self.get_parameter('frame_width').value)
        self.h = int(self.get_parameter('frame_height').value)
        self.show = bool(self.get_parameter('show_window').value)

        actions_file = self.get_parameter('qr_actions_file').value
        self.table = load_actions(actions_file)
        n = len(self.table.get('actions', {}))
        self.get_logger().info(
            f'loaded {n} QR action(s) from {actions_file}')

        self.qr = QRReader()
        self.pub_payload = self.create_publisher(String, '~/payload', 10)
        self.pub_action = self.create_publisher(String, '~/action', 10)

        self.cap = cv.VideoCapture(self.cam_index)
        if not self.cap.isOpened():
            self.get_logger().error(f'cannot open camera index {self.cam_index}')
            raise RuntimeError('camera open failed')
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, self.w)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.h)

        if self.show:
            cv.namedWindow(WINDOW, cv.WINDOW_AUTOSIZE)

        self._last_payload = None
        self.timer = self.create_timer(0.05, self._tick)

    def _tick(self):
        ok, frame = self.cap.read()
        if not ok:
            return
        frame = cv.resize(frame, (self.w, self.h))

        payload, points = self.qr.detect(frame)
        action = resolve_action(payload, self.table) if payload else None
        a_type = action['type'] if action else ''

        # Only publish/log on a change so we don't spam the topics every tick.
        if payload != self._last_payload:
            self._last_payload = payload
            self.pub_payload.publish(String(data=payload or ''))
            self.pub_action.publish(String(data=a_type))
            if payload:
                matched = a_type if action else 'NO MATCH'
                self.get_logger().info(f'QR "{payload}" -> {matched}')

        if self.show:
            if points is not None:
                pts = points.reshape(-1, 2).astype(int)
                for i in range(len(pts)):
                    cv.line(frame, tuple(pts[i]), tuple(pts[(i + 1) % len(pts)]),
                            (0, 255, 0), 2)
            label = f'{payload} -> {a_type}' if payload else 'no QR'
            cv.putText(frame, label, (10, 24),
                       cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv.imshow(WINDOW, frame)
            key = cv.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                self._shutdown()

    def _shutdown(self):
        try:
            self.cap.release()
            if self.show:
                cv.destroyAllWindows()
        finally:
            rclpy.shutdown()


def main():
    rclpy.init()
    node = QRCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._shutdown()


if __name__ == '__main__':
    main()
