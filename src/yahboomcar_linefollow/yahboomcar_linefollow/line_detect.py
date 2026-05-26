#!/usr/bin/env python3
# encoding: utf-8
"""Line-follow DETECT mode.

Open the camera, let the user drag a rectangle on the live frame to pick
a sample of the line. Compute the HSV inRange that covers those pixels
(plus a small padding) and immediately preview the resulting binary mask.

The frame source is either a local USB camera (camera_index, default) or a
ROS sensor_msgs/Image topic (image_topic, e.g. Gazebo's /camera/image_raw),
so you can box-select on the simulated image just like on a real camera.

Keys (focus the OpenCV window):
    Mouse left-drag : select ROI (auto-learn HSV)
    s               : save current HSV to the params file
    r               : reset (clear HSV, pick again)
    q / ESC         : quit
"""

import os
import time

import cv2 as cv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory

from .line_common import (
    hsv_from_roi,
    mask_with_hsv,
    largest_contour_centroid,
    read_hsv,
    write_hsv,
)

WINDOW = 'line_detect'


def _default_hsv_path():
    """Persistent HSV file inside the package's share/params directory.

    With `colcon build --symlink-install` (the common dev workflow) this
    points back to the source tree, so writes survive across runs and can
    be committed to git. Without symlink-install it lives in install
    space and gets reset on the next colcon build -- so prefer the
    symlinked workflow if you want HSV.txt to be sticky.
    """
    return os.path.join(
        get_package_share_directory('yahboomcar_linefollow'),
        'params', 'HSV.txt')


class LineDetect(Node):
    def __init__(self):
        super().__init__('line_detect')

        # parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('image_topic', '')
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('hsv_file', _default_hsv_path())
        self.declare_parameter('autosave', True)

        self.cam_index = self.get_parameter('camera_index').value
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.w = int(self.get_parameter('frame_width').value)
        self.h = int(self.get_parameter('frame_height').value)
        self.hsv_file = self.get_parameter('hsv_file').value

        self.pub_status = self.create_publisher(String, '~/status', 10)

        # ROI selection state
        self.dragging = False
        self.drag_start = (0, 0)
        self.roi = None      # (x0,y0,x1,y1)
        self.hsv_range = read_hsv(self.hsv_file)  # may already exist

        # image source: a ROS topic (simulation / external driver) or a local
        # USB camera opened directly with OpenCV.
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

        cv.namedWindow(WINDOW, cv.WINDOW_AUTOSIZE)
        cv.setMouseCallback(WINDOW, self._on_mouse)

        self._t_prev = time.time()
        self.timer = self.create_timer(0.03, self._tick)

    # ---- mouse ----
    def _on_mouse(self, event, x, y, flags, _param):
        if event == cv.EVENT_LBUTTONDOWN:
            self.dragging = True
            self.drag_start = (x, y)
            self.roi = (x, y, x, y)
        elif event == cv.EVENT_MOUSEMOVE and self.dragging:
            x0, y0 = self.drag_start
            self.roi = (min(x0, x), min(y0, y), max(x0, x), max(y0, y))
        elif event == cv.EVENT_LBUTTONUP and self.dragging:
            self.dragging = False
            x0, y0 = self.drag_start
            self.roi = (min(x0, x), min(y0, y), max(x0, x), max(y0, y))
            self._learn_from_roi()

    def _learn_from_roi(self):
        if self.roi is None:
            return
        if self._last_frame is None:
            # e.g. an image_topic was given but no frame has arrived yet
            self.get_logger().warn('no frame yet; wait for the camera, '
                                   'then re-select the ROI')
            return
        rng = hsv_from_roi(self._last_frame, self.roi)
        if rng:
            self.hsv_range = rng
            lo, hi = rng
            self.get_logger().info(
                f'learned HSV  lo={lo}  hi={hi}')
            self._publish_status(f'learned {lo}-{hi}')
            # Persist immediately so the same HSV.txt is ready for the
            # tracker / safe launch without any extra keystroke.
            if bool(self.get_parameter('autosave').value):
                self._save_hsv()

    def _save_hsv(self):
        if not self.hsv_range:
            return
        try:
            os.makedirs(os.path.dirname(self.hsv_file), exist_ok=True)
            write_hsv(self.hsv_file, self.hsv_range)
            self.get_logger().info(f'HSV.txt updated -> {self.hsv_file}')
            self._publish_status(f'saved {self.hsv_file}')
        except OSError as e:
            self.get_logger().error(f'cannot write {self.hsv_file}: {e}')

    def _publish_status(self, text):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)

    # ---- image source ----
    _last_frame = None

    def _on_image(self, msg):
        try:
            self._frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'cv_bridge convert failed: {exc}')

    def _grab(self):
        if self.cap is not None:
            return self.cap.read()
        if self._frame is None:
            return False, None
        return True, self._frame

    def _tick(self):
        ok, frame = self._grab()
        if not ok or frame is None:
            return
        frame = cv.resize(frame, (self.w, self.h))
        self._last_frame = frame.copy()

        # binary preview
        if self.hsv_range:
            binary = mask_with_hsv(frame, self.hsv_range)
            centroid = largest_contour_centroid(binary)
            if centroid:
                cx, cy, _area, cnt = centroid
                cv.drawContours(frame, [cnt], -1, (0, 255, 0), 2)
                cv.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
        else:
            binary = None

        # ROI rectangle overlay
        if self.roi is not None:
            x0, y0, x1, y1 = self.roi
            cv.rectangle(frame, (x0, y0), (x1, y1), (255, 255, 0), 2)

        # HUD
        if self.hsv_range:
            lo, hi = self.hsv_range
            cv.putText(frame, f'lo {lo}', (10, 20),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv.putText(frame, f'hi {hi}', (10, 40),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv.putText(frame, 'drag=ROI  s=save  r=reset  q=quit',
                   (10, self.h - 10),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        if binary is not None:
            bin_bgr = cv.cvtColor(binary, cv.COLOR_GRAY2BGR)
            view = cv.hconcat([frame, bin_bgr])
        else:
            view = frame
        cv.imshow(WINDOW, view)

        key = cv.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            self._shutdown()
        elif key == ord('s'):
            self._save_hsv()
        elif key == ord('r'):
            self.hsv_range = ()
            self.roi = None
            self.get_logger().info('reset HSV; select a new ROI')

    def _shutdown(self):
        try:
            if self.cap is not None:
                self.cap.release()
            cv.destroyAllWindows()
        finally:
            rclpy.shutdown()


def main():
    rclpy.init()
    node = LineDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._shutdown()


if __name__ == '__main__':
    main()
