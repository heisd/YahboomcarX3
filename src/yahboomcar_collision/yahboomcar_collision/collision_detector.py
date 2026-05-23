#!/usr/bin/env python3
# encoding: utf-8
"""IMU-based collision detector.

Strategy (designed to keep false positives low):
  1. Track a slow EMA baseline of the IMU linear acceleration -- this absorbs
     gravity and any steady offset regardless of mounting orientation.
  2. The "shock" signal is  s = || a - baseline ||.  A real collision shows up
     as a sharp jump in s; vehicle vibrations and gentle handling do not.
  3. Optionally also gate on a sharp jump in angular velocity || w || .
  4. A trigger is only emitted when s (and optionally w) stay above their
     thresholds for `min_trigger_samples` consecutive IMU messages
     (debounce).
  5. After a trigger, the detector enters a cooldown for `cooldown_sec`
     seconds during which no new trigger is emitted.  This avoids reporting
     the same impact many times as it rings out.

Topics:
  sub:  <imu_topic>       sensor_msgs/Imu       (default 'imu/data_raw')
  pub:  ~/collision       std_msgs/Bool         (pulse True on detection)
  pub:  ~/shock           std_msgs/Float32      (current shock magnitude)
  pub:  /cmd_vel          geometry_msgs/Twist   (zero Twist on detection,
                                                 only if stop_on_collision)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist


class CollisionDetector(Node):
    def __init__(self):
        super().__init__('collision_detector')

        # ---- parameters (all overridable from launch / CLI) ----
        # Defaults are deliberately conservative (high) to reduce false
        # positives; tune lower if you want a more sensitive bumper.
        self.declare_parameter('imu_topic', 'imu/data_raw')
        self.declare_parameter('accel_threshold', 12.0)   # m/s^2 over baseline
        self.declare_parameter('gyro_threshold', 6.0)     # rad/s over baseline
        self.declare_parameter('use_gyro', False)         # require gyro spike too
        self.declare_parameter('baseline_alpha', 0.02)    # EMA weight for baseline
        self.declare_parameter('min_trigger_samples', 2)  # consecutive frames
        self.declare_parameter('cooldown_sec', 1.5)
        self.declare_parameter('stop_on_collision', True)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        p = self.get_parameter
        self.imu_topic         = p('imu_topic').value
        self.cmd_vel_topic     = p('cmd_vel_topic').value
        self.stop_on_collision = bool(p('stop_on_collision').value)

        # ---- I/O ----
        self.sub_imu = self.create_subscription(
            Imu, self.imu_topic, self._on_imu, qos_profile_sensor_data)
        self.pub_flag = self.create_publisher(Bool, '~/collision', 10)
        self.pub_shock = self.create_publisher(Float32, '~/shock', 10)
        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, 10) \
            if self.stop_on_collision else None

        # ---- runtime state ----
        self._a_base = None        # baseline linear acceleration (3-vec)
        self._w_base = None        # baseline angular velocity (3-vec)
        self._over_streak = 0
        self._cooldown_until = self.get_clock().now()

        self.get_logger().info(
            f'collision_detector listening on "{self.imu_topic}", '
            f'accel_th={p("accel_threshold").value} m/s^2, '
            f'gyro_th={p("gyro_threshold").value} rad/s '
            f'(use_gyro={p("use_gyro").value}), '
            f'min_samples={p("min_trigger_samples").value}, '
            f'cooldown={p("cooldown_sec").value}s, '
            f'stop_on_collision={self.stop_on_collision}')

    # ---- helpers ----

    @staticmethod
    def _norm(v):
        return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])

    @staticmethod
    def _ema(prev, cur, alpha):
        return tuple(prev[i] + alpha * (cur[i] - prev[i]) for i in range(3))

    # ---- main callback ----

    def _on_imu(self, msg):
        a = (msg.linear_acceleration.x,
             msg.linear_acceleration.y,
             msg.linear_acceleration.z)
        w = (msg.angular_velocity.x,
             msg.angular_velocity.y,
             msg.angular_velocity.z)

        # Read params each tick so they can be tuned live with ros2 param set.
        accel_th = float(self.get_parameter('accel_threshold').value)
        gyro_th  = float(self.get_parameter('gyro_threshold').value)
        use_gyro = bool(self.get_parameter('use_gyro').value)
        alpha    = float(self.get_parameter('baseline_alpha').value)
        min_n    = int(self.get_parameter('min_trigger_samples').value)
        cooldown = float(self.get_parameter('cooldown_sec').value)

        # Initialise baselines on the first sample so we don't trigger from
        # the gravity vector itself.
        if self._a_base is None:
            self._a_base = a
            self._w_base = w
            return

        # Shock signal = magnitude of acceleration deviation from baseline.
        da = (a[0] - self._a_base[0],
              a[1] - self._a_base[1],
              a[2] - self._a_base[2])
        dw = (w[0] - self._w_base[0],
              w[1] - self._w_base[1],
              w[2] - self._w_base[2])
        shock_a = self._norm(da)
        shock_w = self._norm(dw)

        self.pub_shock.publish(Float32(data=float(shock_a)))

        in_cooldown = self.get_clock().now() < self._cooldown_until

        accel_hit = shock_a >= accel_th
        gyro_hit  = shock_w >= gyro_th
        condition = accel_hit and (gyro_hit if use_gyro else True)

        if condition and not in_cooldown:
            self._over_streak += 1
        else:
            self._over_streak = 0

        # Only update the baseline when we are NOT inside an apparent shock,
        # so a hard impact doesn't get smoothed into the baseline and missed.
        if not condition:
            self._a_base = self._ema(self._a_base, a, alpha)
            self._w_base = self._ema(self._w_base, w, alpha)

        if self._over_streak >= min_n and not in_cooldown:
            self._trigger(shock_a, shock_w, cooldown)
            self._over_streak = 0

    def _trigger(self, shock_a, shock_w, cooldown_sec):
        self.get_logger().warn(
            f'COLLISION detected  |Δa|={shock_a:.2f} m/s^2  '
            f'|Δw|={shock_w:.2f} rad/s')
        self.pub_flag.publish(Bool(data=True))
        if self.pub_cmd is not None:
            self.pub_cmd.publish(Twist())  # zero Twist = stop
        dt = rclpy.duration.Duration(
            seconds=int(cooldown_sec),
            nanoseconds=int((cooldown_sec % 1) * 1e9))
        self._cooldown_until = self.get_clock().now() + dt


def main():
    rclpy.init()
    node = CollisionDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
