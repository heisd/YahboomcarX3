#!/usr/bin/env python
# coding:utf-8
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class scan_compression(Node):
    def __init__(self,name):
        super().__init__(name)
        self.multiple = 2
        self.pub = self.create_publisher(LaserScan, "/downsampled_scan", 10)
        self.laserSub = self.create_subscription(LaserScan,"/scan", self.laserCallback, 10)

    def laserCallback(self, data):
        # self.get_logger().info("laserCallback")
        if not isinstance(data, LaserScan): return
        laser_scan = LaserScan()
        # Preserve the original scan timestamp so downstream consumers keep a
        # consistent time reference (works under both real and simulated clocks).
        laser_scan.header.stamp = data.header.stamp
        laser_scan.header.frame_id = data.header.frame_id
        laser_scan.angle_increment = data.angle_increment * self.multiple
        laser_scan.time_increment = data.time_increment * self.multiple
        laser_scan.scan_time = data.scan_time
        laser_scan.angle_min = data.angle_min
        # Keep angle_max consistent with the down-sampled point count.
        laser_scan.angle_max = data.angle_min + \
            (len(data.ranges[::self.multiple]) - 1) * laser_scan.angle_increment
        laser_scan.range_min = data.range_min
        laser_scan.range_max = data.range_max
        # Down-sample ranges and intensities together so the arrays stay aligned.
        laser_scan.ranges = data.ranges[::self.multiple]
        if data.intensities:
            laser_scan.intensities = data.intensities[::self.multiple]
        self.pub.publish(laser_scan)

def main():
    rclpy.init()
    scan_cp = scan_compression("scan_dilute")
    rclpy.spin(scan_cp)
    scan_cp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
