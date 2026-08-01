#!/usr/bin/env python3
"""Measured-health adapter for D435i RTAB-Map visual odometry.

It publishes only source health evidence; PX4 VehicleOdometry remains owned by
vision_to_dds_node and is disabled by default in the accompanying launch.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int8, UInt32


class VioHealthAdapter(Node):
    def __init__(self):
        super().__init__('d435i_vio_health_adapter')
        self.declare_parameter('odometry_topic', '/d435i_vio/odometry')
        self.declare_parameter('quality_topic', '/vision/quality')
        self.declare_parameter('source_epoch_topic', '/vision/source_epoch')
        self.declare_parameter('stream_timeout_s', 0.20)
        self.declare_parameter('maximum_position_variance', 0.04)
        self.declare_parameter('maximum_orientation_variance', 0.09)
        self.declare_parameter('health_rate_hz', 20.0)

        self.odometry_topic = self.get_parameter('odometry_topic').value
        self.quality_topic = self.get_parameter('quality_topic').value
        self.source_epoch_topic = self.get_parameter('source_epoch_topic').value
        self.timeout_us = int(float(self.get_parameter('stream_timeout_s').value) * 1_000_000)
        self.maximum_position_variance = float(
            self.get_parameter('maximum_position_variance').value)
        self.maximum_orientation_variance = float(
            self.get_parameter('maximum_orientation_variance').value)
        health_rate_hz = float(self.get_parameter('health_rate_hz').value)
        if (not self.odometry_topic or not self.quality_topic or not self.source_epoch_topic or
                self.timeout_us <= 0 or self.maximum_position_variance <= 0.0 or
                self.maximum_orientation_variance <= 0.0 or health_rate_hz <= 0.0):
            raise RuntimeError('invalid D435i VIO health configuration')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT)
        self.quality_publisher = self.create_publisher(Int8, self.quality_topic, qos)
        self.epoch_publisher = self.create_publisher(UInt32, self.source_epoch_topic, qos)
        self.subscription = self.create_subscription(Odometry, self.odometry_topic, self.on_odometry, qos)
        self.source_epoch = 1
        self.have_source = False
        self.timed_out = False
        self.last_stamp_us = 0
        self.last_received_us = 0
        self.quality = 0
        self.create_timer(1.0 / health_rate_hz, self.on_timer)
        self.publish()

    def now_us(self):
        return self.get_clock().now().nanoseconds // 1_000

    def advance_epoch(self):
        self.source_epoch = (self.source_epoch + 1) & 0xffffffff
        if self.source_epoch == 0:
            self.source_epoch = 1

    def measured_quality(self, covariance):
        position = (covariance[0], covariance[7], covariance[14])
        orientation = (covariance[21], covariance[28], covariance[35])
        values = position + orientation
        if any(not math.isfinite(value) or value <= 0.0 for value in values):
            return 0
        ratio = max(
            *(value / self.maximum_position_variance for value in position),
            *(value / self.maximum_orientation_variance for value in orientation))
        if not math.isfinite(ratio) or ratio >= 1.0:
            return 0
        return int(round(100.0 * (1.0 - ratio)))

    def on_odometry(self, message):
        stamp_us = message.header.stamp.sec * 1_000_000 + message.header.stamp.nanosec // 1_000
        now_us = self.now_us()
        if self.timed_out:
            self.advance_epoch()
            self.timed_out = False
            self.last_stamp_us = 0
        if stamp_us == 0 or (self.last_stamp_us and stamp_us <= self.last_stamp_us):
            if self.last_stamp_us and stamp_us < self.last_stamp_us:
                self.advance_epoch()
                self.timed_out = True
                self.last_stamp_us = 0
            self.have_source = True
            self.quality = 0
            self.publish()
            return
        self.have_source = True
        self.last_stamp_us = stamp_us
        self.last_received_us = now_us
        self.quality = self.measured_quality(message.pose.covariance)
        self.publish()

    def on_timer(self):
        if self.have_source and self.last_received_us and self.now_us() > self.last_received_us + self.timeout_us:
            self.quality = 0
            self.timed_out = True
        self.publish()

    def publish(self):
        epoch = UInt32()
        epoch.data = self.source_epoch
        self.epoch_publisher.publish(epoch)
        # Do not fabricate a quality measurement before VIO emits its first
        # odometry sample. Once a source was observed, zero is the fail-closed
        # value for stale, frozen, invalid or high-variance odometry.
        if self.have_source:
            quality = Int8()
            quality.data = self.quality
            self.quality_publisher.publish(quality)


def main():
    rclpy.init()
    node = VioHealthAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
