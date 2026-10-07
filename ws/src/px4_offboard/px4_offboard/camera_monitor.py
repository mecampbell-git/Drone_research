#!/usr/bin/env python3
"""Subscribe to the bridged Gazebo camera and log the received frame rate."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class CameraMonitor(Node):
    """Receive /camera/image_raw reliably; image processing goes in image_callback."""

    def __init__(self) -> None:
        super().__init__('camera_monitor')

        # RELIABLE, not sensor-data BEST_EFFORT: a 1280x960 rgb8 frame is ~3.7 MB and is
        # split into many DDS fragments. With BEST_EFFORT one lost fragment drops the whole
        # frame (measured 11-15 Hz of 30); RELIABLE re-requests fragments and gets all 30 Hz.
        # Keep depth small so a slow callback drops old frames instead of building a backlog.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2
        )

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('report_period', 5.0)
        topic = self.get_parameter('image_topic').value

        self.image_subscriber = self.create_subscription(
            Image, topic, self.image_callback, qos_profile)

        self.frame_count = 0
        self.last_size = None
        self.last_report = time.monotonic()
        self.create_timer(self.get_parameter('report_period').value, self.report_rate)
        self.get_logger().info(f'Monitoring {topic}')

    def image_callback(self, msg: Image) -> None:
        """Handle one camera frame."""
        self.frame_count += 1
        self.last_size = (msg.width, msg.height, msg.encoding)

    def report_rate(self) -> None:
        """Log the frame rate received since the last report."""
        now = time.monotonic()
        rate = self.frame_count / (now - self.last_report)
        if self.frame_count:
            width, height, encoding = self.last_size
            self.get_logger().info(f'{rate:.1f} Hz ({width}x{height} {encoding})')
        else:
            self.get_logger().warn('No camera frames received')
        self.frame_count = 0
        self.last_report = now


def main(args=None) -> None:
    rclpy.init(args=args)
    camera_monitor = CameraMonitor()
    try:
        rclpy.spin(camera_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        camera_monitor.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
