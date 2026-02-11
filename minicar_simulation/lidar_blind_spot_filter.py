#!/usr/bin/env python3
"""
LiDAR blind spot filter node.

Filters LaserScan messages to mark ranges in the blind spot as invalid (inf).
This simulates the real robot's LiDAR being blocked by the robot body.

Blind spot: 115° to 246° (back of robot)
- 0° = front
- 90° = left
- 180° = back
- 270° = right
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarBlindSpotFilter(Node):
    """Filter node that marks blind spot ranges as invalid."""

    def __init__(self):
        super().__init__('lidar_blind_spot_filter')

        # Blind spot parameters (in degrees)
        self.declare_parameter('blind_spot_start_deg', 115.0)
        self.declare_parameter('blind_spot_end_deg', 246.0)
        self.declare_parameter('input_topic', 'scan_raw')
        self.declare_parameter('output_topic', 'scan')

        self.blind_start_deg = self.get_parameter('blind_spot_start_deg').value
        self.blind_end_deg = self.get_parameter('blind_spot_end_deg').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # Convert to radians
        self.blind_start_rad = math.radians(self.blind_start_deg)
        self.blind_end_rad = math.radians(self.blind_end_deg)

        self.get_logger().info(
            f'Blind spot filter: {self.blind_start_deg}° to {self.blind_end_deg}° '
            f'({self.blind_start_rad:.3f} to {self.blind_end_rad:.3f} rad)'
        )
        self.get_logger().info(f'Subscribing to: {input_topic}, Publishing to: {output_topic}')

        # Subscriber and publisher
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            10
        )
        self.publisher = self.create_publisher(LaserScan, output_topic, 10)

    def scan_callback(self, msg: LaserScan):
        """Filter scan and mark blind spot as invalid."""
        # Create a copy of the message
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max

        # Copy ranges and mark blind spot as inf
        ranges = list(msg.ranges)
        intensities = list(msg.intensities) if msg.intensities else []

        for i in range(len(ranges)):
            # Calculate angle for this sample
            angle = msg.angle_min + i * msg.angle_increment

            # Normalize angle to [0, 2π)
            while angle < 0:
                angle += 2 * math.pi
            while angle >= 2 * math.pi:
                angle -= 2 * math.pi

            # Check if in blind spot
            if self.blind_start_rad <= angle <= self.blind_end_rad:
                ranges[i] = float('inf')
                if intensities:
                    intensities[i] = 0.0

        filtered_msg.ranges = ranges
        filtered_msg.intensities = intensities

        self.publisher.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarBlindSpotFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
