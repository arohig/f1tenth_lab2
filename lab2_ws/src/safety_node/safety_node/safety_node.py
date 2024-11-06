#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """

    def __init__(self):
        super().__init__("safety_node")
        self.get_logger().info(f"Initializating safety node")
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """

        self.ttc_threshold = 1.5  # seconds

        # Subscriber to the /scan topic
        self.scan_subscriber = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 100  # QoS history depth
        )
        self.get_logger().info("Subscribed to scan topic")

        # Subscriber to the /odom topic
        self.odom_subscriber = self.create_subscription(
            Odometry, "/ego_racecar/odom", self.odom_callback, 100  # QoS history depth
        )
        self.get_logger().info("Subscribed to odometry topic")

        # Publisher to the /drive topic in the event of an emergency brake
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, "drive", 10)

        self.speed = 0.0

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # self.get_logger().info(f"Received LaserScan with {scan_msg.ranges} ranges")
        eb = False

        # Go through each range and calculate iTTC
        for i, range_distance in enumerate(scan_msg.ranges):
            # Ignore out-of-range measurements
            if (
                range_distance <= scan_msg.range_min
                or range_distance >= scan_msg.range_max
            ):
                continue

            # Calculate the angle for this specific range measurement
            angle = scan_msg.angle_min + i * scan_msg.angle_increment

            # Calculate range rate for this angle (assuming forward velocity component)
            range_rate = max(self.speed * np.cos(angle), 0.001)

            ttc = range_distance / range_rate
            if ttc < self.ttc_threshold:
                eb = True
                self.get_logger().warn(
                    f"Collision likely! TTC: {ttc:.2f}s at angle {np.degrees(angle):.1f}"
                )
                break

            # Trigger emergency braking
        if eb:
            brake_cmd = AckermannDriveStamped()
            brake_cmd.drive.speed = 0.0
            self.drive_publisher.publish(
                brake_cmd
            )  # No need to continue checking other ranges if we're already braking

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
