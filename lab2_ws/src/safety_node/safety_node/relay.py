import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class Relay(Node):
    def __init__(self):
        super().__init__('relay')
        # Subscribe to the 'drive' topic
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            'drive',
            self.listener_callback,
            10
        )

        # Create publisher for 'drive_relay' topic (processed output)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive_relay', 10)

    def listener_callback(self, msg):
        # Modify speed and steering angle by multiplying them by 3 (example processing)
        processed_msg = AckermannDriveStamped()
        processed_msg.drive.speed = msg.drive.speed * 3
        processed_msg.drive.steering_angle = msg.drive.steering_angle * 3

        # Publish modified values to the 'drive_relay' topic
        self.publisher_.publish(processed_msg)
        self.get_logger().info(f'Relaying: speed={processed_msg.drive.speed}, steering_angle={processed_msg.drive.steering_angle}')

def main(args=None):
    rclpy.init(args=args)
    node = Relay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()