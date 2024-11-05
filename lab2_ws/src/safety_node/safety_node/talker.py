import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        
        # Declare ROS parameters for speed (v) and steering angle (d)
        self.declare_parameter('v', 1.0)  # Initial speed
        self.declare_parameter('d', 0.0)  # Initial steering angle

        # Create a publisher for the 'drive' topic
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)

        # Publish messages at a high frequency (100 Hz)
        self.timer = self.create_timer(0.01, self.publish_drive)

    def publish_drive(self):
        # Retrieve the parameters for speed and steering angle
        v = self.get_parameter('v').get_parameter_value().double_value
        d = self.get_parameter('d').get_parameter_value().double_value

        # Create a message with speed and steering angle values
        msg = AckermannDriveStamped()
        msg.drive.speed = v
        msg.drive.steering_angle = d

        # Publish the message to the 'drive' topic
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: speed={v}, steering_angle={d}')

def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()