#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
import math

class AckermannToTwist(Node):
    def __init__(self):
        super().__init__('ackermann_to_twist')
        self.declare_parameter('wheelbase', 0.325)  # Match plugin and controller
        self.wheelbase = self.get_parameter('wheelbase').value
    
        self.subscriber = self.create_subscription(
            AckermannDriveStamped,
            '/ackermann_cmd',
            self.callback,
            10
        )
        self.publisher = self.create_publisher(
            Twist,
            '/car_1/cmd_vel',
            10
        )
        self.get_logger().info('AckermannToTwist node started')

    def callback(self, msg):
        twist_msg = Twist()
        twist_msg.linear.x = msg.drive.speed
        twist_msg.angular.z = msg.drive.speed * math.tan(msg.drive.steering_angle) / self.wheelbase
        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AckermannToTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()