#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
import math
from sensor_msgs.msg import Joy

class AckermannToTwist(Node):
    def __init__(self):
        super().__init__('ackermann_to_twist')
        self.declare_parameter('wheelbase', 0.325)  # Match plugin and controller
        self.wheelbase = self.get_parameter('wheelbase').value
        self.sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)

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
        
        self.axis_throttle = 1  # left stick vertical
        self.axis_steering = 3  # right stick horizontal
        self.max_speed = 2.0
        self.max_steer = 0.2
        self.deadman_button = 5  # R1

        self.get_logger().info('AckermannToTwist node started')

    def callback(self, msg):
        twist_msg = Twist()
        twist_msg.linear.x = msg.drive.speed
        twist_msg.angular.z = msg.drive.speed * math.tan(msg.drive.steering_angle) / self.wheelbase
        self.publisher.publish(twist_msg)
        
   
    def joy_callback(self, msg):
        if len(msg.buttons) > self.deadman_button and msg.buttons[self.deadman_button]:
            speed = -msg.axes[self.axis_throttle] * self.max_speed
            steering = msg.axes[self.axis_steering] * self.max_steer

            self.get_logger().info(f"Publishing: speed={speed:.2f}, steering={steering:.2f}")

            cmd = AckermannDriveStamped()
            cmd.drive.speed = speed
            cmd.drive.steering_angle = steering
            self.pub.publish(cmd)
     

def main(args=None):
    rclpy.init(args=args)
    node = AckermannToTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()