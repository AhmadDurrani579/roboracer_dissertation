#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from ackermann_msgs.msg import AckermannDriveStamped
import math

class AckermannRelayNode(Node):
    def __init__(self):
        super().__init__("twist_relay.py")

        self.wheelbase = 0.325  # adjust as needed

        # Ackermann → Twist
        self.ackermann_sub = self.create_subscription(
            AckermannDriveStamped,
            "/ackermann_cmd",
            self.ackermann_callback,
            10
        )
        self.twist_pub = self.create_publisher(
            Twist,
            "/ackermann_cmd_as_twist",
            10
        )

        # Twist → Ackermann
        self.twist_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.twist_callback,
            10
        )
        self.ackermann_pub = self.create_publisher(
            AckermannDriveStamped,
            "/cmd_vel_as_ackermann",
            10
        )

    def ackermann_callback(self, msg):
        twist = Twist()
        twist.linear.x = msg.drive.speed
        if abs(msg.drive.steering_angle) > 1e-4:
            twist.angular.z = msg.drive.speed * math.tan(msg.drive.steering_angle) / self.wheelbase
        else:
            twist.angular.z = 0.0
        self.twist_pub.publish(twist)

    def twist_callback(self, msg):
        ackermann = AckermannDriveStamped()
        ackermann.drive.speed = msg.linear.x
        if abs(msg.linear.x) > 1e-4:
            ackermann.drive.steering_angle = math.atan(msg.angular.z * self.wheelbase / msg.linear.x)
        else:
            ackermann.drive.steering_angle = 0.0
        self.ackermann_pub.publish(ackermann)

def main(args=None):
    rclpy.init(args=args)
    node = AckermannRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
