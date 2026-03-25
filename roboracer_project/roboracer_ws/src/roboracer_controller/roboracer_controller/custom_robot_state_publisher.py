#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from urdf_parser_py.urdf import URDF
import math

class CustomRobotStatePublisher(Node):
    def __init__(self):
        super().__init__('custom_robot_state_publisher')
        self.declare_parameter('robot_description', '')
        self.declare_parameter('use_sim_time', True)

        robot_description = self.get_parameter('robot_description').value
        if not robot_description:
            self.get_logger().error("robot_description parameter must be set")
            raise ValueError("robot_description parameter is empty")

        # Parse the URDF
        try:
            self.robot = URDF.from_xml_string(robot_description)
        except Exception as e:
            self.get_logger().error(f"Failed to parse robot_description: {str(e)}")
            raise e

        # Initialize TransformBroadcaster with BEST_EFFORT QoS
        tf_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        self.br_ = TransformBroadcaster(self, qos=tf_qos)
        self.transforms = []

        # Extract fixed joints from URDF
        for joint in self.robot.joints:
            if joint.type == 'fixed':
                if joint.origin is not None:
                    transform = TransformStamped()
                    transform.header.frame_id = joint.parent
                    transform.child_frame_id = joint.child
                    transform.transform.translation.x = joint.origin.xyz[0]
                    transform.transform.translation.y = joint.origin.xyz[1]
                    transform.transform.translation.z = joint.origin.xyz[2]
                    if joint.origin.rpy:
                        # Convert RPY to quaternion
                        roll, pitch, yaw = joint.origin.rpy
                        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
                        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
                        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
                        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
                        transform.transform.rotation.x = qx
                        transform.transform.rotation.y = qy
                        transform.transform.rotation.z = qz
                        transform.transform.rotation.w = qw
                    else:
                        transform.transform.rotation.w = 1.0
                    self.transforms.append(transform)

        # Publish transforms periodically
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Custom Robot State Publisher started, publishing fixed transforms")

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        for transform in self.transforms:
            transform.header.stamp = now
            self.br_.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = CustomRobotStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()