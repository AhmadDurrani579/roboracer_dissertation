#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class StaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_publisher')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'odom')
        self.declare_parameter('translation_x', 0.0)
        self.declare_parameter('translation_y', 0.0)
        self.declare_parameter('translation_z', 0.0)
        self.declare_parameter('rotation_x', 0.0)
        self.declare_parameter('rotation_y', 0.0)
        self.declare_parameter('rotation_z', 0.0)
        self.declare_parameter('rotation_w', 1.0)

        frame_id = self.get_parameter('frame_id').value
        child_frame_id = self.get_parameter('child_frame_id').value
        translation_x = self.get_parameter('translation_x').value
        translation_y = self.get_parameter('translation_y').value
        translation_z = self.get_parameter('translation_z').value
        rotation_x = self.get_parameter('rotation_x').value
        rotation_y = self.get_parameter('rotation_y').value
        rotation_z = self.get_parameter('rotation_z').value
        rotation_w = self.get_parameter('rotation_w').value

        tf_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        self.br_ = TransformBroadcaster(self, qos=tf_qos)
        self.transform = TransformStamped()
        self.transform.header.frame_id = frame_id
        self.transform.child_frame_id = child_frame_id
        self.transform.transform.translation.x = float(translation_x)
        self.transform.transform.translation.y = float(translation_y)
        self.transform.transform.translation.z = float(translation_z)
        self.transform.transform.rotation.x = float(rotation_x)
        self.transform.transform.rotation.y = float(rotation_y)
        self.transform.transform.rotation.z = float(rotation_z)
        self.transform.transform.rotation.w = float(rotation_w)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(f"Publishing static transform from {frame_id} to {child_frame_id}")

    def timer_callback(self):
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.br_.sendTransform(self.transform)

def main(args=None):
    rclpy.init(args=args)
    node = StaticTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()