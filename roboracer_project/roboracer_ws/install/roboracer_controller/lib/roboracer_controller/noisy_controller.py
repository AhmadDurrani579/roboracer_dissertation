#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import numpy as np
from rclpy.time import Time
from rclpy.constants import S_TO_NS
import math
from transforms3d.euler import euler2quat
from rclpy.qos import QoSProfile
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class NoisyController(Node):
    
    def __init__(self):
        super().__init__("ackermann_controller")
        # Ensure use_sim_time is set (reinforce for consistency)
        self.use_sim_time = self.get_parameter_or(
            "use_sim_time", rclpy.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        ).value
        self.get_logger().info(f"Using simulation time: {self.use_sim_time}")

        # Parameters
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("wheelbase", 0.325)
        self.declare_parameter("track_width", 0.18)
        self.declare_parameter("max_steering_angle", 0.5)
        self.declare_parameter("joint_state_topic", "/model/car_1/joint_state")
        self.declare_parameter("odom_topic", "/car_1/odom")
        self.declare_parameter("cmd_topic", "/ackermann_cmd")
        self.declare_parameter("velocity_threshold", 0.001)
        self.declare_parameter("pose_cov_x", 0.01)
        self.declare_parameter("pose_cov_y", 0.01)
        self.declare_parameter("pose_cov_yaw", 0.01)
        self.declare_parameter("twist_cov_linear_x", 0.01)
        self.declare_parameter("twist_cov_angular_z", 0.01)

        self.wheel_radius_ = self.get_parameter("wheel_radius").value
        self.wheelbase_ = self.get_parameter("wheelbase").value
        self.track_width_ = self.get_parameter("track_width").value
        self.max_steering_angle_ = self.get_parameter("max_steering_angle").value
        self.joint_state_topic_ = self.get_parameter("joint_state_topic").value
        self.odom_topic_ = self.get_parameter("odom_topic").value
        self.cmd_topic_ = self.get_parameter("cmd_topic").value
        self.velocity_threshold_ = self.get_parameter("velocity_threshold").value
        self.pose_cov_x_ = self.get_parameter("pose_cov_x").value
        self.pose_cov_y_ = self.get_parameter("pose_cov_y").value
        self.pose_cov_yaw_ = self.get_parameter("pose_cov_yaw").value
        self.twist_cov_linear_x_ = self.get_parameter("twist_cov_linear_x").value
        self.twist_cov_angular_z_ = self.get_parameter("twist_cov_angular_z").value

        self.get_logger().info(f"Wheel radius: {self.wheel_radius_} m")
        self.get_logger().info(f"Joint state topic: {self.joint_state_topic_}")
        self.get_logger().info(f"Covariances: pose_x={self.pose_cov_x_}, pose_y={self.pose_cov_y_}, "
                              f"pose_yaw={self.pose_cov_yaw_}, twist_x={self.twist_cov_linear_x_}, "
                              f"twist_z={self.twist_cov_angular_z_}")

        # State
        self.left_rear_pos_ = 0.0
        self.right_rear_pos_ = 0.0
        self.left_steer_pos_ = 0.0
        self.right_steer_pos_ = 0.0
        self.prev_left_rear_pos_ = 0.0
        self.prev_right_rear_pos_ = 0.0
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        self.linear_vel_ = 0.0
        self.angular_vel_ = 0.0
        self.prev_time_ = None

        # QoS
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # Subscribers
        self.vel_sub_ = self.create_subscription(
            AckermannDriveStamped, self.cmd_topic_, self.vel_callback, qos
        )
        self.joint_sub_ = self.create_subscription(
            JointState, self.joint_state_topic_, self.joint_callback, qos
        )

        # Publishers
        self.odom_pub_ = self.create_publisher(Odometry, self.odom_topic_, 10)
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "car_1_base_link"
        self.br_ = TransformBroadcaster(self, qos=rclpy.qos.QoSProfile(depth=10))
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "car_1_base_link"

        # Covariance
        self.odom_msg_.pose.covariance = np.diag([
            self.pose_cov_x_, self.pose_cov_y_, 1e-6, 1e-6, 1e-6, self.pose_cov_yaw_
        ]).flatten().tolist()
        self.odom_msg_.twist.covariance = np.diag([
            self.twist_cov_linear_x_, 1e-6, 1e-6, 1e-6, 1e-6, self.twist_cov_angular_z_
        ]).flatten().tolist()

        # Publish an initial transform
        initial_time = self.get_clock().now()
        self.transform_stamped_.header.stamp = initial_time.to_msg()
        self.transform_stamped_.transform.translation.x = 0.0
        self.transform_stamped_.transform.translation.y = 0.0
        self.transform_stamped_.transform.translation.z = 0.0
        self.transform_stamped_.transform.rotation.x = 0.0
        self.transform_stamped_.transform.rotation.y = 0.0
        self.transform_stamped_.transform.rotation.z = 0.0
        self.transform_stamped_.transform.rotation.w = 1.0
        self.br_.sendTransform(self.transform_stamped_)
        self.get_logger().info("Published initial odom to car_1_base_link transform")

        self.get_logger().info("AckermannController initialized")
        
    def joint_callback(self, msg):
        try:
            left_rear_idx = msg.name.index("car_1_left_rear_wheel_joint")
            right_rear_idx = msg.name.index("car_1_right_rear_wheel_joint")
            left_steer_idx = msg.name.index("car_1_left_steering_hinge_joint")
            right_steer_idx = msg.name.index("car_1_right_steering_hinge_joint")
        except ValueError as e:
            self.get_logger().error(f"Joint not found: {e}")
            return

        # Calculate time difference
        current_time = Time.from_msg(msg.header.stamp)
        if self.prev_time_ is None:
            self.prev_time_ = current_time
            return

        dt = (current_time - self.prev_time_).nanoseconds / S_TO_NS
        if dt <= 0:
            self.get_logger().warn(f"Non-positive time difference (dt={dt}), skipping odometry update. "
                                  f"Current: {current_time.nanoseconds}, Prev: {self.prev_time_.nanoseconds}")
            self.prev_time_ = current_time
            return

        self.get_logger().debug(f"dt={dt}, stamp={current_time.nanoseconds}")

        # Add noise to joint readings
        noise_scale = 0.005  # Adjust noise level as needed
        self.left_rear_pos_ = msg.position[left_rear_idx] + np.random.normal(0, noise_scale)
        self.right_rear_pos_ = msg.position[right_rear_idx] + np.random.normal(0, noise_scale)
        self.left_steer_pos_ = msg.position[left_steer_idx] + np.random.normal(0, noise_scale)
        self.right_steer_pos_ = msg.position[right_steer_idx] + np.random.normal(0, noise_scale)

        # Calculate position increments
        dp_left = self.left_rear_pos_ - self.prev_left_rear_pos_
        dp_right = self.right_rear_pos_ - self.prev_right_rear_pos_

        # Update previous positions
        self.prev_left_rear_pos_ = self.left_rear_pos_
        self.prev_right_rear_pos_ = self.right_rear_pos_

        # Calculate rear wheel velocities
        left_rear_vel = dp_left / dt
        right_rear_vel = dp_right / dt
        linear_vel = self.wheel_radius_ * (left_rear_vel + right_rear_vel) / 2

        # Calculate steering angle and angular velocity
        avg_steer_angle = (self.left_steer_pos_ + self.right_steer_pos_) / 2
        angular_vel = linear_vel * math.tan(avg_steer_angle) / self.wheelbase_

        # Update odometry (Ackermann kinematics)
        d_theta = angular_vel * dt
        d_s = linear_vel * dt
        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)

        # Prepare odometry message
        q = euler2quat(0, 0, self.theta_)
        self.odom_msg_.header.stamp = current_time.to_msg()  # Use joint state stamp for consistency
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.position.z = 0.0
        self.odom_msg_.pose.pose.orientation.x = q[1]
        self.odom_msg_.pose.pose.orientation.y = q[2]
        self.odom_msg_.pose.pose.orientation.z = q[3]
        self.odom_msg_.pose.pose.orientation.w = q[0]
        self.odom_msg_.twist.twist.linear.x = linear_vel
        self.odom_msg_.twist.twist.angular.z = angular_vel

        # Prepare and send transform
        self.transform_stamped_.header.stamp = current_time.to_msg()
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.translation.z = 0.0
        self.transform_stamped_.transform.rotation.x = q[1]
        self.transform_stamped_.transform.rotation.y = q[2]
        self.transform_stamped_.transform.rotation.z = q[3]
        self.transform_stamped_.transform.rotation.w = q[0]

        # Publish
        self.odom_pub_.publish(self.odom_msg_)
        self.br_.sendTransform(self.transform_stamped_)
        self.prev_time_ = current_time

def main(args=None):
    rclpy.init(args=args)
    node = NoisyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
