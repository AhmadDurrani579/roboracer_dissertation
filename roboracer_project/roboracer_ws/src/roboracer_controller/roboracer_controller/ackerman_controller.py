#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from transforms3d.euler import euler2quat
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from tf2_ros import TransformBroadcaster
import math
import numpy as np
from rosgraph_msgs.msg import Clock

class AckermannController(Node):
    def __init__(self):
        super().__init__("ackermann_controller")
        self.use_sim_time = self.get_parameter_or(
            "use_sim_time", rclpy.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        ).value
        self.get_logger().info(f"Using simulation time: {self.use_sim_time}")

        # Parameters
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("wheelbase", 0.325)
        self.declare_parameter("track_width", 0.18)
        self.declare_parameter("max_steering_angle", 0.5)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("odom_topic", "/car_1/odom")
        self.declare_parameter("cmd_topic", "/ackermann_cmd")
        self.declare_parameter("velocity_threshold", 0.001)
        self.declare_parameter("pose_cov_x", 0.01)
        self.declare_parameter("pose_cov_y", 0.01)
        self.declare_parameter("pose_cov_yaw", 0.01)
        self.declare_parameter("twist_cov_linear_x", 0.01)
        self.declare_parameter("twist_cov_angular_z", 0.01)
        self.declare_parameter("tf_publish_rate", 50.0)  # New parameter for TF rate

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
        self.tf_publish_rate_ = self.get_parameter("tf_publish_rate").value
        if self.tf_publish_rate_ <= 0.0:
            self.get_logger().warn("Invalid tf_publish_rate, will publish TF on every joint state update.")
            self.tf_publish_period_ = None
        else:
            self.tf_publish_period_ = 1.0 / self.tf_publish_rate_
            self.last_tf_publish_time_ = self.get_clock().now()

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

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        self.vel_sub_ = self.create_subscription(
            AckermannDriveStamped, self.cmd_topic_, self.vel_callback, qos
        )
        self.joint_sub_ = self.create_subscription(
            JointState, self.joint_state_topic_, self.joint_callback, qos
        )

        self.odom_pub_ = self.create_publisher(Odometry, self.odom_topic_, 10)
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "car_1_base_link"
        self.br_ = TransformBroadcaster(self, qos=rclpy.qos.QoSProfile(depth=10))
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "car_1_base_link"

        self.odom_msg_.pose.covariance = np.diag([
            self.pose_cov_x_, self.pose_cov_y_, 1e-6, 1e-6, 1e-6, self.pose_cov_yaw_
        ]).flatten().tolist()
        self.odom_msg_.twist.covariance = np.diag([
            self.twist_cov_linear_x_, 1e-6, 1e-6, 1e-6, 1e-6, self.twist_cov_angular_z_
        ]).flatten().tolist()

        self.get_logger().info("AckermannController initialized")

    def vel_callback(self, msg):
        self.linear_vel_ = msg.drive.speed
        steering_angle = max(min(msg.drive.steering_angle, self.max_steering_angle_), -self.max_steering_angle_)
        self.angular_vel_ = self.linear_vel_ * math.tan(steering_angle) / self.wheelbase_

    def _get_joint_indices(self, msg):
        required_joints = [
            "car_1_left_rear_wheel_joint",
            "car_1_right_rear_wheel_joint",
            "car_1_left_steering_hinge_joint",
            "car_1_right_steering_hinge_joint"
        ]
        indices = {}
        missing_joints = []
        for joint in required_joints:
            try:
                indices[joint] = msg.name.index(joint)
            except ValueError:
                missing_joints.append(joint)
        if missing_joints:
            self.get_logger().error(f"Missing joints in /joint_states: {missing_joints}")
            return None
        return indices

    def _calculate_wheel_velocities(self, msg, dt, joint_indices):
        left_rear_vel = 0.0
        right_rear_vel = 0.0
        if (len(msg.velocity) > max(joint_indices["car_1_left_rear_wheel_joint"], joint_indices["car_1_right_rear_wheel_joint"]) and
            msg.velocity[joint_indices["car_1_left_rear_wheel_joint"]] is not None and
            msg.velocity[joint_indices["car_1_right_rear_wheel_joint"]] is not None):
            left_rear_vel = msg.velocity[joint_indices["car_1_left_rear_wheel_joint"]]
            right_rear_vel = msg.velocity[joint_indices["car_1_right_rear_wheel_joint"]]
        else:
            left_rear_pos = msg.position[joint_indices["car_1_left_rear_wheel_joint"]]
            right_rear_pos = msg.position[joint_indices["car_1_right_rear_wheel_joint"]]
            left_rear_vel = (left_rear_pos - self.prev_left_rear_pos_) / dt if dt > 0 else 0.0
            right_rear_vel = (right_rear_pos - self.prev_right_rear_pos_) / dt if dt > 0 else 0.0
            self.prev_left_rear_pos_ = left_rear_pos
            self.prev_right_rear_pos_ = right_rear_pos
        return left_rear_vel, right_rear_vel

    def _update_odometry(self, linear_vel, angular_vel, dt, current_time):
        self.theta_ += angular_vel * dt
        self.x_ += linear_vel * math.cos(self.theta_) * dt
        self.y_ += linear_vel * math.sin(self.theta_) * dt

        q = euler2quat(0, 0, self.theta_)
        self.odom_msg_.header.stamp = current_time.to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.orientation.x = q[1]
        self.odom_msg_.pose.pose.orientation.y = q[2]
        self.odom_msg_.pose.pose.orientation.z = q[3]
        self.odom_msg_.pose.pose.orientation.w = q[0]
        self.odom_msg_.twist.twist.linear.x = linear_vel
        self.odom_msg_.twist.twist.angular.z = angular_vel
        self.odom_pub_.publish(self.odom_msg_)

    def _publish_transform(self, current_time):
        self.transform_stamped_.header.stamp = current_time.to_msg()
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_

        yaw = self.theta_
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        self.transform_stamped_.transform.rotation.x = qx
        self.transform_stamped_.transform.rotation.y = qy
        self.transform_stamped_.transform.rotation.z = qz
        self.transform_stamped_.transform.rotation.w = qw
        self.br_.sendTransform(self.transform_stamped_)
        
        
    def joint_callback(self, msg):
        joint_indices = self._get_joint_indices(msg)
        if joint_indices is None:
            return

        current_time = self.get_clock().now()
        if self.prev_time_ is None:
            self.prev_time_ = current_time
            return

        dt = (current_time - self.prev_time_).nanoseconds / 1e9
        if dt <= 0:
            self.get_logger().warn(f"Invalid dt={dt}")
            return

        left_rear_pos = msg.position[joint_indices["car_1_left_rear_wheel_joint"]]
        right_rear_pos = msg.position[joint_indices["car_1_right_rear_wheel_joint"]]
        self.left_steer_pos_ = msg.position[joint_indices["car_1_left_steering_hinge_joint"]]
        self.right_steer_pos_ = msg.position[joint_indices["car_1_right_steering_hinge_joint"]]

        left_rear_vel, right_rear_vel = self._calculate_wheel_velocities(msg, dt, joint_indices)
        linear_vel_from_wheels = self.wheel_radius_ * (left_rear_vel + right_rear_vel) / 2
        avg_steer_angle = (self.left_steer_pos_ + self.right_steer_pos_) / 2
        avg_steer_angle = max(min(avg_steer_angle, self.max_steering_angle_), -self.max_steering_angle_)
        angular_vel_from_wheels = linear_vel_from_wheels * math.tan(avg_steer_angle) / self.wheelbase_

        linear_vel = self.linear_vel_
        angular_vel = self.angular_vel_

        if abs(linear_vel_from_wheels) > self.velocity_threshold_ or abs(angular_vel_from_wheels) > self.velocity_threshold_:
            linear_vel = linear_vel_from_wheels
            angular_vel = angular_vel_from_wheels
            # self.get_logger().info("Using wheel velocities for odometry.")
        # else:
        #     self.get_logger().info("Using commanded velocities for odometry.")

        self._update_odometry(linear_vel, angular_vel, dt, current_time)

        if self.tf_publish_period_ is None or (current_time - self.last_tf_publish_time_).nanoseconds / 1e9 >= self.tf_publish_period_:
            self._publish_transform(current_time)
            self.last_tf_publish_time_ = current_time

        self.prev_time_ = current_time
        self.left_rear_pos_ = left_rear_pos
        self.right_rear_pos_ = right_rear_pos

def main(args=None):
    rclpy.init(args=args)
    node = AckermannController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()