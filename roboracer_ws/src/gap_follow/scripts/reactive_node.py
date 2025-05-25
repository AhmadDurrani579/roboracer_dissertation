#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class FollowTheGapNode(Node):
    def __init__(self):
        super().__init__('follow_the_gap')

        # parameters
        self.declare_parameter('safety_radius', 3.0)
        self.declare_parameter('max_throttle', 1.7)
        self.declare_parameter('min_throttle', 0.5)
        self.declare_parameter('max_steering_angle', 0.69)
        self.declare_parameter('gap_alpha', 2.0)
        self.declare_parameter('base_bubble', 1.75)
        self.declare_parameter('bubble_gain', 0.55)
        self.declare_parameter('max_bubble_radius', 3.5)
        # PID steering control
        self.declare_parameter('steering_kp', 1.0)
        self.declare_parameter('steering_ki', 0.05)
        self.declare_parameter('steering_kd', 0.1)
        self.declare_parameter('steering_deadzone', 0.02)  # normalized output
        self.declare_parameter('integral_limit', 0.5)
        self.declare_parameter('derivative_filter_alpha', 0.7)
        self.declare_parameter('angle_smooth_alpha', 0.3)
        self.declare_parameter('max_angle_change', 0.4)
        self.angle_smooth_alpha = self.get_parameter('angle_smooth_alpha').value
        self.max_angle_change = self.get_parameter('max_angle_change').value
        self.last_raw_angle = 0.0

        p = self.get_parameter
        self.safety_radius      = p('safety_radius').value
        self.max_throttle       = p('max_throttle').value
        self.min_throttle       = p('min_throttle').value
        self.max_steering_angle = p('max_steering_angle').value
        self.alpha              = p('gap_alpha').value
        self.base_bubble        = p('base_bubble').value
        self.bubble_gain        = p('bubble_gain').value
        self.max_bubble_radius  = p('max_bubble_radius').value
        self.kp                 = p('steering_kp').value
        self.ki                 = p('steering_ki').value
        self.kd                 = p('steering_kd').value
        self.deadzone           = p('steering_deadzone').value
        self.i_limit            = p('integral_limit').value
        self.d_filter_alpha     = p('derivative_filter_alpha').value

        # internal state for PID
        self.error_integral = 0.0
        self.last_error = 0.0
        self.filtered_derivative = 0.0
        now = self.get_clock().now().nanoseconds
        self.last_time = now / 1e9
        self.last_speed = self.max_throttle

        # subscriptions & publications
        self.scan_sub = self.create_subscription(
            LaserScan, '/hokuyo/scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/ackermann_cmd', 10)

        self.get_logger().info('FollowTheGap node with PID steering started')

    def _apply_bubble(self, ranges, angle_inc):

        idx = int(np.argmin(ranges))
        nearest = ranges[idx]

        R = min(self.base_bubble + self.bubble_gain * nearest,
                self.max_bubble_radius)
        
        angles = np.arange(ranges.size) * angle_inc
        d = np.sqrt(ranges**2 + nearest**2
                    - 2 * ranges * nearest * np.cos(angles - angles[idx]))
        proc = ranges.copy()
        proc[d < R] = 0.0
        return proc

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = msg.range_max

        proc = self._apply_bubble(ranges, msg.angle_increment)
        raw = self._find_best_gap(proc, msg.angle_min, msg.angle_increment)

        raw = (self.angle_smooth_alpha * raw +
            (1 - self.angle_smooth_alpha) * getattr(self, 'last_raw_angle', 0.0))

        delta = raw - getattr(self, 'last_raw_angle', raw)
        raw = getattr(self, 'last_raw_angle', raw) + max(
            -self.max_angle_change, min(self.max_angle_change, delta)
        )
        self.last_raw_angle = raw

        self.get_logger().info(f'Best raw angle: {math.degrees(raw):.1f}°')
        self._publish_ackermann(raw)

    def _find_best_gap(self, ranges, angle_min, angle_inc):
            mask = ranges > self.safety_radius
            if not np.any(mask):
                return 0.0

            max_len = start = end = cur = 0
            for i, ok in enumerate(mask):
                if ok:
                    cur += 1
                else:
                    if cur > max_len:
                        max_len, start, end = cur, i - cur, i
                    cur = 0
            if cur > max_len:
                max_len, start, end = cur, len(mask) - cur, len(mask)

            mid = (start + end) // 2
            return angle_min + mid * angle_inc



    def _publish_ackermann(self, error):
        # PID control
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time if self.last_time else 0.02
        # integral with anti-windup
        self.error_integral += error * dt
        self.error_integral = max(-self.i_limit, min(self.i_limit, self.error_integral))
        # derivative with filtering
        raw_deriv = (error - self.last_error) / dt if dt > 1e-6 else 0.0
        self.filtered_derivative = (
            self.d_filter_alpha * self.filtered_derivative +
            (1 - self.d_filter_alpha) * raw_deriv
        )

        # compute control (normalized)
        ctrl = self.kp * error + self.ki * self.error_integral + self.kd * self.filtered_derivative
        # deadzone
        if abs(ctrl) < self.deadzone:
            ctrl = 0.0
            self.filtered_derivative = 0.0

        # clamp normalized output to [-1,1]
        ctrl = max(-1.0, min(1.0, ctrl))

        # steering angle
        steer = ctrl * self.max_steering_angle

        # adaptive throttle: reduce on high curvature
        penalty = min(abs(ctrl), 1.0)
        throttle = self.max_throttle * (1.0 - 0.7 * penalty)
        throttle = max(throttle, self.min_throttle)
        self.last_speed = throttle

        # publish
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = float(steer)
        msg.drive.speed = float(throttle)
        self.drive_pub.publish(msg)

        self.get_logger().info(
            f'Speed: {throttle:.2f}, Steering: {math.degrees(steer):.1f}°'
        )

        # update state
        self.last_error = error
        self.last_time = now


def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()