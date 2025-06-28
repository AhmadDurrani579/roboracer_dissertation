import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import math
import numpy as np
from typing import List, Tuple, Optional

class GapFollow(Node):

    def __init__(self):
        super().__init__('gap_follow')

        # Parameters
        self.bubble_radius = 160
        self.preprocess_conv_size = 3
        self.best_point_conv_size = 80
        #self.max_lidar_dist = 30.0
        self.straights_speed = 1.0
        self.corners_speed = 1.0
        self.straights_steering_angle = np.pi / 18
        self.max_steer = 0.4
        self.last_steering=0.0

        # Ackermann geometry
        self.wheelbase = 0.35 # meters
        #self.track_width = 0.20 # meters

        # ROS parameters
        self.declare_parameter('use_curvature', True)
        self.declare_parameter('curvature_weight', 0.5)
        self.use_curvature = self.get_parameter('use_curvature').get_parameter_value().bool_value
        self.curvature_weight = self.get_parameter('curvature_weight').get_parameter_value().double_value

        # ROS Interfaces
        self.laser_sub = self.create_subscription(
            LaserScan, '/hokuyo/scan', self.laser_callback, 10
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/ackermann_cmd', 10
        )

    def preprocess_lidar(self, scan: LaserScan):
        ranges = np.array(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment

        mask = np.logical_and(angles >= -3*np.pi/4, angles <= 3*np.pi/4)
        proc = ranges[mask]
        proc_angles = angles[mask]

        proc = np.nan_to_num(proc,
                            nan=scan.range_max,
                            posinf=scan.range_max,
                            neginf=scan.range_min)

        proc = np.clip(proc, scan.range_min, scan.range_max)

        proc = np.convolve(proc,
                        np.ones(self.preprocess_conv_size),
                        mode='same') / self.preprocess_conv_size

        return proc, proc_angles

    def laser_callback(self, scan_msg: LaserScan):
        proc_ranges, proc_angles = self.preprocess_lidar(scan_msg)
        if proc_ranges.size == 0:
            self.get_logger().warn("Empty front scan")
            self.publish_drive(0.0, 0.0)
            return

        closest = int(np.argmin(proc_ranges))
        min_i = max(0, closest - self.bubble_radius)
        max_i = min(len(proc_ranges) - 1, closest + self.bubble_radius)
        proc_ranges[min_i:max_i+1] = 0

        start, end = self.find_safe_gap(proc_ranges, self.last_steering)
        best_idx = self.find_best_point(start, end, proc_ranges)

        steering = float(np.clip(proc_angles[best_idx], -self.max_steer, self.max_steer))
        speed = self.calculate_speed(steering)
        self.publish_drive(speed, steering)
        self.last_steering = steering

    def find_safe_gap(self, free_space_ranges, current_steering_angle=0.0):
        if len(free_space_ranges) == 0:
            return 0, len(free_space_ranges) - 1

        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        slices = np.ma.notmasked_contiguous(masked)

        if not slices:
            return 0, len(free_space_ranges) - 1

        best_gap = None
        best_score = -float('inf')

        for sl in slices:
            start = max(0, sl.start)
            end = min(len(free_space_ranges) - 1, sl.stop)
            gap_width = end - start
            gap_center = (start + end) / 2
            span = 3*np.pi/2
            center_angle = (gap_center / len(free_space_ranges)) * span - (span/2)


            if self.use_curvature:
                curvature = abs(center_angle - current_steering_angle)
                score = gap_width - self.curvature_weight * curvature
            else:
                score = gap_width

            if score > best_score:
                best_score = score
                best_gap = (start, end)

        if best_gap:
            self.get_logger().info(f"Selected gap: start={best_gap[0]}, end={best_gap[1]}")
            return best_gap
        else:
            return 0, len(free_space_ranges) - 1

    def find_best_point(self, start_i, end_i, ranges):
        averaged_max_gap = np.convolve(
            ranges[start_i:end_i], np.ones(self.best_point_conv_size), 'same'
        ) / self.best_point_conv_size
        return averaged_max_gap.argmax() + start_i

    def get_ackermann_steering_angle(self, lidar_angle):
        if abs(lidar_angle) < 1e-3:
            return 0.0  

        turn_radius = self.wheelbase / math.tan(lidar_angle)
        steering_angle = math.atan(self.wheelbase / turn_radius)
        return float(np.clip(steering_angle, -self.max_steer, self.max_steer))


    def calculate_speed(self, steering_angle):
        abs_angle = abs(steering_angle)
        if abs_angle > self.straights_steering_angle:
            speed = max(0.5, self.corners_speed * (1 - abs_angle / self.max_steer))
        else:
            speed = self.straights_speed
        return speed

    def publish_drive(self, speed, steering):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "car_1_base_link"

        sub_drive_msg = AckermannDrive()
        sub_drive_msg.steering_angle = float(steering)
        sub_drive_msg.steering_angle_velocity = 0.0
        sub_drive_msg.speed = float(speed)
        sub_drive_msg.acceleration = 0.0
        sub_drive_msg.jerk = 0.0

        drive_msg.drive = sub_drive_msg
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GapFollow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()