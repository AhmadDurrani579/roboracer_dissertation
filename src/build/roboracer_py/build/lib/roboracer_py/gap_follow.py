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
        self.bubble_radius = 160  # Radius in terms of lidar indices (after cropping)
        self.preprocess_conv_size = 3
        self.best_point_conv_size = 80
        self.max_lidar_dist = 3000000
        self.fast_speed = 5
        self.straights_speed = 1.0
        self.corners_speed = 1.0
        self.straights_steering_angle = np.pi / 18
        self.fast_steering_angle = 0.0785
        self.safe_threshold = 5
        self.max_steer = 0.4
        self.safety_bubble = 0.1
        self.min_obstacle_dist = 0.2
        self.radians_per_elem = None
        self.declare_parameter('use_curvature', True) #ROS parameter
        self.declare_parameter('curvature_weight', 0.5)
        
        self.use_curvature = self.get_parameter('use_curvature').get_parameter_value().bool_value
        self.curvature_weight = self.get_parameter('curvature_weight').get_parameter_value().double_value # Weighting between gap size and curvature

        # Subscribers/Publishers (unchanged)
        self.laser_sub = self.create_subscription(
            LaserScan, '/hokuyo/scan', self.laser_callback, 10
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/ackermann_cmd', 10
        )

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Crop to front 270 degrees and handle nan values. """
        self.radians_per_elem = (2 * np.pi) / len(ranges)
        # Crop to approximately the front 270 degrees (adjust indices if needed)
        proc_ranges = np.array(ranges[135:-135]) # Ensure float type for nan handling

        # Replace nan values with the maximum lidar distance

        proc_ranges = np.convolve(proc_ranges, np.ones(self.preprocess_conv_size), 'same') / self.preprocess_conv_size
        proc_ranges = np.clip(proc_ranges, 0, self.max_lidar_dist)
        return proc_ranges

    def laser_callback(self, scan_msg: LaserScan):
        try:
            ranges = np.array(scan_msg.ranges)

            # self.get_logger().debug(f"Raw ranges shape: {ranges.shape}, Content (first 20): {ranges[:20]}")
            proc_ranges = self.preprocess_lidar(ranges)

            # self.get_logger().debug(f"Shape of proc_ranges after preprocess: {proc_ranges.shape}, Content (first 20): {proc_ranges[:20]}")

            if proc_ranges.size == 0:
                self.get_logger().warn("Processed ranges array is empty!")
                import pdb; pdb.set_trace() # Breakpoint if proc_ranges is empty

            closest = proc_ranges.argmin()

            min_index = closest - self.bubble_radius
            max_index = closest + self.bubble_radius
            if min_index < 0: min_index = 0
            if max_index >= len(proc_ranges): max_index = len(proc_ranges) - 1
            proc_ranges[min_index:max_index] = 0

            best_gap_start, best_gap_end = self.find_safe_gap(proc_ranges)
            # self.get_logger().debug(f"best_gap_start: {best_gap_start}, best_gap_end: {best_gap_end}") # Log gap indices


            best = self.find_best_point(best_gap_start, best_gap_end, proc_ranges)

            steering_angle = self.get_angle(best, len(proc_ranges)) # Use length of cropped array
            # self.get_logger().debug(f"Calculated steering angle: {steering_angle:.4f}")
            self.get_logger().debug(f"Calculated steering angle (high precision): {steering_angle:.8f}")

            speed = 1.0
            self.publish_drive(speed, steering_angle)

        except Exception as e:
            self.get_logger().error(f"Error in laser_callback: {e}")
            self.publish_drive(0.0, 0.0)
            
            
            

    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indices of max-gap range, respectively
        Return index of best point in ranges
        Naive: Choose the furthest point within ranges and go there
        """
        # do a sliding window average over the data in the max gap, this will
        # help the car to avoid hitting corners
        averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(self.best_point_conv_size),
                                       'same') / self.best_point_conv_size
        return averaged_max_gap.argmax() + start_i



    def find_safe_gap(self, free_space_ranges, current_steering_angle=0.0): # added steering angle
        """Finds the safest gap, considering both width and curvature."""
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
            center_angle = (gap_center / len(free_space_ranges)) * np.pi - (np.pi / 2)  # Angle of gap center

            if self.use_curvature:
              # Estimate curvature (a simple approximation)
              curvature = abs(center_angle - current_steering_angle)  # Smaller = better alignment
              # Combine gap width and curvature into a score.
              score = gap_width - self.curvature_weight * curvature # higher is better
            else:
               score = gap_width

            if score > best_score:
                best_score = score
                best_gap = (start, end)

        if best_gap:
            self.get_logger().info(f"Selected gap: start={best_gap[0]}, end={best_gap[1]}")
            return best_gap
        else:
            self.get_logger().info("No safe gap found, returning full range")
            return 0, len(free_space_ranges) - 1
        
        

    def get_angle(self, range_index, range_len):
        """ Get the angle of a particular element in the LiDAR data and transform it into an appropriate steering angle
        """
        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem
        steering_angle = lidar_angle / 2
        return steering_angle


    def calculate_speed(self, steering_angle):
        """ More aggressive speed reduction in turns """
        abs_angle = abs(steering_angle)
        if abs_angle > self.straights_steering_angle:
            # Progressive speed reduction based on steering
            speed = max(0.5, self.corners_speed * (1 - abs_angle/self.max_steer))
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