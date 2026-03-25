#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from math import sin, cos, pi
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from rclpy.duration import Duration

class DisparityExtender(Node):
    def __init__(self):
        super().__init__('disparity_extender')
        self.laser_sub = self.create_subscription(
            LaserScan, '/hokuyo/scan', self.lidar_callback, 10  # Corrected topic name
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/ackermann_cmd', 10)  # Corrected topic name
        self.marker_pub = self.create_publisher(Marker, '/viz_marker', 10)
        self.marker_scan = self.create_publisher(LaserScan, '/viz_scan', 10)

        # Parameters
        self.car_half_width = 0.1
        self.disparity_threshold = 0.5
        self.safety_margin = 2.0
        self.min_speed = 1.0
        self.max_speed = 1.5
        self.safety_dist = 0.8
        self.dist_for_max_speed = 1.0
        self.smoothing_factor = 0.2
        self.deadband = 0.5
        self.max_steering_change = 1.0
        self.prev_steering_angle = 0.0
        self.desired_min_angle = -pi/2  # -90°
        self.desired_max_angle = pi/2    # +90°

        self.get_logger().info("Optimized Disparity Extender node started.")

    def detect_disparities(self, lidar_data, threshold=0.5):
        """Detect disparities using a more robust method with rolling window."""
        diffs = np.abs(np.diff(lidar_data))
        window_size = 4
        smoothed_diffs = np.convolve(diffs, np.ones(window_size)/window_size, mode='same')
        return np.where(smoothed_diffs > threshold)[0] + 1

    def process_disparity(self, lidar_data, index, angle_inc):
        """Process disparity by extending the obstacle based on car width and distance."""
        if index >= len(lidar_data):
            return lidar_data

        d_left = lidar_data[index - 1] if index > 0 else lidar_data[index]
        d_right = lidar_data[index]
        closer = min(d_left, d_right)

        # calculate samples to overwrite based on car width and distance (this is simulation based) #TODO adjust it while working on hardware
        if closer > 0.3:
            angle_span = np.arctan2(self.car_half_width, closer)
            num_samples = int(angle_span / angle_inc) + 1
        else:
            num_samples = int(self.safety_margin)

        # overwrite samples in both directions
        for j in range(-num_samples, num_samples + 1):
            idx = index + j
            if 0 <= idx < len(lidar_data) and lidar_data[idx] > closer:
                lidar_data[idx] = closer
        return lidar_data

    def compute_path_integral(self, processed_scan, angle_min, angle_inc, front_indices):
        """Compute path integral with balanced distance and angle weighting."""
        path_integral = np.zeros(len(processed_scan))
        for i in front_indices:
            distance = processed_scan[i] ** 1
            angle = angle_min + i * angle_inc
            # weight by distance and angle (favor paths that are far and straight)
            weight = (distance ** 2) * (2.0 - abs(angle) / (pi/2))  # wider FOV
            path_integral[i] = weight
        return path_integral

    def find_optimal_angle(self, path_integral, angle_min, angle_inc, front_indices):
        """Find the optimal steering angle."""
        max_value = -1
        max_index = None
        for i in front_indices:
            if path_integral[i] > max_value:
                max_value = path_integral[i]
                max_index = i
        if max_index is None:
            return 0.0
        return angle_min + max_index * angle_inc

    def smooth_steering(self, new_angle):
        """Smooth the steering angle with a low-pass filter."""
        angle_change = new_angle - self.prev_steering_angle
        if abs(angle_change) > self.deadband:
            smoothed_angle = self.smoothing_factor * self.prev_steering_angle + (1 - self.smoothing_factor) * new_angle
            limited_angle = self.prev_steering_angle + np.sign(angle_change) * min(abs(angle_change), self.max_steering_change)
            self.prev_steering_angle = limited_angle
            return limited_angle
        return self.prev_steering_angle

    def adjust_speed(self, front_dist):
        """Adjust speed based on the distance to the nearest obstacle."""
        if front_dist < self.safety_dist:
            return self.min_speed * 0.5
        if front_dist > self.dist_for_max_speed:
            return self.max_speed
        ratio = (front_dist - self.safety_dist) / (self.dist_for_max_speed - self.safety_dist)
        return self.min_speed + ratio * (self.max_speed - self.min_speed)

    def expand_obstacle_buffer(self, lidar_data, angle_min, angle_inc):
        """Expand the buffer around obstacles to ensure the car maintains a safe distance."""
        expanded_data = np.copy(lidar_data)
        for i in range(len(lidar_data)):
            distance = lidar_data[i] 
            angle = angle_min + i * angle_inc
            buffer_distance = max(0.5, distance - 2.5)  # Ebuffer by 2.5 meters (idk adjust it depending on the situation)
            expanded_data[i] = buffer_distance
        return expanded_data

    def lidar_callback(self, scan_msg):
        #LIDAR data to numpy array
        ranges = np.array(scan_msg.ranges)
        angle_min = scan_msg.angle_min
        angle_max = scan_msg.angle_max
        angle_inc = scan_msg.angle_increment

        #filter out-of-range values
        valid_mask = (ranges >= scan_msg.range_min) & (ranges <= scan_msg.range_max)
        ranges[~valid_mask] = scan_msg.range_max

        # smooth the LIDAR data
        smoothed_data = np.convolve(ranges, np.ones(5)/2, mode='same')

        # detect and process disparities (i hope this is working tbh)
        disparity_indices = self.detect_disparities(smoothed_data, threshold=self.disparity_threshold)
        for idx in disparity_indices:
            smoothed_data = self.process_disparity(smoothed_data, idx, angle_inc)

        # expand obstacle buffer
        expanded_data = self.expand_obstacle_buffer(smoothed_data, angle_min, angle_inc)

        # plus minus 90 for better side detection
        angles = angle_min + np.arange(len(expanded_data)) * angle_inc
        front_mask = (angles >= self.desired_min_angle) & (angles <= self.desired_max_angle)
        front_indices = np.where(front_mask)[0]

        # compute path integral and find optimal angle
        path_integral = self.compute_path_integral(expanded_data, angle_min, angle_inc, front_indices)
        computed_angle = self.find_optimal_angle(path_integral, angle_min, angle_inc, front_indices)

        # smooth the steering angle
        steering_angle = self.smooth_steering(computed_angle)

        # adjust speed based on front distance
        front_dist = expanded_data[front_indices].max()
        speed = self.adjust_speed(front_dist)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

        p0 = Point(x=0.0, y=0.0, z=0.0)
        p1 = Point(x=front_dist * cos(steering_angle), y=front_dist * sin(steering_angle), z=0.0)
        marker = Marker()
        marker.header.frame_id = scan_msg.header.frame_id
        marker.header.stamp = scan_msg.header.stamp
        marker.action = Marker.ADD
        marker.type = Marker.ARROW
        marker.id = 1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.4
        marker.scale.z = 0.6
        marker.color = ColorRGBA(a=1.0, r=0.0, g=1.0, b=0.0)
        marker.points = [p0, p1]
        marker.lifetime = Duration(seconds=1).to_msg()
        self.marker_pub.publish(marker)

        scan_msg.ranges = expanded_data.tolist()
        self.marker_scan.publish(scan_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DisparityExtender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt detected, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()