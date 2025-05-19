#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import numpy as np
 
 
class DisparityExtender(Node):
    def __init__(self):
        super().__init__("disparity_extender")
        self.CAR_WIDTH = 0.31  # in meters, make it a ROS parameter if needed
        self.DIFFERENCE_THRESHOLD = 0.1  # in meters, make it a ROS parameter
        self.SPEED = 1.0  # m/s, make it a ROS parameter
        self.SAFETY_PERCENTAGE = 10.0  #  Make it a ROS parameter.  Paper uses percentages like 20%
        self.laser_sub = self.create_subscription(
            LaserScan, '/hokuyo/scan', self.laser_callback, 10
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/ackermann_cmd', 10
        )
        self.radians_per_point = 0.0 # Initialize here, calculated in callback
        self.previous_steering_angle = 0.0  # Add this line

    def preprocess_lidar(self, ranges):
        """
        Preprocessing of the LiDAR data.
        Possible Improvements: smoothing of outliers in the data and placing
        a cap on the maximum distance a point can be.
        """
        # remove quadrant of LiDAR directly behind us
        # self.radians_per_elem = (2 * np.pi) / len(ranges) # This is calculated *per message*
        # Crop to approximately the front 270 degrees (adjust indices if needed)
        proc_ranges = np.array(ranges[90:-90])  # Ensure float type for nan handling.  Good!

        # eighth = int(len(ranges)/8) # Unused variable
        return proc_ranges
    
     
    def get_differences(self, ranges):
        """
        Gets the absolute difference between adjacent elements in
        the LiDAR data and returns them in an array.
        Possible Improvements: replace for loop with numpy array arithmetic
        """
        # differences = [0.]  # set first element to 0.  Better to handle edge in subtraction.
        # for i in range(1, len(ranges)):
        #     differences.append(abs(ranges[i]-ranges[i-1]))
        # return differences

        # Numpy implementation (more efficient)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=0.0, neginf=0.0) # <--- Handle -inf
        differences = np.abs(np.diff(ranges)) # diff calculates differences, abs for absolute
        return np.insert(differences, 0, 0.0) # insert a 0 at the beginning to match size.

    
    def get_disparities(self, differences, threshold):
        """
        Gets the indexes of the LiDAR points that were greatly
        different to their adjacent point.
        Possible Improvements: replace for loop with numpy array arithmetic
        """
        # disparities = []
        # for index, difference in enumerate(differences):
        #     if difference > threshold:
        #         disparities.append(index)
        # return disparities
        
        # Numpy implementation
        disparities = np.where(differences > threshold)[0] # where returns indices
        return disparities

    def get_num_points_to_cover(self, dist, width):
        """
        Returns the number of LiDAR points that correspond to a width at
        a given distance.
        We calculate the angle that would span the width at this distance,
        then convert this angle to the number of LiDAR points that
        span this angle.
        Current math for angle:
            sin(angle/2) = (w/2)/d) = w/2d
            angle/2 = sininv(w/2d)
            angle = 2sininv(w/2d)
            where w is the width to cover, and d is the distance to the close
            point.
        Possible Improvements: use a different method to calculate the angle.  The current one is correct.
        """
        angle = 2 * np.arcsin(width / (2 * dist))
        num_points = int(np.ceil(angle / self.radians_per_point))
        return num_points

    def cover_points(self, num_points, start_idx, cover_right, ranges):
        """
        'covers' a number of LiDAR points with the distance of a closer
        LiDAR point, to avoid us crashing with the corner of the car.
        num_points: the number of points to cover
        start_idx: the LiDAR point we are using as our distance
        cover_right: True/False, decides whether we cover the points to
                     right or to the left of start_idx
        ranges: the LiDAR points
        Possible improvements: reduce this function to fewer lines.  It's already pretty compact.
        """
        new_dist = ranges[start_idx]
        if cover_right:
            # for i in range(num_points): # vectorized
            #     next_idx = start_idx+1+i
            #     if next_idx >= len(ranges): break
            #     if ranges[next_idx] > new_dist:
            #         ranges[next_idx] = new_dist
            
            indices_to_cover = np.arange(start_idx + 1, min(start_idx + 1 + num_points, len(ranges)))
            ranges[indices_to_cover] = np.minimum(ranges[indices_to_cover], new_dist)

        else:
            # for i in range(num_points):
            #     next_idx = start_idx-1-i
            #     if next_idx < 0: break
            #     if ranges[next_idx] > new_dist:
            #         ranges[next_idx] = new_dist
            indices_to_cover = np.arange(start_idx - 1, max(start_idx - 1 - num_points, -1), -1)
            ranges[indices_to_cover] = np.minimum(ranges[indices_to_cover], new_dist)
        return ranges

    def extend_disparities(self, disparities, ranges, car_width, extra_pct):
        """
        For each pair of points we have decided have a large difference
        between them, we choose which side to cover (the opposite to
        the closer point), call the cover function, and return the
        resultant covered array.
        Possible Improvements: reduce to fewer lines
        """
        width_to_cover = (car_width / 2) * (1 + extra_pct / 100)
        close_dist = 0.0  # Initialize close_dist to a default value
        for index in disparities:
            first_idx = index - 1
            if first_idx < 0 or first_idx + 1 >= len(ranges):
                continue  # Skip if index is out of bounds
            points = ranges[first_idx:first_idx + 2]
            close_idx = first_idx + np.argmin(points)
            far_idx = first_idx + np.argmax(points)
            close_dist = ranges[close_idx] # error was happening here
            num_points_to_cover = self.get_num_points_to_cover(close_dist, width_to_cover)
            cover_right = close_idx < far_idx
            ranges = self.cover_points(num_points_to_cover, close_idx, cover_right, ranges)
        return ranges
            
    def get_steering_angle(self, range_index, range_len):
        """
        Calculate the angle that corresponds to a given LiDAR point and
        process it into a steering angle.
        Possible improvements: smoothing of aggressive steering angles
        """
        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_point
        steering_angle = np.clip(lidar_angle, np.radians(-90), np.radians(90))
        
        # Apply a simple smoothing filter (e.g., exponential moving average)
        alpha = 0.1  # Smoothing factor (0 < alpha < 1).  Adjust as needed.  Lowered alpha further
        steering_angle = alpha * steering_angle + (1 - alpha) * self.previous_steering_angle
        steering_angle = np.clip(steering_angle, np.radians(-30), np.radians(30)) # added limit
        self.previous_steering_angle = steering_angle # update
        return steering_angle
    
    def laser_callback(self, scan_msg: LaserScan):
        ranges = scan_msg.ranges
        self.radians_per_point = (2 * np.pi) / len(ranges) # Correct calculation
        proc_ranges = self.preprocess_lidar(ranges)
        differences = self.get_differences(proc_ranges)
        disparities = self.get_disparities(differences, self.DIFFERENCE_THRESHOLD)
        proc_ranges = self.extend_disparities(disparities, proc_ranges,
                                             self.CAR_WIDTH, self.SAFETY_PERCENTAGE)
        steering_angle = self.get_steering_angle(proc_ranges.argmax(),
                                                 len(proc_ranges))
        speed = self.SPEED
        
        self.publish_drive(speed, steering_angle)
            
    def publish_drive(self, speed, steering):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "car_1_base_link" # Make this a ROS parameter
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
    node = DisparityExtender()
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()
