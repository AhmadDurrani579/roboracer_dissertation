#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math
import time

class State:
    FREE = 0
    DANGER = 1

class SafetyStop(Node):
    def __init__(self):
        super().__init__("safety_stop_node")
        # Declare parameters
        self.declare_parameter('warning_distance', 3.0)
        self.declare_parameter('danger_distance', 1.0)
        self.declare_parameter('scan_topic', '/hokuyo/scan')
        self.declare_parameter('safety_stop_topic', 'safety_stop')
        self.declare_parameter('angle_offset', 0.0)
        self.declare_parameter("straight_tolerance_deg", 15.0)
        
        self.warning_distance = self.get_parameter('warning_distance').get_parameter_value().double_value
        self.danger_distance = self.get_parameter('danger_distance').get_parameter_value().double_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.safety_stop_topic = self.get_parameter('safety_stop_topic').get_parameter_value().string_value
        self.angle_offset = self.get_parameter('angle_offset').get_parameter_value().double_value
        
        self.laser_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.laser_callback, 10
        )
        self.safety_stop_pub = self.create_publisher(Bool, self.safety_stop_topic, 10)
        
        self.state = State.FREE
        self.prev_state = State.FREE
        self.last_state_change = time.time()
        self.metadata_logged = False

    def laser_callback(self, msg: LaserScan):
        self.state = State.FREE
        min_range = float('inf')
        directions = []
        
        if not self.metadata_logged:
            self.get_logger().info(
                f"LIDAR: angle_min={math.degrees(msg.angle_min):.2f}°, "
                f"angle_max={math.degrees(msg.angle_max):.2f}°, "
                f"samples={len(msg.ranges)}"
            )
            self.metadata_logged = True

        avg_ranges = msg.ranges  # Use raw ranges (or filtered if you added filtering)

        straight_tolerance_deg = self.get_parameter("straight_tolerance_deg").get_parameter_value().double_value
        straight_tolerance_rad = math.radians(straight_tolerance_deg)

        for idx, r in enumerate(avg_ranges):
            if not math.isinf(r) and not math.isnan(r) and r >= msg.range_min and r <= msg.range_max:
                raw_angle = msg.angle_min + idx * msg.angle_increment
                angle = raw_angle + self.angle_offset
                angle = math.atan2(math.sin(angle), math.cos(angle))
                
                min_range = min(min_range, r)
                
                # Modified: Only consider front-left to center (0° to -15°) for obstacles
                if r <= self.danger_distance + 0.1 and -straight_tolerance_rad <= angle <= 0.0:
                    self.state = State.DANGER
                    
                    if abs(angle) <= straight_tolerance_rad / 2:
                        direction = "Straight"
                    elif angle > 0:
                        direction = "Left"
                    else:
                        direction = "Center"
                    
                    angle_deg = math.degrees(angle)
                    directions.append((direction, r, idx, raw_angle, angle_deg))
        
        current_time = time.time()
        if (current_time - self.last_state_change >= 0.25 and
            (self.state != self.prev_state or 
            (self.state == State.FREE and min_range <= self.danger_distance) or 
            (self.state == State.DANGER and min_range > self.danger_distance + 0.3))):
            
            if min_range <= self.danger_distance:
                self.state = State.DANGER
            elif min_range > self.danger_distance + 0.3:
                self.state = State.FREE
                
            is_safety_stop = Bool()
            is_safety_stop.data = (self.state == State.DANGER)
            self.safety_stop_pub.publish(is_safety_stop)
            
            if self.state == State.DANGER:
                for direction, r, idx, raw_angle, angle_deg in directions:
                    self.get_logger().info(
                        f"Detected obstacle: Direction={direction}, range={r:.2f}m, idx={idx}, "
                        f"raw_angle={math.degrees(raw_angle):.1f}°, angle={angle_deg:.1f}°"
                    )
            else:
                self.get_logger().info(f"State: FREE, Min Range: {min_range:.2f}m")
            
            self.prev_state = self.state
            self.last_state_change = current_time

def main(args=None):
    rclpy.init(args=args)
    node = SafetyStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()