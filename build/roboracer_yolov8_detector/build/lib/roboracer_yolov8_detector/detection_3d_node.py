import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import numpy as np
import message_filters
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy

class Detection3DNode(Node):
    def __init__(self):
        super().__init__('detection_3d_node')
        self.bridge = CvBridge()
        self.K = None
        self.fx = self.fy = self.cx = self.cy = None

        # Subscribers (synchronized)
        self.detection_sub = message_filters.Subscriber(self, Detection2DArray, '/yolo_detections')
        self.depth_sub = message_filters.Subscriber(self, Image, '/car_1/rgbd_camera/depth/image_raw')
        self.info_sub = self.create_subscription(CameraInfo, '/car_1/rgbd_camera/rgb/camera_info', self.info_cb, 10)
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.detection_sub, self.depth_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.synced_callback)

        # Publisher for MarkerArray
        marker_qos = QoSProfile(depth=10)
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.create_publisher(MarkerArray, '/yolo_3d_markers', marker_qos)
        self.get_logger().info('Detection3DNode started (Markers).')

    def info_cb(self, msg):
        self.K = np.array(msg.k).reshape((3, 3))
        self.fx = self.K[0, 0]
        self.fy = self.K[1, 1]
        self.cx = self.K[0, 2]
        self.cy = self.K[1, 2]

    def synced_callback(self, detections_msg, depth_msg):
        if self.K is None:
            self.get_logger().warn('Camera intrinsics not received yet.')
            return

        # Convert depth image to numpy (float32 meters)
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        if depth_img.dtype != np.float32:
            depth_img = depth_img.astype(np.float32) / 1000.0

        points, classes = [], []
        for det in detections_msg.detections:
            bbox = det.bbox
            x = int(bbox.center.position.x)
            y = int(bbox.center.position.y)
            if 0 <= x < depth_img.shape[1] and 0 <= y < depth_img.shape[0]:
                Z = depth_img[y, x]
                if np.isnan(Z) or Z <= 0.1 or Z > 10.0:
                    continue
                X = (x - self.cx) * Z / self.fx
                Y = (y - self.cy) * Z / self.fy
                points.append((float(X), float(Y), float(Z)))

                # Extract class name from detection (vision_msgs format)
                if det.results:
                    # If using vision_msgs, the class label is usually a string in results[0].hypothesis.class_id
                    class_id = det.results[0].hypothesis.class_id
                else:
                    class_id = "unknown"
                classes.append(str(class_id))

        if points:
            self.publish_markers(points, classes, detections_msg.header)

    def publish_markers(self, points, classes, header):
        marker_array = MarkerArray()
        for i, (pt, cls) in enumerate(zip(points, classes)):
            # Sphere marker
            marker = Marker()
            marker.header = header
            marker.header.frame_id = "map"
            marker.ns = "yolo_detection"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = pt
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25
            # Color by class
            color = self.get_color(cls)
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = *color, 1.0
            marker_array.markers.append(marker)

            # Text label
            text_marker = Marker()
            text_marker.header = header
            text_marker.header.frame_id = "map" 
            text_marker.ns = "yolo_label"
            text_marker.id = 1000 + i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = pt[0]
            text_marker.pose.position.y = pt[1]
            text_marker.pose.position.z = pt[2] + 0.3
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.2
            text_marker.color.r, text_marker.color.g, text_marker.color.b, text_marker.color.a = 1.0, 1.0, 0.0, 1.0  # Yellow
            text_marker.text = cls
            marker_array.markers.append(text_marker)
        self.marker_pub.publish(marker_array)

    def get_color(self, cls):
        # Simple hardcoded colors for common classes, extend as needed
        if cls in ["0", "person"]:
            return (0.0, 1.0, 0.0)  # Green
        elif cls in ["1", "car"]:
            return (0.0, 0.0, 1.0)  # Blue
        elif cls in ["2", "bicycle"]:
            return (1.0, 0.0, 1.0)  # Magenta
        else:
            return (1.0, 0.0, 0.0)  # Red as default

def main(args=None):
    rclpy.init(args=args)
    node = Detection3DNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
