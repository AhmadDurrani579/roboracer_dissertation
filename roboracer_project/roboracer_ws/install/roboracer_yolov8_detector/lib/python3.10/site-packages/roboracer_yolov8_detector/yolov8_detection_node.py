import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Import YOLO from ultralytics
from ultralytics import YOLO

# Import vision_msgs for 2D detections
from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import BoundingBox2D
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import ObjectHypothesis # <-- NEW: Needed for the nested 'hypothesis' field
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Point

class YOLODetectionNode(Node):
    def __init__(self):
        super().__init__('yolov8_detection_node')

        # Declare parameters
        self.declare_parameter('model_path', 'yolov8n.pt') # Default to nano model
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('image_topic', '/car_1/rgbd_camera/rgb/image_raw')
        self.declare_parameter('detections_output_topic', '/yolo_detections')
        self.declare_parameter('annotated_image_output_topic', '/yolo_annotated_image')

        # Get parameters
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.detections_output_topic = self.get_parameter('detections_output_topic').get_parameter_value().string_value
        self.annotated_image_output_topic = self.get_parameter('annotated_image_output_topic').get_parameter_value().string_value

        self.get_logger().info(f"Loading YOLOv8 model from: {self.model_path}")
        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info("YOLOv8 model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLOv8 model: {e}")
            rclpy.shutdown()
            return

        self.cv_bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            rclpy.qos.qos_profile_sensor_data # Qos profile for camera data
        )
        self.get_logger().info(f"Subscribing to image topic: {self.image_topic}")

        # Publishers
        self.detections_pub = self.create_publisher(
            Detection2DArray,
            self.detections_output_topic,
            10 # QoS queue size
        )
        self.get_logger().info(f"Publishing 2D detections to: {self.detections_output_topic}")

        self.annotated_image_pub = self.create_publisher(
            Image,
            self.annotated_image_output_topic,
            10 # QoS queue size
        )
        self.get_logger().info(f"Publishing annotated images to: {self.annotated_image_output_topic}")


    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        results = self.model.predict(source=cv_image, conf=self.conf_threshold, verbose=False)

        detections_array_msg = Detection2DArray()
        detections_array_msg.header = msg.header

        annotated_image = cv_image.copy()

        if results:
            for r in results:
                for box in r.boxes:
                    conf = float(box.conf)
                    if conf < self.conf_threshold:
                        continue

                    class_id = int(box.cls)
                    if class_id < 0 or class_id >= len(self.model.names):
                        self.get_logger().warn(f"Invalid class_id: {class_id}. Skipping detection.")
                        continue
                    class_name = self.model.names[class_id]

                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    detection_msg = Detection2D()
                    detection_msg.header = msg.header

                    bbox2d = BoundingBox2D()
                    if bbox2d.center.position is None:
                        bbox2d.center.position = Point()

                    # --- DEBUGGING LINES (Keep for now) ---
                    self.get_logger().info(f"DEBUG: Type of bbox2d: {type(bbox2d)}")
                    self.get_logger().info(f"DEBUG: Type of bbox2d.center: {type(bbox2d.center)}")
                    self.get_logger().info(f"DEBUG: Dir of bbox2d.center: {dir(bbox2d.center)}")
                    self.get_logger().info(f"DEBUG: Type of bbox2d.center.position: {type(bbox2d.center.position)}")
                    self.get_logger().info(f"DEBUG: Dir of bbox2d.center.position: {dir(bbox2d.center.position)}")
                    # --- END DEBUGGING LINES ---

                    bbox2d.center.position.x = float(x1 + x2) / 2.0
                    bbox2d.center.position.y = float(y1 + y2) / 2.0
                    bbox2d.center.theta = 0.0 # Set theta for Pose2D
                    bbox2d.size_x = float(x2 - x1)
                    bbox2d.size_y = float(y2 - y1)
                    detection_msg.bbox = bbox2d

                    hypothesis = ObjectHypothesisWithPose()
                    
                    # Ensure the nested 'hypothesis' field is instantiated
                    if hypothesis.hypothesis is None:
                        hypothesis.hypothesis = ObjectHypothesis()

                    # --- NEW DEBUGGING LINES FOR OBJECTHYPOTHESISWITHPOSE (Keep for now) ---
                    # --- END NEW DEBUGGING LINES ---

                    # Access class_id and score through the nested 'hypothesis' field
                    hypothesis.hypothesis.class_id = str(class_id)
                    hypothesis.hypothesis.score = conf
                    
                    detection_msg.results.append(hypothesis)
                    detections_array_msg.detections.append(detection_msg)

                    color = (0, 255, 0)
                    cv2.rectangle(annotated_image, (x1, y1), (x2, y2), color, 2)
                    label = f"{class_name}: {conf:.2f}"
                    cv2.putText(annotated_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        self.detections_pub.publish(detections_array_msg)

        try:
            annotated_image_msg = self.cv_bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            annotated_image_msg.header = msg.header
            self.annotated_image_pub.publish(annotated_image_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to convert and publish annotated image: {e}")
            
            
def main(args=None):
    rclpy.init(args=args)
    yolo_node = YOLODetectionNode()
    rclpy.spin(yolo_node)
    yolo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()