#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <chrono> // For chrono_literals
#include <string>

class RGBDPointCloudPublisher : public rclcpp::Node {
public:
  RGBDPointCloudPublisher() : Node("rgbd_pointcloud_publisher") {
    // Declare parameters for topics and camera intrinsics
    this->declare_parameter<std::string>("rgb_topic", "/car_1/rgbd_camera/rgb/image_raw");
    this->declare_parameter<std::string>("depth_topic", "/car_1/rgbd_camera/depth/image_raw");
    this->declare_parameter<std::string>("output_pointcloud_topic", "/rgbd_point_cloud");
    this->declare_parameter<std::string>("camera_frame_id", "car_1_rgbd_camera_link");
    this->declare_parameter<double>("max_sync_time_diff", 0.05); // Max time diff for message_filters

    // Camera Intrinsics (from your camera_settings.yaml)
    this->declare_parameter<double>("camera.fx", 535.4);
    this->declare_parameter<double>("camera.fy", 539.2);
    this->declare_parameter<double>("camera.cx", 320.1);
    this->declare_parameter<double>("camera.cy", 247.6);
    this->declare_parameter<double>("min_depth_m", 0.1); // Minimum valid depth in meters
    this->declare_parameter<double>("max_depth_m", 6.0); // Maximum valid depth in meters


    // Get parameter values
    rgb_topic_ = this->get_parameter("rgb_topic").as_string();
    depth_topic_ = this->get_parameter("depth_topic").as_string();
    output_pointcloud_topic_ = this->get_parameter("output_pointcloud_topic").as_string();
    camera_frame_id_ = this->get_parameter("camera_frame_id").as_string();
    max_sync_time_diff_ = this->get_parameter("max_sync_time_diff").as_double();
    
    fx_ = this->get_parameter("camera.fx").as_double();
    fy_ = this->get_parameter("camera.fy").as_double();
    cx_ = this->get_parameter("camera.cx").as_double();
    cy_ = this->get_parameter("camera.cy").as_double();
    min_depth_m_ = this->get_parameter("min_depth_m").as_double();
    max_depth_m_ = this->get_parameter("max_depth_m").as_double();

    // Setup publishers
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_pointcloud_topic_, rclcpp::QoS(10)); // QoS can be adjusted if needed

    // Setup subscribers and approximate time synchronizer
    rgb_sub_.subscribe(this, rgb_topic_);
    depth_sub_.subscribe(this, depth_topic_);

    sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
        MySyncPolicy(10), rgb_sub_, depth_sub_); // Queue size for synchronizer
    sync_->registerCallback(
        std::bind(&RGBDPointCloudPublisher::synchronized_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "RGBDPointCloudPublisher node initialized.");
    RCLCPP_INFO(this->get_logger(), "Subscribing to RGB: %s, Depth: %s", 
                rgb_topic_.c_str(), depth_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to PointCloud: %s", 
                output_pointcloud_topic_.c_str());
  }

private:
  // --- ROS 2 Members ---
  std::string rgb_topic_;
  std::string depth_topic_;
  std::string output_pointcloud_topic_;
  std::string camera_frame_id_;
  double max_sync_time_diff_;

  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  using MySyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

  // --- Camera Intrinsics ---
  double fx_, fy_, cx_, cy_;
  double min_depth_m_, max_depth_m_;


  // --- Synchronized Image Callback ---
  void synchronized_callback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                             const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
    // Check synchronization timestamp difference
    rclcpp::Time rgb_time(rgb_msg->header.stamp);
    rclcpp::Time depth_time(depth_msg->header.stamp);
    double time_diff = fabs((rgb_time - depth_time).seconds());
    if (time_diff > max_sync_time_diff_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Synchronized images have large time difference: %.4f s (max allowed: %.4f s). Skipping.", 
                          time_diff, max_sync_time_diff_);
      return;
    }

    cv::Mat rgb_img;
    try {
      rgb_img = cv_bridge::toCvCopy(rgb_msg, "bgr8")->image;
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "RGB cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat depth_img;
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(depth_msg);
      if (depth_msg->encoding == "16UC1") {
        cv_ptr->image.convertTo(depth_img, CV_32F, 0.001); // Convert 16-bit depth (mm) to 32-bit float (meters)
      } else if (depth_msg->encoding == "32FC1") {
        depth_img = cv_ptr->image; // Already 32-bit float (meters)
      } else {
        RCLCPP_ERROR(this->get_logger(), "Unsupported depth image encoding: %s. Expected 16UC1 or 32FC1.", depth_msg->encoding.c_str());
        return;
      }
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Depth cv_bridge exception: %s", e.what());
      return;
    }

    if (rgb_img.empty() || depth_img.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty image(s).");
      return;
    }
    if (rgb_img.size() != depth_img.size()) {
      RCLCPP_ERROR(this->get_logger(), "RGB and Depth image sizes do not match. RGB: %dx%d, Depth: %dx%d",
                  rgb_img.cols, rgb_img.rows, depth_img.cols, depth_img.rows);
      return;
    }

    publish_dense_pointcloud(rgb_img, depth_img, rgb_msg->header.stamp);
  }

  // --- Point Cloud Generation Function ---
  void publish_dense_pointcloud(const cv::Mat& rgb_img, const cv::Mat& depth_img, const builtin_interfaces::msg::Time& stamp_ros) {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = stamp_ros;
    cloud_msg.header.frame_id = camera_frame_id_;
    cloud_msg.height = 1; // Unordered point cloud
    cloud_msg.is_dense = false; // Contains NaNs or infs, or invalid points

    // Define point cloud fields (x, y, z, rgb)
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                     "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                     "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                     "rgb", 1, sensor_msgs::msg::PointField::UINT32); // Use UINT32 for packed RGB

    modifier.resize(rgb_img.rows * rgb_img.cols); // Allocate max possible points

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_rgb(cloud_msg, "rgb");

    size_t num_valid_points = 0;
    for (int v = 0; v < rgb_img.rows; ++v) {
      for (int u = 0; u < rgb_img.cols; ++u) {
        float d = depth_img.at<float>(v, u);

        // Check for valid depth (not NaN/Inf and within min/max range)
        if (std::isnan(d) || std::isinf(d) || d <= min_depth_m_ || d > max_depth_m_) {
          continue; // Skip invalid depth points
        }

        // Project pixel to 3D point (Pinhole Camera Model)
        float Z = d;
        float X = (static_cast<float>(u) - cx_) * Z / fx_;
        float Y = (static_cast<float>(v) - cy_) * Z / fy_;

        // Get color from RGB image (OpenCV typically loads BGR)
        cv::Vec3b rgb = rgb_img.at<cv::Vec3b>(v, u);
        uint32_t rgb_packed = (static_cast<uint32_t>(rgb[2]) << 16 | // Red
                               static_cast<uint32_t>(rgb[1]) << 8  | // Green
                               static_cast<uint32_t>(rgb[0]));       // Blue

        // Populate point cloud iterators
        *iter_x = X;
        *iter_y = Y;
        *iter_z = Z;
        *iter_rgb = rgb_packed;

        ++iter_x; ++iter_y; ++iter_z; ++iter_rgb;
        num_valid_points++;
      }
    }

    modifier.resize(num_valid_points); // Resize to actual number of valid points
    pointcloud_pub_->publish(cloud_msg);
    // RCLCPP_INFO(this->get_logger(), "Published point cloud with %zu points.", num_valid_points);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RGBDPointCloudPublisher>());
  rclcpp::shutdown();
  return 0;
}