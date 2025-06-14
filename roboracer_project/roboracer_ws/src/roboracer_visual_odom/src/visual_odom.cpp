#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <System.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <deque>
// Ensure this path is correct for your build:
#include <ImuTypes.h>  // OR <ImuTypes.h> if your include path is set

namespace fs = std::filesystem;

class VisualOdometryNode : public rclcpp::Node {
public:
  VisualOdometryNode() : Node("visual_odom"), slam_(nullptr), last_image_timestamp_(0.0) {
    this->declare_parameter<std::string>("rgb_topic", "/car_1/rgbd_camera/rgb/image_raw");
    this->declare_parameter<std::string>("depth_topic", "/car_1/rgbd_camera/depth/image_raw");
    this->declare_parameter<std::string>("vocabulary_path",
      "/usr/local/ORB_SLAM3/Vocabulary/ORBvoc.txt");
    this->declare_parameter<std::string>("settings_path",
      "/home/dev/roboracer_ws/src/roboracer_description/config/camera_settings.yaml");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("camera_frame", "car_1_rgbd_camera_link");
    this->declare_parameter<double>("max_time_diff", 0.05);

    rgb_topic_ = this->get_parameter("rgb_topic").as_string();
    depth_topic_ = this->get_parameter("depth_topic").as_string();
    vocabulary_path_ = this->get_parameter("vocabulary_path").as_string();
    settings_path_ = this->get_parameter("settings_path").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    camera_frame_ = this->get_parameter("camera_frame").as_string();
    max_time_diff_ = this->get_parameter("max_time_diff").as_double();

    if (!validate_paths()) {
      rclcpp::shutdown();
      return;
    }

    initialize_slam();
    if (!slam_) {
      rclcpp::shutdown();
      return;
    }

    initialize_publishers();

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/car_1/imu", 200,
        std::bind(&VisualOdometryNode::imu_callback, this, std::placeholders::_1));

    rgb_sub_.subscribe(this, rgb_topic_);
    depth_sub_.subscribe(this, depth_topic_);

    sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
      MySyncPolicy(10), rgb_sub_, depth_sub_);
    sync_->registerCallback(
      std::bind(&VisualOdometryNode::image_callback, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Node initialized. Waiting for synchronized images on %s and %s",
      rgb_topic_.c_str(), depth_topic_.c_str());
  }

  ~VisualOdometryNode() override {
    if (slam_) {
      RCLCPP_INFO(get_logger(), "Shutting down ORB-SLAM3");
      slam_->Shutdown();
    }
  }

private:
  std::unique_ptr<ORB_SLAM3::System> slam_;
  std::string rgb_topic_, depth_topic_, vocabulary_path_, settings_path_;
  std::string map_frame_, camera_frame_;
  double max_time_diff_;

  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  using MySyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr features_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub_;

  nav_msgs::msg::Path path_msg_;

  // IMU Buffer (must be Vector3d for ORB-SLAM3!)
  std::deque<ORB_SLAM3::IMU::Point> imu_buffer_;
  double last_image_timestamp_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  bool validate_paths() {
    if (!fs::exists(vocabulary_path_)) {
      RCLCPP_FATAL(get_logger(), "Vocabulary file not found: %s", vocabulary_path_.c_str());
      return false;
    }
    if (!fs::exists(settings_path_)) {
      RCLCPP_FATAL(get_logger(), "Settings file not found: %s", settings_path_.c_str());
      return false;
    }
    return true;
  }

  void initialize_publishers() {
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/visual_odometry/pose", rclcpp::QoS(10).reliable());
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/visual_odometry/trajectory", rclcpp::QoS(10).reliable().transient_local());
    features_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/visual_odometry/features_image", rclcpp::QoS(10).reliable());
    map_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/orb_slam3/map_points", rclcpp::QoS(10).reliable().transient_local());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    path_msg_.header.frame_id = map_frame_;
  }

  void initialize_slam() {
    try {
      std::string empty_session_file = "";
      slam_ = std::make_unique<ORB_SLAM3::System>(
          vocabulary_path_,
          settings_path_,
          ORB_SLAM3::System::IMU_RGBD, // <--- Inertial RGBD mode!
          false,
          0,
          empty_session_file
      );
      RCLCPP_INFO(get_logger(), "ORB-SLAM3 IMU_RGBD initialized successfully");
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "ORB-SLAM3 initialization failed: %s", e.what());
    }
  }

  // IMU callback (must use Vector3d, not Vector3f!)
void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    double t = msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;
    cv::Point3f acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    cv::Point3f gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    imu_buffer_.emplace_back(acc, gyro, t);
    // Clean up old IMU data (>5 seconds old)
    while (!imu_buffer_.empty() && imu_buffer_.front().t < t - 5.0)
        imu_buffer_.pop_front();
}
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                     const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
    rclcpp::Time rgb_time(rgb_msg->header.stamp);
    rclcpp::Time depth_time(depth_msg->header.stamp);
    double time_diff = fabs((rgb_time - depth_time).seconds());
    if (time_diff > max_time_diff_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                          "Images not synchronized. Time difference: %.3f s", time_diff);
      return;
    }

    RCLCPP_INFO_ONCE(get_logger(), "Successfully receiving synchronized RGB and Depth images!");

    static size_t frame_count = 0;
    if (++frame_count % 30 == 0) {
      RCLCPP_INFO(get_logger(), "Processed %zu synchronized image pairs.", frame_count);
    }

    // Process RGB image
    cv::Mat rgb_img;
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(rgb_msg, "bgr8");
      rgb_img = cv_ptr->image;
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "RGB cv_bridge exception: %s", e.what());
      return;
    }

    // Process Depth image
    cv::Mat depth_img;
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(depth_msg);
      if (depth_msg->encoding == "16UC1") {
        cv_ptr->image.convertTo(depth_img, CV_32F, 0.001); // mm to meters
      } else if (depth_msg->encoding == "32FC1") {
        depth_img = cv_ptr->image; // Already in meters
      } else {
        RCLCPP_ERROR(get_logger(), "Unsupported depth image encoding: %s", depth_msg->encoding.c_str());
        return;
      }
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "Depth cv_bridge exception: %s", e.what());
      return;
    }

    if (rgb_img.empty() || depth_img.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty image(s)");
      return;
    }

    process_frame(rgb_img, depth_img, rgb_msg->header.stamp);
  }

  void process_frame(const cv::Mat& rgb_img, const cv::Mat& depth_img, const builtin_interfaces::msg::Time& stamp_ros) {
    double timestamp = stamp_ros.sec + stamp_ros.nanosec * 1e-9;
    // Gather IMU measurements between last and current image
    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
    for (const auto& imu : imu_buffer_) {
      if (imu.t > last_image_timestamp_ && imu.t <= timestamp)
        vImuMeas.push_back(imu);
    }
    last_image_timestamp_ = timestamp;

    try {
      // IMU-enabled call (with IMU measurements vector!)
      Sophus::SE3f Tcw = slam_->TrackRGBD(rgb_img, depth_img, timestamp, vImuMeas);

      // --- Log SLAM State ---
      auto current_state = slam_->GetTrackingState();
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Current ORB-SLAM3 state: %d", static_cast<int>(current_state));

      // --- Feature Visualization ---
      cv::Mat img_with_features = rgb_img.clone();
      cv::Ptr<cv::ORB> detector = cv::ORB::create();
      std::vector<cv::KeyPoint> keypoints_for_vis;
      detector->detect(rgb_img, keypoints_for_vis);
      cv::drawKeypoints(img_with_features, keypoints_for_vis, img_with_features,
                        cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DEFAULT);

      sensor_msgs::msg::Image out_msg;
      cv_bridge::CvImage img_bridge;
      std_msgs::msg::Header header;
      header.stamp = stamp_ros;
      header.frame_id = camera_frame_;
      img_bridge = cv_bridge::CvImage(header, "bgr8", img_with_features);
      img_bridge.toImageMsg(out_msg);
      features_image_pub_->publish(out_msg);

      if (current_state == ORB_SLAM3::Tracking::eTrackingState::LOST) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Tracking lost");
      }
      if (current_state == ORB_SLAM3::Tracking::eTrackingState::OK) {
        publish_results(Tcw, stamp_ros);
        publish_map_points(stamp_ros);
      }

    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Tracking error: %s", e.what());
    }
  }

  void publish_results(const Sophus::SE3f& Tcw, const builtin_interfaces::msg::Time& stamp) {
    const Eigen::Matrix3f Rcw = Tcw.rotationMatrix();
    const Eigen::Vector3f tcw = Tcw.translation();
    const Eigen::Matrix3f Rwc = Rcw.transpose();
    const Eigen::Vector3d twc = -(Rwc * tcw).cast<double>();
    const Eigen::Quaterniond q(Rwc.cast<double>());

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose.position.x = twc.x();
    pose_msg.pose.position.y = twc.y();
    pose_msg.pose.position.z = twc.z();
    pose_msg.pose.orientation = tf2::toMsg(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()));
    pose_pub_->publish(pose_msg);

    path_msg_.poses.push_back(pose_msg);
    path_msg_.header.stamp = stamp;
    path_pub_->publish(path_msg_);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = camera_frame_;
    transform.transform.translation.x = twc.x();
    transform.transform.translation.y = twc.y();
    transform.transform.translation.z = twc.z();
    transform.transform.rotation = pose_msg.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  void publish_map_points(const builtin_interfaces::msg::Time& stamp) {
    if (!slam_) return;
    std::vector<ORB_SLAM3::MapPoint*> pMPs = slam_->GetTrackedMapPoints();

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = stamp;
    cloud_msg.header.frame_id = map_frame_;
    cloud_msg.height = 1;
    cloud_msg.is_dense = false;
    cloud_msg.fields.clear();
    std::vector<std::string> field_names = {"x", "y", "z"};
    for (size_t i = 0; i < field_names.size(); ++i) {
      sensor_msgs::msg::PointField field;
      field.name = field_names[i];
      field.offset = i * 4;
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field.count = 1;
      cloud_msg.fields.push_back(field);
    }
    cloud_msg.point_step = 12;
    cloud_msg.row_step = cloud_msg.point_step * pMPs.size();
    cloud_msg.data.resize(pMPs.size() * cloud_msg.point_step);
    size_t valid_points = 0;
    for (size_t i = 0; i < pMPs.size(); ++i) {
      ORB_SLAM3::MapPoint* pMP = pMPs[i];
      if (pMP && !pMP->isBad()) {
        Eigen::Vector3f pos = pMP->GetWorldPos();
        float* data_ptr = reinterpret_cast<float*>(&cloud_msg.data[valid_points * 12]);
        data_ptr[0] = pos.x();
        data_ptr[1] = pos.y();
        data_ptr[2] = pos.z();
        ++valid_points;
      }
    }
    cloud_msg.width = valid_points;
    cloud_msg.data.resize(valid_points * cloud_msg.point_step);

    map_points_pub_->publish(cloud_msg);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
