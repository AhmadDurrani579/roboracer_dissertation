cmake_minimum_required(VERSION 3.8)
project(roboracer_visual_odom)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ——— Find ROS2 & deps ———
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED) # <--- Ensure message_filters is found here

# ——— ORB-SLAM3 locations ———
set(ORB_SLAM3_INC  "/usr/local/include/ORB_SLAM3")  # Updated to installed location
set(ORB_SLAM3_LIB  "/usr/local/lib/libORB_SLAM3.so") # Updated to installed location

# Critical additional includes
set(ORB_SLAM3_EXTRA_INCLUDES
  "/usr/local/include/ORB_SLAM3/CameraModels"
  "/usr/local/include/ORB_SLAM3/Thirdparty/DBoW2/DBoW2"
  "/usr/local/include/ORB_SLAM3/Thirdparty/g2o"
  "/usr/local/include/ORB_SLAM3/Thirdparty/Sophus"
)

# ——— Build the visual_odom node ———
add_executable(visual_odom src/visual_odom.cpp) # Assuming your ORB-SLAM3 node source is visual_odom.cpp

# Include dirs for visual_odom
target_include_directories(visual_odom PRIVATE
  "/usr/local/include/ORB_SLAM3"
  "/usr/local/include/ORB_SLAM3/Thirdparty/DBoW2"
  "/usr/local/include/ORB_SLAM3/Thirdparty/DBoW2/DBoW2"
  "/usr/local/include/ORB_SLAM3/Thirdparty/g2o"
  ${ORB_SLAM3_INC}
  ${ORB_SLAM3_EXTRA_INCLUDES}
  ${Sophus_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Link libraries for visual_odom
target_link_libraries(visual_odom
  ${ORB_SLAM3_LIB}
  Sophus::Sophus
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${Pangolin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  GL
  GLEW
  pthread
  dl
)

# Threading support for visual_odom
target_compile_options(visual_odom PRIVATE -pthread)
target_link_options(visual_odom PRIVATE -pthread)

# ROS2 dependencies for visual_odom
ament_target_dependencies(visual_odom
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  pcl_conversions
  message_filters # <--- ensure message_filters is here if visual_odom uses it
)


# --- NEW: Build the rgbd_pointcloud_publisher node ---
add_executable(rgbd_pointcloud_publisher src/rgbd_pointcloud_publisher.cpp)

# Include dirs for rgbd_pointcloud_publisher (minimal, mostly just OpenCV and ROS)
target_include_directories(rgbd_pointcloud_publisher PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

# Link libraries for rgbd_pointcloud_publisher
target_link_libraries(rgbd_pointcloud_publisher
  ${OpenCV_LIBS}
)

# ROS2 dependencies for rgbd_pointcloud_publisher
ament_target_dependencies(rgbd_pointcloud_publisher
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters # <--- Needed for ApproximateTimeSynchronizer
)


# ——— Install Targets ———
install(TARGETS
  visual_odom
  rgbd_pointcloud_publisher # <--- Add the new executable here
  DESTINATION lib/${PROJECT_NAME}
)

# ——— Export dependencies for the package ———
# This ensures other packages that depend on roboracer_visual_odom can find its dependencies
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  OpenCV # Export OpenCV if other packages might use it via this one
  Eigen3
  Pangolin
  Sophus
  PCL
  pcl_conversions
  message_filters # <--- Export message_filters
)

ament_package()