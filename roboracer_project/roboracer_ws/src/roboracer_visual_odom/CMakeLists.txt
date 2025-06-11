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

# ——— ORB-SLAM3 locations ———
set(ORB_SLAM3_INC  "/usr/local/include/ORB_SLAM3")  # Updated to installed location
set(ORB_SLAM3_LIB  "/usr/local/lib/libORB_SLAM3.so") # Updated to installed location

# Critical additional includes
set(ORB_SLAM3_EXTRA_INCLUDES
  "/usr/local/include/ORB_SLAM3/CameraModels"
  "/usr/local/include/ORB_SLAM3/Thirdparty/DBoW2/DBoW2"  # Changed from .../include
  "/usr/local/include/ORB_SLAM3/Thirdparty/g2o"
  "/usr/local/include/ORB_SLAM3/Thirdparty/Sophus"
)

# ——— Build the node ———
add_executable(visual_odom src/visual_odom.cpp)

# Include dirs
target_include_directories(visual_odom PRIVATE
  "/usr/local/include/ORB_SLAM3"
  "/usr/local/include/ORB_SLAM3/Thirdparty/DBoW2"  # For DBoW2 includes
  "/usr/local/include/ORB_SLAM3/Thirdparty/DBoW2/DBoW2"  # Direct path to headers
  "/usr/local/include/ORB_SLAM3/Thirdparty/g2o"
  ${ORB_SLAM3_INC}
  ${ORB_SLAM3_EXTRA_INCLUDES}
  ${Sophus_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(visual_odom
  ${ORB_SLAM3_LIB}
  Sophus::Sophus
  ${OpenCV_LIBS}           # Use _LIBS instead of _LIBRARIES
  Eigen3::Eigen
  ${Pangolin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}  # Additional PCL libs
  ${PCL_IO_LIBRARIES}
  GL
  GLEW
  pthread
  dl                       # Add dynamic linking library
)

# Threading support
target_compile_options(visual_odom PRIVATE -pthread)
target_link_options(visual_odom PRIVATE -pthread)

# ROS2 dependencies
ament_target_dependencies(visual_odom
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  nav_msgs
  tf2_ros 
  tf2_geometry_msgs 
  cv_bridge 
  pcl_conversions
)

# Install
install(TARGETS visual_odom
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()