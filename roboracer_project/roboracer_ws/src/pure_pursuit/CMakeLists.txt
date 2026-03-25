cmake_minimum_required(VERSION 3.5)
project(pure_pursuit)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake          REQUIRED)
find_package(rclcpp               REQUIRED)
find_package(geometry_msgs        REQUIRED)
find_package(nav_msgs             REQUIRED)
find_package(ackermann_msgs       REQUIRED)
find_package(sensor_msgs          REQUIRED)
find_package(std_msgs             REQUIRED)
find_package(visualization_msgs   REQUIRED)
find_package(tf2                  REQUIRED)
find_package(tf2_ros              REQUIRED)
find_package(tf2_geometry_msgs    REQUIRED)
find_package(eigen3_cmake_module  REQUIRED)
find_package(Eigen3               REQUIRED)





include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}     
)


add_executable(pure_pursuit
  src/pure_pursuit.cpp
)

ament_target_dependencies(pure_pursuit
  rclcpp
  ackermann_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
)

target_include_directories(pure_pursuit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)


target_link_libraries(pure_pursuit
  ${qpOASES_LIBRARY}              
)


add_executable(waypoint_visualizer
  src/waypoint_visualizer.cpp
)

ament_target_dependencies(waypoint_visualizer
  rclcpp
  geometry_msgs
  std_msgs
  visualization_msgs
  eigen3_cmake_module
  Eigen3
)


install(TARGETS
  pure_pursuit
  waypoint_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
