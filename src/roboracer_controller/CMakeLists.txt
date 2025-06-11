cmake_minimum_required(VERSION 3.8)
project(roboracer_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ackermann_msgs REQUIRED)

# Install Python package as a module

install(
   DIRECTORY config launch 
   DESTINATION share/${PROJECT_NAME}
 
)

# Install launch files
install(PROGRAMS
   ${PROJECT_NAME}/ackerman_controller.py
   ${PROJECT_NAME}/ackermann_to_twist.py
   ${PROJECT_NAME}/noisy_controller.py
   ${PROJECT_NAME}/static_tf_publisher.py
   ${PROJECT_NAME}/custom_robot_state_publisher.py
   ${PROJECT_NAME}/twist_relay.py
   DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright and cpplint checks (uncomment when ready)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()