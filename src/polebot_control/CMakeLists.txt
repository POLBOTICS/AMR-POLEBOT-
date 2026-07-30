cmake_minimum_required(VERSION 3.8)
project(polebot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(polebot_interfaces REQUIRED)

# ===== C++ executables =====
add_executable(diff_drive_controller
  src/diff_drive_controller.cpp
)
ament_target_dependencies(diff_drive_controller
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  polebot_interfaces
)

add_executable(odometry_publisher
  src/odometry_publisher.cpp
)
ament_target_dependencies(odometry_publisher
  rclcpp
  nav_msgs
  tf2
  tf2_ros
  polebot_interfaces
)

install(TARGETS
  diff_drive_controller
  odometry_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config include DESTINATION share/${PROJECT_NAME})

# ===== Python scripts =====
install(PROGRAMS
  polebot_control/robot_status_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME robot_status_publisher
)

ament_package()
