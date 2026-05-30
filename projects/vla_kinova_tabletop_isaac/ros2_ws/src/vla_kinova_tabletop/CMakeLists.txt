cmake_minimum_required(VERSION 3.8)
project(vla_kinova_tabletop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_servo REQUIRED)

add_executable(pose_tracking_node src/pose_tracking_node.cpp)
ament_target_dependencies(pose_tracking_node
  rclcpp
  std_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_servo
)

install(TARGETS pose_tracking_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/home_robot.py
  scripts/servo_sine_test.py
  scripts/twist_pose_tracking_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
