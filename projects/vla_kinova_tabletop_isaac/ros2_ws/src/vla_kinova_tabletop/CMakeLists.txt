cmake_minimum_required(VERSION 3.8)
project(vla_kinova_tabletop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

add_executable(joint_state_merger_node src/joint_state_merger.cpp)

ament_target_dependencies(joint_state_merger_node
  rclcpp
  sensor_msgs
)

install(TARGETS
  joint_state_merger_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
