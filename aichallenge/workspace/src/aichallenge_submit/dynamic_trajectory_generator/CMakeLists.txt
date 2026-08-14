cmake_minimum_required(VERSION 3.10)
project(dynamic_trajectory_generator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(lanelet2_extension REQUIRED)
find_package(lanelet2_routing REQUIRED)
find_package(lanelet2_traffic_rules REQUIRED)

add_executable(dynamic_trajectory_generator_node src/dynamic_trajectory_generator.cpp)

ament_target_dependencies(dynamic_trajectory_generator_node
  rclcpp
  geometry_msgs
  autoware_auto_planning_msgs
  tf2_ros
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  lanelet2_extension
  lanelet2_routing
  lanelet2_traffic_rules
)

install(TARGETS dynamic_trajectory_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()