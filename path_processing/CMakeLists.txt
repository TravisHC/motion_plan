cmake_minimum_required(VERSION 3.5)
project(path_processing)

set(CMAKE_BUILD_TYPE Debug)

# 默认使用C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

add_executable(path_processing_node src/main.cpp)
ament_target_dependencies(path_processing_node rclcpp visualization_msgs geometry_msgs nav_msgs)

install(TARGETS
  path_processing_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()