cmake_minimum_required(VERSION 3.5)
project(motion_plan)

# 默认使用C++17（ROS2 Humble推荐标准）
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 查找必备组件
find_package(ament_cmake REQUIRED)

# 最终打包声明
ament_package()