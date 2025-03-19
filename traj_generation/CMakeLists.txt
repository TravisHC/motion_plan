cmake_minimum_required(VERSION 3.5)
project(traj_generation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找必备组件
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
)

# 定义所有ros2依赖项，需要配合package.xml使用
set(dependencies
  rclcpp
  geometry_msgs
  visualization_msgs
)

# 声明可执行文件
add_executable(click_gen 
  src/click_gen.cpp
)

# 添加ament标准依赖
ament_target_dependencies(click_gen
  ${dependencies}
)

# 显式链接Eigen3（非ROS库需要单独处理）
target_link_libraries(click_gen Eigen3::Eigen)

# 安装可执行文件
install(TARGETS 
  click_gen
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})


# 导出所有依赖
ament_export_dependencies(${dependencies})

# 最终打包声明
ament_package()