cmake_minimum_required(VERSION 3.22)
project(pointcloud_to_grid)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)

# ROS2 + PCL + OpenCV
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 添加可执行文件（必须放在 target 操作之前！）
add_executable(pointcloud_to_occupancy_grid_node src/pointcloud_to_occupancy_grid_node.cpp)

# 依赖头文件
target_include_directories(pointcloud_to_occupancy_grid_node PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 链接依赖库
target_link_libraries(pointcloud_to_occupancy_grid_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

# ROS 2 的依赖
ament_target_dependencies(pointcloud_to_occupancy_grid_node
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_conversions
  visualization_msgs
  tf2_geometry_msgs
)

# 安装
install(TARGETS pointcloud_to_occupancy_grid_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
