cmake_minimum_required(VERSION 3.8)
project(bike_control)

# 默认使用 C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)  
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)  
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(integrated_navigation REQUIRED)
find_package(serial REQUIRED)

# 包含目录设置
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 添加可视化库
add_library(bike_visualizer
  src/bike_visualizer.cpp
)
ament_target_dependencies(bike_visualizer
  rclcpp
  sensor_msgs
  cv_bridge
  geometry_msgs  
  integrated_navigation
)
target_link_libraries(bike_visualizer
  ${OpenCV_LIBS}
  cv_bridge::cv_bridge 
)

# 添加可执行文件
add_executable(ebike_control
  src/main.cpp
  src/bike_model.cpp
  src/bike_controller.cpp
  src/trajectory_generator.cpp
  src/motor.cpp
  src/crc8.cpp
)

target_include_directories(
  ebike_control PRIVATE 
    ${CMAKE_SOURCE_DIR}/include/bike_control     # include
    ${serial_INCLUDE_DIRS}       # serial库的头文件
)

# 添加依赖
ament_target_dependencies(ebike_control
  rclcpp
  nav_msgs
  tf2
  geometry_msgs
  integrated_navigation
)

# 链接库
target_link_libraries(ebike_control
  bike_visualizer
  ${OpenCV_LIBS}
  ${serial_LIBRARIES} 
  pthread
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)
# 安装目标
install(TARGETS 
  ebike_control
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS bike_visualizer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

ament_package()

# 设置调试信息
set(CMAKE_BUILD_TYPE Debug)