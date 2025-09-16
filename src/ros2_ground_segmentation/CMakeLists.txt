cmake_minimum_required(VERSION 3.10)
project(ground_segmentation)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(cv_bridge REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation sample_consensus search)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenMP REQUIRED)

# 添加Eigen3依赖
find_package(Eigen3 REQUIRED)



include_directories(
  include 
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}  # 添加Eigen3头文件路径
)

# add_executable(ground_segmentation_node src/ground_segmentation_nodeuse.cpp)
add_executable(ground_segmentation_node src/ground_segmentation_node222.cpp)
ament_target_dependencies(ground_segmentation_node
  rclcpp sensor_msgs nav_msgs tf2 cv_bridge tf2_ros pcl_conversions
)

target_compile_options(ground_segmentation_node PUBLIC "-fopenmp")

target_link_libraries(ground_segmentation_node 
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)


install(TARGETS
  ground_segmentation_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()