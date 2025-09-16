cmake_minimum_required(VERSION 3.8)
project(integrated_navigation)

# 设置策略以消除警告
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wno-unknown-pragmas -Wno-deprecated-declarations)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# 修改PCL相关查找方式
find_package(PCL 1.8 REQUIRED COMPONENTS common io conversions)
find_package(pcl_conversions REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IntegratedData.msg"
  DEPENDENCIES nav_msgs geometry_msgs sensor_msgs
)

add_executable(integrated_navigation_node src/integrated_navigation_node.cpp)
ament_target_dependencies(integrated_navigation_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  pcl_conversions
)

# 添加包含目录
include_directories(
  ${PCL_INCLUDE_DIRS}
)

# 修改链接库部分
target_link_libraries(integrated_navigation_node
  ${PCL_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(integrated_navigation_node "${cpp_typesupport_target}")

install(TARGETS
  integrated_navigation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()