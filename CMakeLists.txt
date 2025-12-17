cmake_minimum_required(VERSION 3.8)
project(tf_explorer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(DEPS
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
)

# TF Monitor Node
add_executable(tf_monitor_node src/tf_monitor_node.cpp)
target_include_directories(tf_monitor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(tf_monitor_node ${DEPS})

# Frame Broadcaster Node
add_executable(frame_broadcaster_node src/frame_broadcaster_node.cpp)
target_include_directories(frame_broadcaster_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(frame_broadcaster_node ${DEPS})

install(TARGETS
  tf_monitor_node
  frame_broadcaster_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()