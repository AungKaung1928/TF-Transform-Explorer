cmake_minimum_required(VERSION 3.8)
project(tf_explorer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# ==============================================================================
# DEPENDENCIES
# ==============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

# For custom message generation
find_package(rosidl_default_generators REQUIRED)

# ==============================================================================
# CUSTOM MESSAGE GENERATION
# ==============================================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TFDiagnostics.msg"
  DEPENDENCIES std_msgs
)

# ==============================================================================
# STANDALONE NODES
# ==============================================================================
set(NODE_DEPS
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  std_msgs
)

# TF Monitor Node
add_executable(tf_monitor_node src/tf_monitor_node.cpp)
target_include_directories(tf_monitor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(tf_monitor_node ${NODE_DEPS})

# Frame Broadcaster Node
add_executable(frame_broadcaster_node src/frame_broadcaster_node.cpp)
target_include_directories(frame_broadcaster_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(frame_broadcaster_node ${NODE_DEPS})

# TF Anomaly Detector Node
add_executable(tf_anomaly_detector src/tf_anomaly_detector.cpp)
target_include_directories(tf_anomaly_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(tf_anomaly_detector ${NODE_DEPS})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(tf_anomaly_detector "${cpp_typesupport_target}")

# Simple Wiggle Recovery Node
add_executable(simple_wiggle_recovery src/simple_wiggle_recovery.cpp)
target_include_directories(simple_wiggle_recovery PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(simple_wiggle_recovery rclcpp geometry_msgs)

# Patrol Node (autonomous navigation)
add_executable(patrol_node src/patrol_node.cpp)
ament_target_dependencies(patrol_node rclcpp rclcpp_action nav2_msgs geometry_msgs)

# ==============================================================================
# COSTMAP LAYER PLUGIN
# ==============================================================================
add_library(keepout_layer SHARED src/keepout_layer.cpp)
target_include_directories(keepout_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(keepout_layer
  rclcpp
  nav2_costmap_2d
  pluginlib
)

# ==============================================================================
# INSTALL
# ==============================================================================
install(TARGETS
  tf_monitor_node
  frame_broadcaster_node
  tf_anomaly_detector
  simple_wiggle_recovery
  patrol_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  keepout_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  rviz
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# ==============================================================================
# PLUGIN EXPORT
# ==============================================================================
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

ament_export_include_directories(include)
ament_export_libraries(keepout_layer)

ament_package()