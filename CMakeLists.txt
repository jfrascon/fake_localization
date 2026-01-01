cmake_minimum_required(VERSION 3.5)
project(fake_localization)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)

include_directories(include)

install(DIRECTORY include/ DESTINATION include)

add_library(fake_localization_component SHARED
  src/fake_localization_ros.cpp
  src/fake_localization_component.cpp
)

ament_target_dependencies(
  fake_localization_component
  rclcpp
  rclcpp_components
  geometry_msgs
  message_filters
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles)

rclcpp_components_register_node(
  fake_localization_component PLUGIN
  "fake_localization::FakeLocalization" EXECUTABLE
  exec_fake_localization_component)

ament_export_targets(export_fake_localization_component)

install(
  TARGETS fake_localization_component
  EXPORT export_fake_localization_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

add_executable(fake_localization_node
  src/fake_localization_node.cpp
  src/fake_localization_ros.cpp
)
ament_target_dependencies(
  fake_localization_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles)

install(TARGETS fake_localization_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
