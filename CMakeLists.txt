cmake_minimum_required(VERSION 3.22)
project(fake_localization)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual -Wnull-dereference -Wunused-parameter)
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

install(DIRECTORY include/ DESTINATION include)

add_library(fake_localization_component SHARED
            src/fake_localization_ros.cpp src/fake_localization_component.cpp)

target_include_directories(
  fake_localization_component
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

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
  fake_localization_component PLUGIN "fake_localization::FakeLocalization"
  EXECUTABLE exec_fake_localization_component)

ament_export_dependencies(
  angles
  geometry_msgs
  message_filters
  nav_msgs
  rclcpp
  rclcpp_components
  tf2
  tf2_geometry_msgs
  tf2_ros)

ament_export_include_directories(include)

ament_export_targets(export_fake_localization_component HAS_LIBRARY_TARGET)

install(
  TARGETS fake_localization_component
  EXPORT export_fake_localization_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

add_executable(fake_localization_node src/fake_localization_node.cpp
                                      src/fake_localization_ros.cpp)

target_include_directories(fake_localization_node PRIVATE include)

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

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)

install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Exclude linters that require package-specific configuration.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_clang_format
    ament_cmake_cpplint
    ament_cmake_flake8
    ament_cmake_cppcheck)

  # Register the remaining linters declared as test dependencies in package.xml.
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_clang_format REQUIRED)

  ament_clang_format(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.clang-format" include src test)

  find_package(ament_cmake_cpplint REQUIRED)

  ament_cpplint(
    MAX_LINE_LENGTH
    120
    FILTERS
    -whitespace/newline
    -whitespace/forcolon
    -build/include_order
    -runtime/indentation_namespace
    -legal/copyright)

  find_package(ament_cmake_flake8 REQUIRED)

  ament_flake8(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/ament_flake8.ini")

  find_package(ament_cmake_cppcheck REQUIRED)

  # Check only headers and source files.
  # Some gtest macros such as TEST_F and TEST_P can produce cppcheck false positives in test
  # sources.
  ament_cppcheck(include src INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include")

  # ament_cppcheck skips cppcheck versions known to be slow unless this environment variable
  # explicitly allows them.
  set_tests_properties(cppcheck PROPERTIES ENVIRONMENT "AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1")

  ament_add_gtest(test_transform_math test/test_transform_math.cpp)
  target_include_directories(test_transform_math PRIVATE include)
  ament_target_dependencies(test_transform_math tf2)

  ament_add_pytest_test(test_launch_contract test/test_launch_contract.py)
endif()

ament_package()
