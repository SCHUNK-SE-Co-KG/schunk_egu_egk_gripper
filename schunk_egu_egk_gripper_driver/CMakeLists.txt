cmake_minimum_required(VERSION 3.22)
project(schunk_egu_egk_gripper_driver)

# Set C++ standard to 17 if not set
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler-specific options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default CMake build type
if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message(STATUS "${PROJECT_NAME}: No CMake build type specified. Building in 'Release' mode.")
  message(STATUS "${PROJECT_NAME}: You can specify the build type in colcon with --cmake-args -DCMAKE_BUILD_TYPE=Release|RelWithDebInfo|Debug.")
  set(CMAKE_BUILD_TYPE Release)
endif()

# Find required packages
find_package(schunk_egu_egk_gripper_library REQUIRED)
find_package(schunk_egu_egk_gripper_interfaces REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)


#--------------------------------------------------------------------------------
# Build
#--------------------------------------------------------------------------------

add_library(${PROJECT_NAME} SHARED
  src/schunk_gripper_wrapper.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  control_msgs
  rclcpp_action
  rclcpp_components
  diagnostic_updater
  diagnostic_msgs
  schunk_egu_egk_gripper_interfaces
  std_srvs
)
target_link_libraries(${PROJECT_NAME}
  Schunk::schunk_egu_egk_gripper_library
)

# Make our component discoverable by the ROS2 component tooling
rclcpp_components_register_nodes(${PROJECT_NAME} "SchunkGripperNode")

#--------------------------------------------------------------------------------
# Install
#--------------------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

#--------------------------------------------------------------------------------
# Test
#--------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Ament package configuration
ament_package()
