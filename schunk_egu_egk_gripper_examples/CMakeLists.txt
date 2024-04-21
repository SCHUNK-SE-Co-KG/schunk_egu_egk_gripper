cmake_minimum_required(VERSION 3.22)
project(schunk_egu_egk_gripper_examples)

# Set C++ standard to 17 if not set
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler-specific options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(schunk_egu_egk_gripper_interfaces REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

#--------------------------------------------------------------------------------
# Build
#--------------------------------------------------------------------------------

# Example
add_executable(${PROJECT_NAME}
  src/gripper_example.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  schunk_egu_egk_gripper_interfaces
  rclcpp
  sensor_msgs
  control_msgs
  rclcpp_action
  diagnostic_msgs
)

#--------------------------------------------------------------------------------
# Install
#--------------------------------------------------------------------------------
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
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
