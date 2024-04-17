cmake_minimum_required(VERSION 3.5)
project(schunk_gripper)

# Set C++ standard to 17 if not set
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler-specific options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
#find_package(PythonInterp 3.10 REQUIRED)
#find_package(Python3 3.10 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(CURL REQUIRED)

# Additional include directories
include_directories(
  include
  ${CURL_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Generate ROS interfaces
set(msg_files 
"msg/State.msg"
)

set(srv_files
  "srv/BrakeTest.srv"
  "srv/Acknowledge.srv"
  "srv/ChangeIp.srv"
  "srv/Stop.srv"
  "srv/FastStop.srv"
  "srv/GripperInfo.srv"
  "srv/Softreset.srv"
  "srv/ParameterGet.srv"
  "srv/ParameterSet.srv"
  "srv/PrepareForShutdown.srv"
  "srv/ReleaseForManualMovement.srv"
)
set(action_files
  "action/GripWithPositionAndVelocity.action"
  "action/GripWithPosition.action"
  "action/GripWithVelocity.action"
  "action/Grip.action"
  "action/MoveToAbsolutePosition.action"
  "action/MoveToRelativePosition.action" 
  "action/ReleaseWorkpiece.action" 
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES sensor_msgs control_msgs action_msgs diagnostic_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_dependencies(sensor_msgs control_msgs action_msgs diagnostic_msgs)

#--------------------------------------------------------------------------------
# Build
#--------------------------------------------------------------------------------

# The driver
add_library(schunk_gripper_driver SHARED
  src/schunk_gripper_wrapper.cpp
  src/schunk_gripper_lib.cpp
  src/communication.cpp
)
target_include_directories(schunk_gripper_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(schunk_gripper_driver
  ${CURL_LIBRARIES}
  ${cpp_typesupport_target}
)
ament_target_dependencies(schunk_gripper_driver
  rclcpp
  sensor_msgs
  control_msgs
  rclcpp_action
  rclcpp_components
  diagnostic_updater
  diagnostic_msgs 
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
# Register schunk_gripper_driver as a component node
rclcpp_components_register_nodes(schunk_gripper_driver "SchunkGripperNode")

# Example
add_executable(schunk_gripper_example
  src/gripper_example.cpp
)
ament_target_dependencies(schunk_gripper_example
  rclcpp
  sensor_msgs
  control_msgs
  rclcpp_action
  rclcpp_components
  diagnostic_updater
  diagnostic_msgs
)
target_link_libraries(schunk_gripper_example
  "${cpp_typesupport_target}"
  schunk_gripper_driver
)

#--------------------------------------------------------------------------------
# Install
#--------------------------------------------------------------------------------
install(TARGETS
  schunk_gripper_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS
  schunk_gripper_example
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY 
  include/
  DESTINATION include
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

#--------------------------------------------------------------------------------
# Test
#--------------------------------------------------------------------------------
if(BUILD_TESTING)
endif()

# Ament package configuration
ament_package()

