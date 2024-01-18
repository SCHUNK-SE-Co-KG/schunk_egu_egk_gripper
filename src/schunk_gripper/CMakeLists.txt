cmake_minimum_required(VERSION 3.5)
project(schunk_gripper)

# Set C++ standard to 14 if not set
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compiler-specific options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(CURL REQUIRED)


set(node_plugins "")
# Generate messages, services, and actions
set(msg_files "msg/State.msg")
set(srv_files
   "srv/Acknowledge.srv"
   "srv/ChangeIp.srv"
   "srv/Stop.srv"
   "srv/FastStop.srv"
   "srv/GripperInfo.srv"
   "srv/Softreset.srv"
   "srv/PrepareForShutdown.srv"
   "srv/ReleaseForManMov.srv"
)
set(action_files
   "action/GripWithPosVel.action"
   "action/GripWithPos.action"
   "action/GripWithVel.action"
   "action/Grip.action"
   "action/MovAbsPos.action"
   "action/MovRelPos.action" 
   "action/ReleaseWorkpiece.action" 
)

# Specify additional locations of header files
include_directories(
  include
  ${Boost_INCLUDE_DIRS} 
  ${CURL_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Declare a C++ library
add_library(schunk_driverlib SHARED
  src/schunk_gripper_lib.cpp
  src/communication.cpp
)
# Generate ROS interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES sensor_msgs control_msgs action_msgs diagnostic_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

add_library(schunk_gripper_driver SHARED
  src/schunk_gripper_wrapper.cpp
  src/schunk_gripper_lib.cpp
  src/communication.cpp)
target_compile_definitions(schunk_gripper_driver
  PRIVATE "COMPOSITION_BUILDING_DLL")
  target_link_libraries(schunk_gripper_driver
  ${CURL_LIBRARIES}
  -lcurl)
ament_target_dependencies(schunk_gripper_driver
rclcpp sensor_msgs control_msgs rclcpp_action rclcpp_components diagnostic_updater diagnostic_msgs)
rclcpp_components_register_nodes(schunk_gripper_driver "SchunkGripperNode")
# Link libraries to the executable
set(node_plugins "${node_plugins}SchunkGripperNode;$<TARGET_FILE:schunk_gripper_wrapper>\n")

add_executable(schunk_gripper_example src/gripper_example.cpp)

ament_target_dependencies(schunk_gripper_example
  rclcpp sensor_msgs control_msgs rclcpp_action rclcpp_components diagnostic_updater diagnostic_msgs
)

# Install targets
install(TARGETS
  schunk_gripper_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

  install(TARGETS
  schunk_gripper_example
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  include/
  DESTINATION include
)
# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


# Export include directories and dependencies
ament_export_include_directories(include)
ament_export_dependencies(sensor_msgs control_msgs action_msgs diagnostic_msgs)

# Specify ROS interfaces for the executable
rosidl_target_interfaces(schunk_gripper_driver
  ${PROJECT_NAME} 
  "rosidl_typesupport_cpp"
)
rosidl_target_interfaces(schunk_gripper_example
  ${PROJECT_NAME} 
  "rosidl_typesupport_cpp"
)
# Ament package configuration
ament_package()
