cmake_minimum_required(VERSION 3.22)
project(schunk_egu_egk_gripper_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
    action/Grip.action
    action/GripWithPosition.action
    action/GripWithPositionAndVelocity.action
    action/GripWithVelocity.action
    action/MoveToAbsolutePosition.action
    action/MoveToRelativePosition.action
    action/ReleaseWorkpiece.action
    msg/State.msg
    srv/Acknowledge.srv
    srv/BrakeTest.srv
    srv/ChangeIp.srv
    srv/FastStop.srv
    srv/GripperInfo.srv
    srv/ParameterGet.srv
    srv/ParameterSet.srv
    srv/PrepareForShutdown.srv
    srv/ReleaseForManualMovement.srv
    srv/Softreset.srv
    srv/Stop.srv
  DEPENDENCIES
    std_msgs
    action_msgs
    control_msgs
    sensor_msgs
    diagnostic_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
