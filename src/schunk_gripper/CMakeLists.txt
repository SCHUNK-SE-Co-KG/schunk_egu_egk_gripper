cmake_minimum_required(VERSION 3.5)
project(schunk_gripper)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(OpenSSL REQUIRED)
find_package(CURL REQUIRED)

## Generate messages in the 'msg' folder

set(msg_files
   "msg/State.msg"
 )


## Generate services in the 'srv' folder
  set(srv_files
   "srv/Acknowledge.srv"
   "srv/Stop.srv"
   "srv/FastStop.srv"
   "srv/GripperInfo.srv"
   "srv/Softreset.srv"
   "srv/PrepareForShutdown.srv"
   "srv/ReleaseForManMov.srv"
   )

## Generate actions in the 'action' folder
 set(action_files
   "action/GripWithPosEgk.action"
   "action/GripWithPosEgu.action"
   "action/GripEgk.action"
   "action/GripEgu.action"
   "action/MovAbsPos.action"
   "action/MovRelPos.action" 
   "action/ReleaseWorkpiece.action" 
 )

## Generate added messages and services with any dependencies listed here
 #generate_messages(
 #  DEPENDENCIES
 #  actionlib_msgs   std_msgs
 #)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#  cfg/gripper_parameter.cfg
# )

#catkin_package(
#   INCLUDE_DIRS include
  # LIBRARIES schunk_gripper
#   CATKIN_DEPENDS actionlib actionlib_msgs message_runtime roscpp std_msgs
#)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
${Boost_INCLUDE_DIRS} 
${CURL_INCLUDE_DIR}
)


## Declare a C++ library
add_library( schunk_driverlib
src/schunk_gripper_lib.cpp
src/communication.cpp
)

#add_executable(schunk_gripper_driver src/schunk_gripper_wrapper.cpp)
#add_dependencies(schunk_gripper_driver schunk_gripper_generate_messages_cpp)
#add_dependencies(schunk_gripper_driver schunk_gripper_gencpp)
#add_dependencies(schunk_gripper_driver ${PROJECT_NAME}_gencfg)
#target_link_libraries(schunk_gripper_driver schunk_driverlib ${catkin_LIBRARIES} curl)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES sensor_msgs control_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(schunk_gripper_driver src/schunk_gripper_wrapper.cpp)

ament_target_dependencies(schunk_gripper_driver rclcpp sensor_msgs control_msgs rclcpp_action rclcpp_components)
target_link_libraries(schunk_gripper_driver ${Boost_LIBRARIES} ${CURL_LIBRARIES} schunk_driverlib -lcurl)

install(TARGETS
  schunk_gripper_driver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  include/
  DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(sensor_msgs control_msgs action_msgs)

  rosidl_target_interfaces(schunk_gripper_driver
  ${PROJECT_NAME} 
  "rosidl_typesupport_cpp")



##add_executable(egk_example src/client_egk.cpp)
##add_dependencies(egk_example schunk_gripper_generate_messages_cpp)
##add_dependencies(egk_example schunk_gripper_gencpp)
##add_dependencies(egk_example ${PROJECT_NAME}_gencfg)
#target_link_libraries(egk_example ${catkin_LIBRARIES})

#add_executable(egu_example src/client_egu.cpp)
#add_dependencies(egu_example schunk_gripper_generate_messages_cpp)
#add_dependencies(egu_example schunk_gripper_gencpp)
#add_dependencies(egu_example ${PROJECT_NAME}_gencfg)
#target_link_libraries(egu_example ${catkin_LIBRARIES})


ament_package()


