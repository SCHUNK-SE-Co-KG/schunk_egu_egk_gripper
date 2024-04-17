cmake_minimum_required(VERSION 3.22)
project(schunk_egu_egk_gripper_library)

# Set C++ standard to 17 if not set
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler-specific options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(CURL REQUIRED)

#--------------------------------------------------------------------------------
# Build
#--------------------------------------------------------------------------------

add_library(${PROJECT_NAME}
  SHARED
  src/schunk_gripper_lib.cpp
  src/communication.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CURL_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME}
  ${CURL_LIBRARIES}
)

#--------------------------------------------------------------------------------
# Install
#--------------------------------------------------------------------------------

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
)

#--------------------------------------------------------------------------------
# Test
#--------------------------------------------------------------------------------
enable_testing()

