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

# --------------------------------------------------------------------------------
# Make sure that library relocation works for both build and install.
# See here: https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/RPATH-handling
# --------------------------------------------------------------------------------

set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# The RPATH to use when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif("${isSystemDir}" STREQUAL "-1")

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

install(TARGETS
  ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Create and install a file with all exported targets
install(EXPORT ${PROJECT_NAME}_targets
  DESTINATION lib/cmake/${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE Schunk::
)

# Install the Config file so that find_package() works with this package
install(FILES ${PROJECT_NAME}Config.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

# Install the include directories
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
)

#--------------------------------------------------------------------------------
# Test
#--------------------------------------------------------------------------------
enable_testing()

