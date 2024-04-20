cmake_minimum_required(VERSION 3.22)
project(schunk_egu_egk_gripper_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Pytest REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  pytest_discover_tests(
      PythonTest
      TRIM_FROM_NAME "^test_"
  )
endif()

ament_package()
