cmake_minimum_required(VERSION 3.22)
project(schunk_egu_egk_gripper_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # See: https://docs.ros.org/en/rolling/How-To-Guides/Ament-CMake-Python-Documentation.html
  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests
    test/test_launch.py
    test/test_topics.py
    #test/test_services.py
    #test/test_actions.py
  )
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()

endif()

ament_package()
