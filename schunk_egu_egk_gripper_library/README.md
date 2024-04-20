# Schunk EGU/EGK Gripper Library

This is the C++ driver library for SCHUNK EGU/EGK grippers.
It implements the core functionality of the communication for usage in non-ROS2 C++ frameworks.

## Standalone build and usage

Get the `schunk_egu_egk_gripper` following [these instructions](../README.md#build-and-install).
Navigate into the `schunk_egu_egk_gripper_library` and call

```bash
mkdir build && cd "$_"
cmake ..
make
```
to build the library with plain *CMake*.
You can then include this library in the *CMakeLists.txt* of your application with the usual functionality:
```cmake
cmake_minimum_required(VERSION 3.22)
project(your_application)

# Find the library as a dependency
find_package(schunk_egu_egk_gripper_library REQUIRED)

# Link your application against the library
target_link_libraries(your_application
  Schunk::schunk_egu_egk_gripper_library
)

```
