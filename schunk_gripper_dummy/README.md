# Schunk EGU/EGK Dummy
A minimalist protocol simulator for system tests.

Use this SCHUNK gripper dummy whenever you don't have access to real hardware
and still want to test your application.

## Getting started
Use the `start_dummy` script for starting the dummy simulator.
It will start on localhost with port `8000` by default but you can specify another port via the `port:=<port-id>` syntax.

## Installation with plain python

Although shipped inside a ROS2 package, the dummy itself doesn't need ROS2.
You can simply navigate into the dummy's package, install it inside your favorite environment with

```bash
pip install .
```
Now you can start the simulator anywhere your environment with
```bash
start_dummy port:=8000
```
You can inspect and uninstall it with normal pip functionality
```bash
pip show schunk_gripper_dummy
pip uninstall schunk_gripper_dummy
```

## Installation with ROS2
Install them inside your favorite Python environment with

```bash
pip install --user fastapi uvicorn requests python-multipart
```

When working in a sourced ROS2 environment, you can start the dummy with
```bash
ros2 run schunk_gripper_dummy start_dummy --ros-args -p port:=8000
```
