# Schunk EGU/EGK Dummy
A minimalist protocol simulator for system tests.

Use this SCHUNK gripper dummy whenever you don't have access to real hardware
and still want to test your application.

## Dependencies
We need additional Python dependencies.
Install them inside your favorite Python environment with

```bash
pip install --user fastapi uvicorn requests python-multipart
```

## Getting started
There's a convenience script `start_dummy` for starting the dummy simulator.
It will start on localhost with port `8000` by default but you can specify another port via the `port:=<port-id>` syntax.

### Plain python

Although shipped inside a ROS2 package, the dummy itself doesn't need ROS2.
You can simply navigate into the dummy's package and start it with
```bash
./start_dummy port:=8000
```

### ROS2
When working in a sourced ROS2 environment, you can start the dummy with
```bash
ros2 run schunk_egu_egk_gripper_dummy start_dummy --ros-args -p port:=8000
```

## Run tests locally
Inside the dummy's package

```bash
pip install --user pytest httpx coverage
```

```bash
coverage run -m pytest tests/
coverage report
```
