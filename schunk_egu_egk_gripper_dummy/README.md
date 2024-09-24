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
- **Option 1**: Although shipped inside a ROS2 package, the dummy itself doesn't need ROS2.
   You can simply navigate into the dummy's package and start it with
    ```bash
    ./start_dummy
    ```

- **Option 2**: When working in a sourced environment, you can start the dummy with
   ```bash
   ros2 run schunk_egu_egk_gripper_dummy start_dummy
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
