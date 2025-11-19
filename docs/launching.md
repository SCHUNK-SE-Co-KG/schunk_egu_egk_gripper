## Launching the Driver

If you installed the Python dependencies in a virtual environment, activate it first:
```bash
source .venv/bin/activate
```

Then launch the driver:
```bash
ros2 launch schunk_gripper_driver driver.launch.py
```

### Launch Options

- **No arguments:**
  Starts the driver with an empty configuration. The driver remains in the `unconfigured` state, allowing you to scan the network and add grippers individually using the provided services.

- **Specify IP or Modbus address:**
  Pass connection parameters via launch arguments to immediately connect to specific grippers on startup.

- **Headless mode:**
  In headless mode the driver loads all previously saved grippers from the configuration file and automatically transitions them to the `active` lifecycle state.
