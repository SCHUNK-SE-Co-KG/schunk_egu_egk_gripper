# Launching the Driver

If you installed the Python dependencies in a virtual environment, activate it via `source .venv/bin/activate` before launching the driver.

### Launch Options

- **No arguments (default):**  
  Starts the driver with an empty configuration. The driver remains in the `unconfigured` state, allowing you to scan the network and add grippers individually using the provided services.

  ```bash
  ros2 launch schunk_gripper_driver driver.launch.py
  ```

- **Specify IP or Modbus address:**  
  Connect to a specific gripper immediately on startup.

  - **Modbus:**

    ```bash
    ros2 launch schunk_gripper_driver driver.launch.py headless:=false serial_port:=/dev/ttyUSB0 device_id:=13
    ```

  - **Ethernet:**

    ```bash
    ros2 launch schunk_gripper_driver driver.launch.py headless:=false host:=192.168.0.100 port:=80
    ```

  Adjust the parameters above to match your gripper’s serial port, device ID, or network settings.

- **Headless mode:**  
  In headless mode the driver loads all previously saved grippers from the configuration file and automatically transitions them to the `active` lifecycle state.

  ```bash
  ros2 launch schunk_gripper_driver driver.launch.py headless:=true
  ```