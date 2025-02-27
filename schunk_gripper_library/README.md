# SCHUNK Gripper Library
A Low-level communication library for Modbus RTU and Ethernet-based SCHUNK grippers.


## Dependencies
We need additional Python dependencies.
Install them inside your favorite Python environment with

```bash
pip install --user pymodbus pyserial
```

## Run tests locally
We use `socat` to set-up pseudo terminals for a connection between the driver library and a minimalist Modbus server.
Install that with
```bash
sudo apt-get install socat
```

Additionally, inside the dummy's package, install test-related dependencies with

```bash
pip install --user pytest httpx coverage
```

You can run the tests either directly with `pytest tests/` or with more output through

```bash
coverage run -m pytest tests/
coverage report  # for console output
coverage html    # for web-based output
```
