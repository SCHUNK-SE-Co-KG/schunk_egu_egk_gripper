# SCHUNK Gripper Library
A Low-level communication library for Modbus RTU and Ethernet-based SCHUNK grippers.


## Run tests locally
Inside the dummy's package

```bash
pip install --user pytest httpx coverage
```

```bash
coverage run -m pytest tests/
coverage report
```
