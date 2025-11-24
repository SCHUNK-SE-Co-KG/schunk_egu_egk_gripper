# Testing

All tests are conducted on real hardware. The required hardware setup is specified in the configuration file [here](../schunk_gripper_library/schunk_gripper_library/tests/config.yaml).  

These tests are intended for internal developers to verify driver functionality on actual hardware before merging changes.

To run the test suite, navigate to the project folder and execute:

```bash
python3 -m pytest schunk_gripper_library/schunk_gripper_library/tests
```

Alternatively, the tests can be run using the **VS Code Test Explorer extension** (recommended), which provides fine-grained control and built-in debugging support.
