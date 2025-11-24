# Testing

All tests are conducted on real hardware. The required hardware setup is specified in the configuration file. By default, the project includes a **default configuration** file at:

[config.default.yaml](../schunk_gripper_library/schunk_gripper_library/tests/config.default.yaml)

**DO NOT MODIFY `config.default.yaml` directly.**  

If you wish to customize the configuration for your environment:

1. Copy `config.default.yaml` to `config.yaml` in the same folder.  
2. Modify `config.yaml` as needed.  
3. The test suite will use `config.yaml` if it exists; otherwise, it will fall back to `config.default.yaml`.

These tests are intended for internal developers to verify driver functionality on actual hardware before merging changes.

To run the test suite, navigate to the project folder and execute:

```bash
python3 -m pytest schunk_gripper_library/schunk_gripper_library/tests
```

Alternatively, the tests can be run using the **VS Code Test Explorer extension** (recommended), which provides fine-grained control and built-in debugging support.
