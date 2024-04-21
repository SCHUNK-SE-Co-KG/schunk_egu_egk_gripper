# Schunk EGU/EGK Gripper Tests

## Running tests manually
In the root of a sourced workspace, call
```bash
colcon build --packages-select schunk_egu_egk_gripper_tests
colcon test --packages-select schunk_egu_egk_gripper_tests && colcon test-result --verbose
```
You can clean the test results and outdated errors with
```bash
colcon test-result --delete-yes
```

For more output on failing tests, source your workspace
```bash
source install/setup.bash
```
and call them directly inside the `test` folder, e.g. with
```bash
pytest-3 test_*.py
```
