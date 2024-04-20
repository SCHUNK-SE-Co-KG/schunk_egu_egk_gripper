# Schunk EGU/EGK Gripper Tests

## Additional dependencies
```bash
pip install --user --ignore-installed --upgrade pytest==6.2.5 pytest-cmake
```

## Running tests manually
In the root of a sourced workspace, call
```bash
colcon test --packages-select schunk_egu_egk_gripper_tests && colcon test-result --verbose
```
You can clean the test results and outdated errors with
```bash
colcon test-result --delete-yes
```

For more output on failing tests, call them directly inside the `test` folder, e.g. with
```bash
pytest test_*.py
```
