## Run tests locally
Inside the driver's package

```bash
pip install --user pytest coverage
```

```bash
coverage run -m pytest test/
coverage report
```

## Run individual tests
You can test individual functions, e.g. with
```bash
pytest test_driver::<function-name>
```

Especially the ROS2-related functions may block eternally due to subsequent fails.
Unfortunately, hitting `ctrl-C` will discard useful output for this case.

To catch the first failure message, use
```bash
pytest --maxfail=1 <some-test>
```
