# Schunk EGU/EGK Dummy
A minimalist protocol simulator for system tests.

## Dependencies

```bash
pip install fastapi uvicorn
```

## Getting started
1. Start the dummy standalone with
    ```bash
    uvicorn schunk_egu_egk_gripper_dummy.main:server --port 8000 --reload
    ```

## Run tests locally

```bash
pip install pytest httpx coverage
```

```bash
coverage run -m pytest tests/
coverage report
```
