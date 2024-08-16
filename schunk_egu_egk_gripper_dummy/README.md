# Schunk EGU/EGK Dummy
A minimalist protocol simulator for system tests.

## Dependencies

```bash
pip install --user fastapi uvicorn
```

## Getting started
1. Start the dummy standalone with
    ```bash
    ./start_dummy.sh
    ```

## Run tests locally

```bash
pip install --user pytest httpx coverage
```

```bash
coverage run -m pytest tests/
coverage report
```
